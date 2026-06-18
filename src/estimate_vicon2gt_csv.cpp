/*
 * Offline CSV front-end for vicon2gt.
 */

#include <Eigen/Geometry>

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <boost/filesystem.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "meas/Interpolator.h"
#include "meas/Propagator.h"
#include "solver/ViconGraphSolver.h"

namespace {

struct Options {
  std::string imu_csv_path;
  std::string vicon_csv_path;
  std::string output_states_path;
  std::string output_info_path;
  std::string trajectory_image_path;
  std::string timestamps_csv_path;
  double state_freq_hz = 100.0;
  double sigma_w = 1.6968e-04;
  double sigma_a = 2.0000e-03;
  double sigma_wb = 1.9393e-05;
  double sigma_ab = 3.0000e-03;
  Eigen::Matrix3d R_q = Eigen::Matrix3d::Identity() * std::pow(1e-4, 2);
  Eigen::Matrix3d R_p = Eigen::Matrix3d::Identity() * std::pow(1e-5, 2);
  ViconGraphSolver::Options solver_options;
};

struct ImuSample {
  double timestamp_s = 0.0;
  Eigen::Vector3d wm = Eigen::Vector3d::Zero();
  Eigen::Vector3d am = Eigen::Vector3d::Zero();
};

struct ViconSample {
  double timestamp_s = 0.0;
  Eigen::Vector4d q = Eigen::Vector4d(0.0, 0.0, 0.0, 1.0);
  Eigen::Vector3d p = Eigen::Vector3d::Zero();
};

struct CleanViconColumns {
  std::size_t timestamp = 0;
  bool timestamp_is_ns = false;
  std::size_t x_m = 0;
  std::size_t y_m = 0;
  std::size_t z_m = 0;
  std::size_t qx = 0;
  std::size_t qy = 0;
  std::size_t qz = 0;
  std::size_t qw = 0;
};

std::string trim_ascii(const std::string &value) {
  std::size_t start = 0;
  while (start < value.size() && std::isspace(static_cast<unsigned char>(value[start])) != 0) {
    ++start;
  }

  std::size_t end = value.size();
  while (end > start && std::isspace(static_cast<unsigned char>(value[end - 1])) != 0) {
    --end;
  }
  return value.substr(start, end - start);
}

std::string strip_utf8_bom(std::string value) {
  if (value.size() >= 3 && static_cast<unsigned char>(value[0]) == 0xEF && static_cast<unsigned char>(value[1]) == 0xBB &&
      static_cast<unsigned char>(value[2]) == 0xBF) {
    value.erase(0, 3);
  }
  return value;
}

bool parse_double_strict(const std::string &text, double &value) {
  const std::string trimmed = trim_ascii(text);
  if (trimmed.empty()) {
    return false;
  }

  char *end = nullptr;
  value = std::strtod(trimmed.c_str(), &end);
  return end != nullptr && *end == '\0';
}

double parse_double_or_throw(const std::string &text, const std::string &context) {
  double value = 0.0;
  if (!parse_double_strict(text, value)) {
    throw std::runtime_error("failed to parse " + context + " from '" + text + "'");
  }
  return value;
}

std::vector<std::string> split_csv_line(const std::string &raw_line) {
  std::vector<std::string> fields;
  std::string current;
  bool in_quotes = false;
  for (std::size_t index = 0; index < raw_line.size(); ++index) {
    const char ch = raw_line[index];
    if (ch == '"') {
      if (in_quotes && index + 1 < raw_line.size() && raw_line[index + 1] == '"') {
        current.push_back('"');
        ++index;
      } else {
        in_quotes = !in_quotes;
      }
      continue;
    }
    if (ch == ',' && !in_quotes) {
      fields.push_back(current);
      current.clear();
      continue;
    }
    if (ch != '\r') {
      current.push_back(ch);
    }
  }
  fields.push_back(current);
  if (!fields.empty()) {
    fields.front() = strip_utf8_bom(fields.front());
  }
  return fields;
}

std::optional<std::size_t> find_first_matching_column(const std::vector<std::string> &header, const std::vector<std::string> &candidates) {
  for (const std::string &candidate : candidates) {
    for (std::size_t index = 0; index < header.size(); ++index) {
      if (trim_ascii(header[index]) == candidate) {
        return index;
      }
    }
  }
  return std::nullopt;
}

void print_usage() {
  std::cerr << "Usage: estimate_vicon2gt_csv --imu-csv <path> --vicon-csv <path>\n"
            << "                             --output-states <path> [options]\n\n"
            << "Options:\n"
            << "  --output-info <path>           Optional calibration/info output text file\n"
            << "  --trajectory-image <path>      Optional PNG/JPEG gravity-aligned top-down trajectory plot\n"
            << "  --timestamps-csv <path>        Optional CSV of output timestamps in the IMU clock\n"
            << "  --state-freq <hz>              Output rate when no timestamps CSV is given (default: 100)\n"
            << "  --imu-time-offset-s <sec>      Initial IMU->Vicon offset guess (default: 0)\n"
            << "  --gravity-magnitude <m/s^2>    Gravity magnitude (default: 9.81)\n"
            << "  --no-estimate-time-offset      Keep IMU/Vicon time offset fixed\n"
            << "  --no-estimate-orientation      Keep Vicon-body to IMU rotation fixed at identity\n"
            << "  --no-estimate-position         Keep Vicon-body to IMU translation fixed at zero\n"
            << "  --num-loop-relin <count>       Relinearization loops (default: 0)\n"
            << "  --help                         Show this message\n";
}

Options parse_options(int argc, char **argv) {
  Options options;

  auto require_value = [&](int &index, const std::string &name) -> std::string {
    if (index + 1 >= argc) {
      throw std::runtime_error("missing value for " + name);
    }
    ++index;
    return argv[index];
  };

  for (int index = 1; index < argc; ++index) {
    const std::string arg = argv[index];
    if (arg == "--help" || arg == "-h") {
      print_usage();
      std::exit(EXIT_SUCCESS);
    } else if (arg == "--imu-csv") {
      options.imu_csv_path = require_value(index, arg);
    } else if (arg == "--vicon-csv") {
      options.vicon_csv_path = require_value(index, arg);
    } else if (arg == "--output-states") {
      options.output_states_path = require_value(index, arg);
    } else if (arg == "--output-info") {
      options.output_info_path = require_value(index, arg);
    } else if (arg == "--trajectory-image") {
      options.trajectory_image_path = require_value(index, arg);
    } else if (arg == "--timestamps-csv") {
      options.timestamps_csv_path = require_value(index, arg);
    } else if (arg == "--state-freq") {
      options.state_freq_hz = parse_double_or_throw(require_value(index, arg), arg);
    } else if (arg == "--imu-time-offset-s") {
      options.solver_options.init_toff_imu_to_vicon = parse_double_or_throw(require_value(index, arg), arg);
    } else if (arg == "--gravity-magnitude") {
      options.solver_options.gravity_magnitude = parse_double_or_throw(require_value(index, arg), arg);
    } else if (arg == "--num-loop-relin") {
      options.solver_options.num_loop_relin = static_cast<int>(std::llround(parse_double_or_throw(require_value(index, arg), arg)));
    } else if (arg == "--no-estimate-time-offset") {
      options.solver_options.gtsam_config.estimate_vicon_imu_toff = false;
    } else if (arg == "--no-estimate-orientation") {
      options.solver_options.gtsam_config.estimate_vicon_imu_ori = false;
    } else if (arg == "--no-estimate-position") {
      options.solver_options.gtsam_config.estimate_vicon_imu_pos = false;
    } else {
      throw std::runtime_error("unknown option: " + arg);
    }
  }

  if (options.imu_csv_path.empty() || options.vicon_csv_path.empty() || options.output_states_path.empty()) {
    print_usage();
    throw std::runtime_error("--imu-csv, --vicon-csv, and --output-states are required");
  }
  if (options.output_info_path.empty()) {
    const boost::filesystem::path states_path(options.output_states_path);
    const std::string stem = states_path.stem().empty() ? std::string("vicon2gt_states") : states_path.stem().string();
    options.output_info_path = (states_path.parent_path() / (stem + "_info.txt")).string();
  }
  if (options.state_freq_hz <= 0.0) {
    throw std::runtime_error("state frequency must be positive");
  }
  return options;
}

std::vector<ImuSample> load_imu_csv(const std::string &path) {
  std::ifstream stream(path);
  if (!stream.is_open()) {
    throw std::runtime_error("failed to open IMU CSV: " + path);
  }

  std::string line;
  if (!std::getline(stream, line)) {
    throw std::runtime_error("IMU CSV is empty: " + path);
  }
  std::vector<std::string> header = split_csv_line(line);
  for (std::string &field : header) {
    field = trim_ascii(field);
  }

  const auto timestamp_s_column = find_first_matching_column(header, {"timestamp_s"});
  const auto timestamp_ns_column = find_first_matching_column(header, {"timestamp_ns"});
  const auto gyro_x_column = find_first_matching_column(header, {"gyro_x", "gx", "angular_velocity_x", "angular_velocity_rad_s.x"});
  const auto gyro_y_column = find_first_matching_column(header, {"gyro_y", "gy", "angular_velocity_y", "angular_velocity_rad_s.y"});
  const auto gyro_z_column = find_first_matching_column(header, {"gyro_z", "gz", "angular_velocity_z", "angular_velocity_rad_s.z"});
  const auto accel_x_column = find_first_matching_column(header, {"accel_x", "ax", "linear_acceleration_x", "linear_acceleration_m_s2.x"});
  const auto accel_y_column = find_first_matching_column(header, {"accel_y", "ay", "linear_acceleration_y", "linear_acceleration_m_s2.y"});
  const auto accel_z_column = find_first_matching_column(header, {"accel_z", "az", "linear_acceleration_z", "linear_acceleration_m_s2.z"});

  if ((!timestamp_s_column && !timestamp_ns_column) || !gyro_x_column || !gyro_y_column || !gyro_z_column || !accel_x_column ||
      !accel_y_column || !accel_z_column) {
    throw std::runtime_error("IMU CSV must contain timestamp plus gyro/accel xyz columns");
  }

  std::vector<ImuSample> samples;
  std::size_t line_number = 1;
  while (std::getline(stream, line)) {
    ++line_number;
    if (trim_ascii(line).empty()) {
      continue;
    }
    const std::vector<std::string> fields = split_csv_line(line);
    const auto read_field = [&](std::size_t column, const std::string &name) -> double {
      if (column >= fields.size()) {
        throw std::runtime_error("IMU CSV line " + std::to_string(line_number) + " missing column " + name);
      }
      return parse_double_or_throw(fields[column], "IMU " + name);
    };

    double timestamp_s = 0.0;
    if (timestamp_s_column) {
      timestamp_s = read_field(*timestamp_s_column, header[*timestamp_s_column]);
    } else {
      timestamp_s = read_field(*timestamp_ns_column, header[*timestamp_ns_column]) * 1e-9;
    }

    ImuSample sample;
    sample.timestamp_s = timestamp_s;
    sample.wm << read_field(*gyro_x_column, header[*gyro_x_column]), read_field(*gyro_y_column, header[*gyro_y_column]),
        read_field(*gyro_z_column, header[*gyro_z_column]);
    sample.am << read_field(*accel_x_column, header[*accel_x_column]), read_field(*accel_y_column, header[*accel_y_column]),
        read_field(*accel_z_column, header[*accel_z_column]);
    samples.push_back(sample);
  }

  if (samples.empty()) {
    throw std::runtime_error("IMU CSV contains no samples: " + path);
  }
  return samples;
}

CleanViconColumns find_clean_vicon_columns(const std::vector<std::string> &header_row) {
  const auto timestamp_s = find_first_matching_column(header_row, {"timestamp_s"});
  const auto timestamp_ns = find_first_matching_column(header_row, {"timestamp_ns"});
  const auto x_m = find_first_matching_column(header_row, {"x_m", "x"});
  const auto y_m = find_first_matching_column(header_row, {"y_m", "y"});
  const auto z_m = find_first_matching_column(header_row, {"z_m", "z"});
  const auto qx = find_first_matching_column(header_row, {"qx"});
  const auto qy = find_first_matching_column(header_row, {"qy"});
  const auto qz = find_first_matching_column(header_row, {"qz"});
  const auto qw = find_first_matching_column(header_row, {"qw"});
  if ((!timestamp_s && !timestamp_ns) || !x_m || !y_m || !z_m || !qx || !qy || !qz || !qw) {
    throw std::runtime_error("cleaned Vicon CSV is missing one or more required columns");
  }

  CleanViconColumns columns;
  if (timestamp_s) {
    columns.timestamp = *timestamp_s;
  } else {
    columns.timestamp = *timestamp_ns;
    columns.timestamp_is_ns = true;
  }
  columns.x_m = *x_m;
  columns.y_m = *y_m;
  columns.z_m = *z_m;
  columns.qx = *qx;
  columns.qy = *qy;
  columns.qz = *qz;
  columns.qw = *qw;
  return columns;
}

std::vector<ViconSample> load_vicon_csv(const std::string &path) {
  std::ifstream stream(path);
  if (!stream.is_open()) {
    throw std::runtime_error("failed to open Vicon CSV: " + path);
  }

  std::string line;
  if (!std::getline(stream, line)) {
    throw std::runtime_error("Vicon CSV is empty: " + path);
  }

  std::vector<std::string> header_row = split_csv_line(line);
  for (std::string &field : header_row) {
    field = trim_ascii(field);
  }
  const CleanViconColumns columns = find_clean_vicon_columns(header_row);

  std::vector<ViconSample> samples;
  std::size_t line_number = 1;
  while (std::getline(stream, line)) {
    ++line_number;
    if (trim_ascii(line).empty()) {
      continue;
    }

    const std::vector<std::string> fields = split_csv_line(line);
    const auto read_field = [&](std::size_t column, const std::string &name) -> double {
      if (column >= fields.size()) {
        throw std::runtime_error("Vicon CSV line " + std::to_string(line_number) + " missing column " + name);
      }
      return parse_double_or_throw(fields[column], "Vicon " + name);
    };

    Eigen::Quaterniond q(read_field(columns.qw, "qw"), read_field(columns.qx, "qx"), read_field(columns.qy, "qy"),
                         read_field(columns.qz, "qz"));
    q.normalize();

    ViconSample sample;
    sample.timestamp_s = read_field(columns.timestamp, columns.timestamp_is_ns ? "timestamp_ns" : "timestamp_s");
    if (columns.timestamp_is_ns) {
      sample.timestamp_s *= 1e-9;
    }
    sample.p << read_field(columns.x_m, "x_m"), read_field(columns.y_m, "y_m"), read_field(columns.z_m, "z_m");
    sample.q << q.x(), q.y(), q.z(), q.w();
    samples.push_back(sample);
  }

  if (samples.size() < 2) {
    throw std::runtime_error("Vicon CSV needs at least two samples: " + path);
  }
  return samples;
}

std::vector<double> load_output_timestamps(const Options &options, const std::vector<ImuSample> &imu_samples,
                                           const std::vector<ViconSample> &vicon_samples) {
  if (options.timestamps_csv_path.empty()) {
    std::vector<double> timestamps;
    const double start_time_s = vicon_samples.front().timestamp_s;
    const double end_time_s = vicon_samples.back().timestamp_s;
    for (double timestamp_s = start_time_s; timestamp_s <= end_time_s; timestamp_s += 1.0 / options.state_freq_hz) {
      timestamps.push_back(timestamp_s);
    }
    return timestamps;
  }

  std::ifstream stream(options.timestamps_csv_path);
  if (!stream.is_open()) {
    throw std::runtime_error("failed to open timestamps CSV: " + options.timestamps_csv_path);
  }

  std::string line;
  if (!std::getline(stream, line)) {
    throw std::runtime_error("timestamps CSV is empty: " + options.timestamps_csv_path);
  }
  std::vector<std::string> header = split_csv_line(line);
  for (std::string &field : header) {
    field = trim_ascii(field);
  }

  const auto timestamp_s_column = find_first_matching_column(header, {"timestamp_s"});
  const auto timestamp_ns_column = find_first_matching_column(header, {"timestamp_ns"});
  if (!timestamp_s_column && !timestamp_ns_column) {
    throw std::runtime_error("timestamps CSV is missing a supported timestamp column");
  }

  std::vector<double> timestamps;
  while (std::getline(stream, line)) {
    if (trim_ascii(line).empty()) {
      continue;
    }
    const std::vector<std::string> fields = split_csv_line(line);
    if (timestamp_s_column) {
      if (*timestamp_s_column >= fields.size()) {
        throw std::runtime_error("timestamps CSV row missing timestamp column");
      }
      timestamps.push_back(parse_double_or_throw(fields[*timestamp_s_column], "timestamp_s"));
    } else {
      if (*timestamp_ns_column >= fields.size()) {
        throw std::runtime_error("timestamps CSV row missing timestamp_ns column");
      }
      timestamps.push_back(parse_double_or_throw(fields[*timestamp_ns_column], "timestamp_ns") * 1e-9);
    }
  }

  if (timestamps.empty()) {
    throw std::runtime_error("timestamps CSV contains no timestamps: " + options.timestamps_csv_path);
  }
  (void)imu_samples;
  return timestamps;
}

std::vector<cv::Point2d> estimate_vicon_xy_trajectory(ViconGraphSolver &solver) {
  std::vector<double> times;
  std::vector<Eigen::Matrix<double, 10, 1>> poses;
  solver.get_imu_poses(times, poses);

  std::vector<cv::Point2d> points;
  points.reserve(poses.size());
  for (const Eigen::Matrix<double, 10, 1> &pose : poses) {
    const Eigen::Vector3d position = pose.block<3, 1>(4, 0);
    points.emplace_back(position.x(), position.y());
  }
  return points;
}

std::vector<cv::Point2d> raw_vicon_body_xy_trajectory(const std::vector<ViconSample> &vicon_samples) {
  std::vector<cv::Point2d> points;
  points.reserve(vicon_samples.size());
  for (const ViconSample &sample : vicon_samples) {
    points.emplace_back(sample.p.x(), sample.p.y());
  }
  return points;
}

void render_trajectory_image(const std::string &path, const std::vector<cv::Point2d> &estimated_points,
                             const std::vector<cv::Point2d> &vicon_body_points, const std::string &estimated_label) {
  if (path.empty()) {
    return;
  }
  if (estimated_points.size() < 2 || vicon_body_points.size() < 2) {
    throw std::runtime_error("need at least two estimated and raw Vicon-body points to render a trajectory image");
  }

  double min_x = std::numeric_limits<double>::infinity();
  double min_y = std::numeric_limits<double>::infinity();
  double max_x = -std::numeric_limits<double>::infinity();
  double max_y = -std::numeric_limits<double>::infinity();
  const auto is_finite_point = [](const cv::Point2d &point) { return std::isfinite(point.x) && std::isfinite(point.y); };
  const auto accumulate_bounds = [&](const std::vector<cv::Point2d> &points) {
    for (const cv::Point2d &point : points) {
      if (!is_finite_point(point)) {
        continue;
      }
      min_x = std::min(min_x, point.x);
      min_y = std::min(min_y, point.y);
      max_x = std::max(max_x, point.x);
      max_y = std::max(max_y, point.y);
    }
  };
  accumulate_bounds(estimated_points);
  accumulate_bounds(vicon_body_points);

  constexpr int kImageSizePx = 1200;
  constexpr int kMarginPx = 80;
  const double span_x = std::max(max_x - min_x, 1e-6);
  const double span_y = std::max(max_y - min_y, 1e-6);
  const double scale =
      std::min(static_cast<double>(kImageSizePx - 2 * kMarginPx) / span_x, static_cast<double>(kImageSizePx - 2 * kMarginPx) / span_y);

  auto to_pixel = [&](const cv::Point2d &point) {
    const int x = static_cast<int>(std::llround(kMarginPx + (point.x - min_x) * scale));
    const int y = static_cast<int>(std::llround(kImageSizePx - kMarginPx - (point.y - min_y) * scale));
    return cv::Point(x, y);
  };

  cv::Mat image(kImageSizePx, kImageSizePx, CV_8UC3, cv::Scalar(250, 250, 250));
  cv::rectangle(image, cv::Rect(kMarginPx, kMarginPx, kImageSizePx - 2 * kMarginPx, kImageSizePx - 2 * kMarginPx),
                cv::Scalar(220, 220, 220), 1);

  const auto draw_path = [&](const std::vector<cv::Point2d> &points, const cv::Scalar &color, const std::string &label, int legend_y) {
    std::optional<cv::Point2d> first_valid;
    std::optional<cv::Point2d> previous_valid;
    cv::Point2d last_valid;
    for (const cv::Point2d &point : points) {
      if (!is_finite_point(point)) {
        previous_valid.reset();
        continue;
      }
      if (!first_valid.has_value()) {
        first_valid = point;
      }
      last_valid = point;
      if (previous_valid.has_value()) {
        cv::line(image, to_pixel(*previous_valid), to_pixel(point), color, 2, cv::LINE_AA);
      }
      previous_valid = point;
    }
    if (first_valid.has_value()) {
      cv::circle(image, to_pixel(*first_valid), 6, color, cv::FILLED, cv::LINE_AA);
      cv::circle(image, to_pixel(last_valid), 4, color, 2, cv::LINE_AA);
    }
    cv::line(image, cv::Point(40, legend_y), cv::Point(90, legend_y), color, 3, cv::LINE_AA);
    cv::putText(image, label, cv::Point(105, legend_y + 7), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(30, 30, 30), 2, cv::LINE_AA);
  };

  draw_path(vicon_body_points, cv::Scalar(185, 110, 40), "Raw Vicon body XY", 48);
  draw_path(estimated_points, cv::Scalar(45, 120, 30), estimated_label, 82);

  cv::putText(image, "Top-down Vicon-XY trajectory comparison", cv::Point(40, kImageSizePx - 28), cv::FONT_HERSHEY_SIMPLEX, 0.75,
              cv::Scalar(40, 40, 40), 2, cv::LINE_AA);

  if (!cv::imwrite(path, image)) {
    throw std::runtime_error("failed to write trajectory image: " + path);
  }
}

} // namespace

int main(int argc, char **argv) {
  try {
    const Options options = parse_options(argc, argv);
    const ViconGraphSolver::ExportOptions export_options;

    const std::vector<ImuSample> imu_samples = load_imu_csv(options.imu_csv_path);
    const std::vector<ViconSample> vicon_samples = load_vicon_csv(options.vicon_csv_path);
    std::vector<double> output_timestamps = load_output_timestamps(options, imu_samples, vicon_samples);

    std::cout << "Loaded IMU samples: " << imu_samples.size() << " range=[" << std::fixed << std::setprecision(6)
              << imu_samples.front().timestamp_s << ", " << imu_samples.back().timestamp_s << "] s\n";
    std::cout << "Loaded Vicon samples: " << vicon_samples.size() << " range=[" << vicon_samples.front().timestamp_s << ", "
              << vicon_samples.back().timestamp_s << "] s\n";
    std::cout << "Requested output timestamps: " << output_timestamps.size() << '\n';

    auto propagator = std::make_shared<Propagator>(options.sigma_w, options.sigma_wb, options.sigma_a, options.sigma_ab);
    auto interpolator = std::make_shared<Interpolator>();

    for (const ImuSample &sample : imu_samples) {
      propagator->feed_imu(sample.timestamp_s, sample.wm, sample.am);
    }
    for (const ViconSample &sample : vicon_samples) {
      interpolator->feed_pose(sample.timestamp_s, sample.q, sample.p, options.R_q, options.R_p);
    }

    ViconGraphSolver solver(options.solver_options, propagator, interpolator, std::move(output_timestamps));
    solver.build_and_solve();
    solver.write_to_file(options.output_states_path, options.output_info_path, export_options);
    if (!options.trajectory_image_path.empty()) {
      render_trajectory_image(options.trajectory_image_path, estimate_vicon_xy_trajectory(solver),
                              raw_vicon_body_xy_trajectory(vicon_samples), "Optimized IMU in Vicon XY");
    }

    std::cout << "Wrote states to: " << options.output_states_path << '\n';
    std::cout << "Wrote info to:   " << options.output_info_path << '\n';
    if (!options.trajectory_image_path.empty()) {
      std::cout << "Wrote image to:  " << options.trajectory_image_path << '\n';
    }
    return EXIT_SUCCESS;
  } catch (const std::exception &e) {
    std::cerr << "[estimate_vicon2gt_csv] " << e.what() << '\n';
    return EXIT_FAILURE;
  }
}