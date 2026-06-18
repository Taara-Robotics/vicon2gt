# vicon2gt

This utility was created to generate groundtruth trajectories using a motion capture system (e.g. Vicon or OptiTrack) for use in evaluating visual-inertial estimation systems.
Specifically we want to calculate the inertial IMU state (full 15 dof) at camera frequency rate and generate a groundtruth trajectory similar to those provided by the EurocMav datasets.
Please take a look at the **[REPORT](docs/tr_vicon2gt.pdf)** for specific details and a more formal description.

This fork is intentionally CSV-only. It reads IMU and Vicon measurements from CSV files and writes a groundtruth trajectory plus calibration output without any ROS dependency.

## CSV Workflow

You can either build locally with CMake or use the provided Docker image to avoid installing toolchain dependencies on the host.

Build the CSV executable with plain CMake:

```cmd
mkdir -p build
cd build
cmake ..
make -j6 estimate_vicon2gt_csv
```

Run it against IMU and Vicon CSV files:

```cmd
./estimate_vicon2gt_csv \
  --imu-csv /path/to/imu.csv \
  --vicon-csv /path/to/vicon.csv \
  --output-states /path/to/gt_states.csv
```

Or build and run it with Docker CLI:

```cmd
docker build -t vicon2gt-csv .
docker run --rm \
  -v /absolute/path/to/data:/work \
  vicon2gt-csv \
  --imu-csv /work/imu.csv \
  --vicon-csv /work/vicon.csv \
  --output-states /work/gt_states.csv
```

## CSV Formats

All input CSVs must include a header row.

### IMU CSV

The IMU CSV passed to `--imu-csv` must contain:

1. A timestamp column named `timestamp_s` or `timestamp_ns`
2. Gyroscope columns for x/y/z in rad/s
3. Accelerometer columns for x/y/z in m/s^2

Accepted gyroscope header names are:

1. X: `gyro_x`, `gx`, `angular_velocity_x`, or `angular_velocity_rad_s.x`
2. Y: `gyro_y`, `gy`, `angular_velocity_y`, or `angular_velocity_rad_s.y`
3. Z: `gyro_z`, `gz`, `angular_velocity_z`, or `angular_velocity_rad_s.z`

Accepted accelerometer header names are:

1. X: `accel_x`, `ax`, `linear_acceleration_x`, or `linear_acceleration_m_s2.x`
2. Y: `accel_y`, `ay`, `linear_acceleration_y`, or `linear_acceleration_m_s2.y`
3. Z: `accel_z`, `az`, `linear_acceleration_z`, or `linear_acceleration_m_s2.z`

Recommended canonical header example:

```csv
timestamp_s,gyro_x,gyro_y,gyro_z,accel_x,accel_y,accel_z
```

### Vicon CSV

The Vicon CSV passed to `--vicon-csv` must already be in the cleaned pose format and must contain:

1. A timestamp column named `timestamp_s` or `timestamp_ns`
2. Position columns in meters: `x_m` or `x`, `y_m` or `y`, `z_m` or `z`
3. Quaternion columns: `qx`, `qy`, `qz`, `qw`

Recommended canonical header example:

```csv
timestamp_s,x_m,y_m,z_m,qx,qy,qz,qw
```

### Output Timestamps CSV

If `--timestamps-csv` is provided, that CSV must contain a timestamp column named `timestamp_s` or `timestamp_ns`. Any additional columns are ignored.

Recommended canonical header example:

```csv
timestamp_s
```

Useful CSV options:

1. `--timestamps-csv <path>` exports states at specific timestamps in the IMU clock.
2. `--trajectory-image <path>` writes a top-down trajectory plot.
3. `--output-info <path>` overrides the calibration/info output path.

## Example Outputs

![trajectory optimized](docs/traj.png)

```
state_0:
m_time:[1413394882.43529]'
q:[0.0251614, -0.79553, 0.0362641, 0.604304]'
bg:[-0.00190837, 0.0243624, 0.0807357]'
v:[-0.00342677, 0.00767059, 0.0012364]'
ba:[-0.012316, 0.0955729, 0.0272365]'
p:[-1.05904, 0.429683, 1.33247]'

state_N:
m_time:[1413394997.13767]'
q:[-0.493136, -0.636743, -0.359694, 0.471165]'
bg:[-0.00152427, 0.0255527, 0.0799344]'
v:[-0.00181216, 0.00289851, -0.00109534]'
ba:[-0.00140464, 0.0198116, 0.0590847]'
p:[-3.02558, -0.526505, 0.958149]'

R_BtoI:
 0.301581  0.017225  0.953285
0.0239218 -0.999659  0.010495
  0.95314 0.0196392 -0.301891

p_BinI:
 0.0789319
-0.0253601
 -0.118787

R_GtoV:
    0.999968            0   0.00803607
-7.00472e-05     0.999962   0.00871631
 -0.00803577  -0.00871659      0.99993

t_off_vicon_to_imu:
0.197127
```

## Frequently Asked Questions

1. _Help, doesn't converge to a good trajectory!_ -- This can be caused by many things and is hard to debug. A good first step is to save `--trajectory-image` and inspect how the optimized trajectory compares against the raw Vicon body trajectory. If the raw poses are noisy, the optimization will struggle. Another source might be poor noise values, so try adjusting both the IMU noises and Vicon pose noises. If the dataset is degenerate, you might want to try fixing the marker-to-IMU transform as if there is not enough motion this can bias the trajectory results.

2. _Do I need an initial guess of the marker body to IMU?_ -- We found that on good datasets you can just set these to be an identity transformation. If you know these values you can also try fixing and not estimating them online. Additionally if there is enough rotation in the dataset the time offset usually robustly converges.

3. _The calibration seems to not converge to the same thing on different collected datasets, why?_ -- We don't expect the calibration to converge to the same thing on each dataset as this is highly depends on the motion of the trajectory. If you worry about it you can try fixing it and not optimizing it, but in general even if the calibration is different the trajectory itself should still be of high quality.

4. _Do I need camera data?_ -- No. You only need a set of timestamps that defines where to export the optimized states. If you do not provide `--timestamps-csv`, the tool samples uniformly across the Vicon time range using `--state-freq`.

5. _Explain the timestamps the groundtruth file has in it_ -- We have two time systems in the project: vicon and inertial. The states we estimate in the optimization problem are in the IMU clock frame and the CSV file we save has timestamps in the IMU clock frame. We use an arbitrary topic timestamps to define what timestamps we will export, but all these times are still in the IMU clock frame. E.g. if we use a camera topic, the CSV will have the poses at the IMU clock time of the timestamps in this topic (i.e. if you wish to get the pose at the camera timestamp you will have an additional imu-to-camera time offset you need to worry about and is not taken into account here).

6. _What frame of reference is the groundtruth file in?_ -- The saved trajectory is in the gravity aligned frame of reference with its origin located at the vicon frame and yaw set to zero. We estimate the roll pitch rotation from the vicon frame to the gravity aligned frame, and save all groundtruth orientations rotated into this gravity frame. Thus one should use either a position + yaw or SE(3) alignment method.

## Building GTSAM and Dependencies

GTSAM is the only package you need to build the others (boost, eigen3) typically come on a linux-based OS.
You can optionally install Intel TBB installed as this will allow for GTSAM multithreaded performance ([link](https://software.intel.com/en-us/articles/installing-intel-free-libs-and-python-apt-repo)).
You don't need this, but this will allow for faster optimization.

```cmd
wget https://apt.repos.intel.com/intel-gpg-keys/GPG-PUB-KEY-INTEL-SW-PRODUCTS-2019.PUB
sudo apt-key add GPG-PUB-KEY-INTEL-SW-PRODUCTS-2019.PUB
rm GPG-PUB-KEY-INTEL-SW-PRODUCTS-2019.PUB
sudo sh -c 'echo deb https://apt.repos.intel.com/tbb all main > /etc/apt/sources.list.d/intel-tbb.list'
sudo apt-get update
sudo apt-get install intel-tbb-2020.3-912
```

Then we can [build](https://gtsam.org/get_started/) [GTSAM](https://gtsam.org/build/) as normal and install it globally on our system.
Note that we use the system Eigen since we want to link with packages which also use that Eigen version.

```cmd
sudo apt install libboost-all-dev libeigen3-dev libmetis-dev
git clone https://github.com/borglab/gtsam
cd gtsam
git checkout 4.2a5 # supported by 4.2a5 version of GTSAM
mkdir build/
cd build/
cmake -DCMAKE_BUILD_TYPE=Release -DGTSAM_USE_SYSTEM_EIGEN=ON ..
make -j6
sudo make -j6 install
echo 'export LD_LIBRARY_PATH=/usr/local/lib/:$LD_LIBRARY_PATH' >> ~/.bashrc
```

## Credit / Licensing

This code was written by the [Robot Perception and Navigation Group (RPNG)](https://sites.udel.edu/robot/) at the University of Delaware.
If you have any issues with the code please open an issue on our github page with relevant implementation details and references.
For researchers that have leveraged or compared to this work, please cite the following:

```txt
@TechReport{Geneva2020TRVICON2GT,
  Title = {vicon2gt: Derivations and Analysis},
  Author = {Patrick Geneva and Guoquan Huang},
  Number = {RPNG-2020-VICON2GT},
  Institution = {University of Delaware},
  Note = {Available: \url{http://udel.edu/~ghuang/papers/tr_vicon2gt.pdf}},
  Year = {2020},
}
```

The codebase is licensed under the [GNU General Public License v3 (GPL-3)](https://www.gnu.org/licenses/gpl-3.0.txt).
