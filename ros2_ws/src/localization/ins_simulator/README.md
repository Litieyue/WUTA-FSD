# INS Simulator

`ins_simulator` is the CG-410 INS input adapter for the WUTA pure Python
simulator. It converts the vehicle ground-truth odometry published by the
vehicle model into the `/cg410/odometry` topic consumed by the existing FSD
localization pipeline.

## Data Flow

```text
/sim/ground_truth (nav_msgs/Odometry, 50 Hz)
        |
        v
ins_simulator
  - downsample to 20 Hz
  - copy map-frame pose and base_link twist
  - add deterministic Gaussian noise
  - fill covariance
        |
        v
/cg410/odometry (nav_msgs/Odometry, 20 Hz)
        |
        v
FSD ekf_node
```

## Topics

| Direction | Topic | Type | Rate | Notes |
|---|---|---|---|---|
| Subscribe | `/sim/ground_truth` | `nav_msgs/msg/Odometry` | 50 Hz | Published by `vehicle_model` |
| Publish | `/cg410/odometry` | `nav_msgs/msg/Odometry` | 20 Hz | Consumed by `ekf_node` |

The default frame convention follows the current simulator:

```text
header.frame_id: map
child_frame_id: base_link
```

## Noise Model

Default parameters:

```yaml
publish_rate: 20.0
position_noise_std: 0.05       # meters, applied to x and y
z_noise_std: 0.0               # meters
yaw_noise_std_deg: 0.5         # degrees
velocity_noise_std: 0.02       # m/s, applied to linear.x
angular_velocity_noise_std: 0.0
seed: 42
```

The fixed random seed makes repeated simulator runs reproducible when all
other simulator seeds and command inputs are also fixed.

## Run

Standalone:

```bash
ros2 launch ins_simulator ins_simulator.launch.py
```

From the integrated bringup:

```bash
ros2 launch simulator_bringup simulator.launch.py launch_ins:=true
```

Useful overrides:

```bash
ros2 launch simulator_bringup simulator.launch.py \
  ins_position_noise_std:=0.03 \
  ins_yaw_noise_std_deg:=0.3 \
  ins_seed:=7
```

## Acceptance Checks

```bash
ros2 topic hz /cg410/odometry
ros2 topic echo /cg410/odometry --once
```

Expected behavior:

- `/cg410/odometry` publishes at about 20 Hz.
- `header.frame_id` is `map`.
- `child_frame_id` is `base_link`.
- Position follows `/sim/ground_truth` with small noise.
- Yaw follows `/sim/ground_truth` with small noise.
- Pose and twist covariance diagonals are non-zero where the EKF needs
  measurement uncertainty.
