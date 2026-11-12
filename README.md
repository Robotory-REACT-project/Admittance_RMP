# Admittance RMP ROS 2 packages

This repository contains two ROS 2 packages:

| Directory | Package | Purpose |
| --- | --- | --- |
| `REACT_intrinsic/` | `REACT_intrinsic` | Intrinsic force sensing and contact data on arbitrary surfaces. |
| `ur10_intent_admittance_rmp_v2/` | `ur10_intent_admittance_rmp_v2` | Experimental UR10 intent admittance RMP and contact adapter. |

Clone this repository under a ROS 2 workspace's `src/` directory. Both package directories can then be discovered by `colcon`:

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon list --base-paths src/Admittance_RMP
```

The admittance package also depends on packages outside this repository, including `rb10_rmpflow_rviz` and `Y2FT_AQ`. See [its README](ur10_intent_admittance_rmp_v2/README.md) for build and launch instructions. Runtime logs and generated build files are excluded from version control.
