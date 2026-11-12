# UR10 seven-RMP intent/admittance experiment

This package adds one experimental contact-admittance RMP to the existing
guarded six-policy UR10 controller **without modifying the production
`rb10_rmpflow_rviz` source or launcher**.

The seventh leaf uses the production controller's existing external-RMP input:

```text
contact point + base-frame force + compliance probability + measured q/qd
                              |
                              v
              contact admittance natural pair
                              |
                              v
 /escape_on/external_rmp/admittance/{metric_sqrt,acceleration}
                              |
                              v
 existing metric-weighted root resolve (six policies + admittance)
                              |
                              v
 existing tracked reference and command/safety guards
```

The external node never publishes `/UR10_right/targetJ`. The existing
`rmpflow_controller` remains the only robot-command publisher.

The Stage-1 hardware profile uses direction-preserving task/joint limits and a
damped spectral inverse. It disables the continuously centering joint-limit
RMP to prevent no-contact drift, while retaining the controller's hard
predictive joint-position guard. Near-singular contact directions with an
achievable/requested acceleration cosine below 0.50 are suppressed.

## What is and is not implemented

Implemented:

- fixed link-local contact point input;
- base-frame force-vector input;
- scalar compliance-intent probability input;
- force/probability hysteresis and a 0.30 s smooth activation ramp;
- translational contact admittance natural pair;
- zero contribution on stale joint state, wrench, or intent input;
- shutdown of the complete experimental launch if the seventh-leaf process exits;
- production joint-limit, velocity-cap, damping, target, orientation, collision,
  tracked-reference and command-guard behavior remains in the command path.
- sensor-1/sensor-2-only contact adapter for the current hardware limitation;
- automatic `supporter -> forearm_link` contact-point conversion;
- automatic `supporter -> base_link` force-vector conversion;
- strongest-valid-contact selection when sensors 1 and 2 report simultaneously;
- manual compliance probability for the pre-vision prototype.
- an isolated Stage-2 controller fork that computes the contact-gated target
  from the exact same `q_ctrl, qd_ctrl` snapshot as the root RMP solve;
- effective activation: target weakening occurs only when a finite,
  direction-valid admittance metric/acceleration pair is actually published.

Not implemented:

- force estimation from the proximity sensors;
- worker-intent inference from camera data;
- rotational compliance or multiple simultaneous contacts;
- certified force, power, or human-contact safety limiting.

The package therefore expects upstream estimators or temporary manual test
publishers. A supplied `WrenchStamped` is treated as an already estimated
force; it must not be described as a direct force measurement when it is only
a proximity-derived proxy.

The experimental controller explicitly uses the production B1 feedback state:
observer enabled, position blend `1.00`, velocity blend `0.25`.

## Build and dry-run

```bash
cd /home/ur/ros2_ws
source /opt/ros/humble/setup.bash
source /home/ur/ros2_ws/install/setup.bash
colcon build --packages-select Y2Matrix Y2Filters Y2FT_AQ REACT_intrinsic \
  ur10_intent_admittance_rmp_v2 --symlink-install
source /home/ur/ros2_ws/install/setup.bash

ros2 launch ur10_intent_admittance_rmp_v2 \
  ur10_7rmp_intent_admittance_experimental.launch.py \
  hardware_command_enabled:=false
```

If `FTGetMain` is already running, leave the default
`start_ft_acquisition:=false`. To start the two-sensor FT acquisition in the
same command, add `start_ft_acquisition:=true`. Never start two FT acquisition
nodes against the same gateway.

Only hardware contact sensors 1 and 2 are accepted. Sensors 3 and 4 are not
subscribed by the adapter.

The copied experimental graph configuration is
`config/params_7rmp_experimental.yaml`. It adds the enabled identity-map
`external` leaf named `admittance_external`; the production launch still
disables its c-space, tangent-escape, self-collision RMP and legacy three-axis
orientation leaves, leaving the same six production policy types plus this
seventh policy.

## Contact pipeline

The integrated path is:

```text
/ur10skku/ftdata_tcp_{1,2}
  -> REACT_intrinsic contact point/corrected wrench
  -> sensor-1/2 contact adapter
  -> /admittance_rmp/contact_point       (forearm_link)
  -> /admittance_rmp/wrench              (base_link)
  -> seventh external admittance RMP
```

Until the vision intent estimator is connected, the adapter publishes a manual
probability of `1.0`. Later launch with `publish_manual_intent:=false` to hand
the topic over to the vision node.

The initial wrench-origin translations are provisional zeros. Validate contact
location and force direction in dry-run. If the corrected wrench points opposite
to the desired compliant motion, test `contact_force_sign:=-1.0` in dry-run
before any hardware command is enabled.

Observe adapter selection and the downstream gate:

```bash
ros2 topic echo /admittance_rmp/contact_adapter_status
ros2 topic echo /admittance_rmp/integration_mode
```

## Manual prototype inputs

Publish a link-local point once:

```bash
ros2 topic pub --once /admittance_rmp/contact_point \
  geometry_msgs/msg/PointStamped \
  '{header: {frame_id: forearm_link}, point: {x: -0.24, y: 0.035, z: 0.06}}'
```

Stream a compliance probability faster than the configured 0.5 s timeout:

```bash
ros2 topic pub -r 20 /worker_intent/compliance_probability \
  std_msgs/msg/Float64 '{data: 0.8}'
```

Stream a base-frame force faster than the configured 0.25 s timeout:

```bash
ros2 topic pub -r 50 /admittance_rmp/wrench \
  geometry_msgs/msg/WrenchStamped \
  '{header: {frame_id: base_link}, wrench: {force: {x: 0.0, y: 10.0, z: 0.0}}}'
```

Observe the gate state:

```bash
ros2 topic echo /admittance_rmp/integration_mode
```

## Stage-2 contact-gated target

Stage 2 keeps avoidance disabled and adds Cartesian target tracking. A static
increase of the contact metric did not reproduce Stage-1 compliance in the
30 N offline dynamics sweep, even at an effective contact metric of 300000.
Stage 2 therefore keeps the validated contact metric at 300 and replaces only
its built-in Cartesian target leaf with an **internal** gated target in an
isolated controller fork. The production controller source remains unchanged.
The internal leaf uses the same `q_ctrl, qd_ctrl`, Pinocchio context, goal
snapshot, and controller cycle as the root solve; only the scalar effective
contact activation crosses a ROS topic boundary:

```text
target metric scale = 0.05 + (1 - effective activation) * 0.95
```

The activation already follows the 0.30 s smoothstep. Thus target authority is
100% without contact and no lower than 5% at full, valid contact. If the
admittance leaf is stale, invalid, singular-direction-blocked, or fails its
calculation, effective activation is zero and target authority immediately
remains 100%. Releasing contact restores the target metric and returns the
robot to the current goal. This arbitration is Stage-2-only; Stage 1 and the
production six-RMP profile are unchanged.

The former `gated_target_external_rmp` path is retained only as legacy source
and is not launched by the hardware-stage wrapper. It must not be used for
hardware testing because it evaluated a separately sampled measured state.

Stage-2 hardware output requires the existing explicit experimental-risk
acknowledgement. The operator remains responsible for supervised validation;
this section documents an experimental candidate, not a released safety-rated
hardware profile.

Useful diagnostics are:

```bash
ros2 topic echo /admittance_rmp/activation
ros2 topic echo /admittance_rmp/internal_target_metric_scale
ros2 topic echo /admittance_rmp/integration_mode
```

Reset the latched contact point and leaf:

```bash
ros2 service call /admittance_rmp/reset std_srvs/srv/Trigger '{}'
```

## Hardware interlock

Hardware output is disabled by default. Enabling it requires two explicit
arguments:

```text
hardware_command_enabled:=true
acknowledge_experimental_hardware_risk:=true
```

Do not use these arguments until dry-run logs show bounded commands and the
contact point, force frame/direction, intent timeout, command guard and physical
clearance have been checked. This package is an experimental prototype, not a
robot safety function.
