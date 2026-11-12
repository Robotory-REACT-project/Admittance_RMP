#!/usr/bin/env bash
# Poster experiment recorder: one rosbag with everything needed to plot
# "computed variables -> UR10 behaviour" (stage 9 + vision bridge).
# Usage: record_poster_experiment.sh [name]      (Ctrl-C to stop)
set -e
source /opt/ros/humble/setup.bash >/dev/null 2>&1
source ~/ros2_ws/install/setup.bash >/dev/null 2>&1
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-7}
NAME=${1:-poster}
OUT=~/poster_bags/$(date +%Y%m%d_%H%M%S)_${NAME}
mkdir -p ~/poster_bags
TOPICS=(
  # robot, command, goal (the goal frame is also broadcast on /tf)
  /UR10_right/joint_states /UR10_right/targetJ /goal_pose /goal_position /tf /tf_static
  # proximity sensors
  /fused_distance1 /fused_distance2 /fused_distance3 /fused_distance4
  # contact (F/T -> adapter -> controller)
  /admittance_rmp/wrench /admittance_rmp/contact_point /admittance_rmp/contact_adapter_status
  /sensor1/contactsensing/cp /sensor1/contactsensing/wrench_corrected /sensor1/contactsensing/contact_state
  /sensor2/contactsensing/cp /sensor2/contactsensing/wrench_corrected /sensor2/contactsensing/contact_state
  # controller internals: weights, activation, target scale, diagnostics
  /admittance_rmp/activation /admittance_rmp/avoidance_diagnostics /admittance_rmp/avoidance_weight
  /admittance_rmp/intent_effective /admittance_rmp/internal_target_metric_scale
  /admittance_rmp/internal_diagnostics /rmp_leaf_ablation /rmp_control_timing /rmp_latency_compensation
  # Optional lateral-height trial diagnostics (subscribed when available).
  /lateral_height_trial/diagnostics
  # intent: vision judgment -> bridge lambda
  /hand_detector/approach_judgment /hand_detector/approach_signed /hand_detector/head_pose
  /worker_intent/compliance_probability /worker_intent/lambda_debug
  # scene marks
  /experiment/scene
)
echo "기록 위치: $OUT"
echo "멈추려면 Ctrl-C"
exec ros2 bag record -o "$OUT" "${TOPICS[@]}"
