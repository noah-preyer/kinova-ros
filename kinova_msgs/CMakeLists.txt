cmake_minimum_required(VERSION 3.8)
project(kinova_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CartesianForce.msg"
  "msg/FingerPosition.msg"
  "msg/JointAngles.msg"
  "msg/JointTorque.msg"
  "msg/JointVelocity.msg"
  "msg/KinovaPose.msg"
  "msg/PoseVelocity.msg"
  "msg/PoseVelocityWithFingerVelocity.msg"
  "msg/PoseVelocityWithFingers.msg"
  "srv/AddPoseToCartesianTrajectory.srv"
  "srv/ClearTrajectories.srv"
  "srv/HomeArm.srv"
  "srv/RunCOMParametersEstimation.srv"
  "srv/SetEndEffectorOffset.srv"
  "srv/SetForceControlParams.srv"
  "srv/SetNullSpaceModeState.srv"
  "srv/SetTorqueControlMode.srv"
  "srv/SetTorqueControlParameters.srv"
  "srv/Start.srv"
  "srv/Stop.srv"
  "srv/ZeroTorques.srv"
  "action/ArmJointAngles.action"
  "action/ArmPose.action"
  "action/ArmKinovaPose.action"
  "action/SetFingersPosition.action"
  DEPENDENCIES geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
