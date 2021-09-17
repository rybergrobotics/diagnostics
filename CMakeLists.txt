cmake_minimum_required(VERSION 2.8.3)
  project(diagnostics)
  
  find_package(catkin REQUIRED COMPONENTS
    rospy
    std_msgs
    message_generation
  )

  add_message_files(
    FILES
    LampStates.msg
    LampDriverStates.msg
    LampsDiagnostics.msg
    VoltageMeasurementsDiagnostics.msg
    IrSensorsDiagnostics.msg
  )

  add_service_files(
    FILES
    TurnLedOn.srv
    TurnLedRed.srv
  )

catkin_python_setup()

generate_messages(
  DEPENDENCIES
  std_msgs
)
  catkin_package(CATKIN_DEPENDS std_msgs message_runtime
)