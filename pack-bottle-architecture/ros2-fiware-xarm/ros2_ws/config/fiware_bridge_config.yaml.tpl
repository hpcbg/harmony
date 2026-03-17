fiware:
  host: "localhost"
  port: 1026
  service: "openiot"
  service_path: "/"

polling_interval: 1.0

fiware_to_ros:
  - fiware_entity: "M5Stick:001"
    fiware_attribute: "buttonBlue"
    ros_topic: "/user_inputs/start_button"
    ros_msg_type: "std_msgs/Bool"

  - fiware_entity: "M5Stick:001"
    fiware_attribute: "buttonRed"
    ros_topic: "/user_inputs/stop_button"
    ros_msg_type: "std_msgs/Bool"

  - fiware_entity: "M5Stick:001"
    fiware_attribute: "angle"
    ros_topic: "/angle"
    ros_msg_type: "std_msgs/Int32"

  - fiware_entity: "VoiceCommand:operator-1"
    fiware_attribute: "command"
    ros_topic: "/user_inputs/voice_command"
    ros_msg_type: "std_msgs/String"
  
  - fiware_entity: "GestureDetector:operator-1"
    fiware_attribute: "command"
    ros_topic: "/user_inputs/gesture_command"
    ros_msg_type: "std_msgs/String"

ros_to_fiware:
  - ros_topic: "/task_pack_bottle/status"
    ros_msg_type: "std_msgs/String"
    fiware_entity: "task_pack_bottle"
    fiware_entity_type: "Status"
    fiware_attribute: "status"
    ros_field: "data"

  - ros_topic: "/bottle_detector_brigde/processed_image"
    ros_msg_type: "std_msgs/String"
    fiware_entity: "last_processed_image"
    fiware_entity_type: "String"
    fiware_attribute: "url"
    ros_field: "data"

  - ros_topic: "/bottle_detector_brigde/found_bottle"
    ros_msg_type: "std_msgs/String"
    fiware_entity: "last_found_bottle"
    fiware_entity_type: "String"
    fiware_attribute: "info"
    ros_field: "data"

  - ros_topic: "/xarm_pack_bottle/robot_status"
    ros_msg_type: "std_msgs/String"
    fiware_entity: "xarm_robot"
    fiware_entity_type: "Status"
    fiware_attribute: "status"
    ros_field: "data"

  - ros_topic: "/filling_station/progress"
    ros_msg_type: "std_msgs/Int32"
    fiware_entity: "filling_station"
    fiware_entity_type: "Int32"
    fiware_attribute: "progress"
    ros_field: "data"