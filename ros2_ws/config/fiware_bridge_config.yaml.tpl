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

ros_to_fiware:
  - ros_topic: "/system_skill_pick_and_place/status"
    ros_msg_type: "std_msgs/String"
    fiware_entity: "system_skill_pick_and_place"
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

  - ros_topic: "/xarm_pick_and_place/robot_status"
    ros_msg_type: "std_msgs/String"
    fiware_entity: "xarm_robot"
    fiware_entity_type: "Status"
    fiware_attribute: "status"
    ros_field: "data"