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

  - fiware_entity: "VoiceCommand:operator-1"
    fiware_attribute: "command"
    ros_topic: "/user_inputs/voice_command"
    ros_msg_type: "std_msgs/String"
  
  - fiware_entity: "GestureDetector:operator-1"
    fiware_attribute: "command"
    ros_topic: "/user_inputs/gesture_command"
    ros_msg_type: "std_msgs/String"

  # NODE-BACKEND ONLY: base64 payload. The Orion-LD DDS bridge has no decode step,
  # so on bridge_backend:=dds this would deliver an undecoded blob. generate_config.py
  # excludes it from the DDS mapping; it works only on bridge_backend:=node.
  - fiware_entity: "BottleDetectionJob:processor-01"
    fiware_attribute: "json"
    ros_topic: "/bottle_detection/job_json"
    ros_msg_type: "std_msgs/String"
    decode_base64: true

ros_to_fiware:
  # Leaf is `status_json`, not `status`: Orion-LD's DDS module reserves the `status`
  # leaf (collides with ROS 2 action GoalStatusArray), so a DDS topic ending in
  # `/status` never bridges. The producer (task_pack_bottle.py) publishes on
  # `/task_pack_bottle/status_json`; the FIWARE attribute stays `status`, so NGSI
  # consumers are unaffected. Works on both backends.
  - ros_topic: "/task_pack_bottle/status_json"
    ros_msg_type: "std_msgs/String"
    fiware_entity: "task_pack_bottle"
    fiware_entity_type: "Status"
    fiware_attribute: "status"
    ros_field: "data"

  - ros_topic: "/xarm_pack_bottle/robot_status"
    ros_msg_type: "std_msgs/String"
    fiware_entity: "xarm_robot"
    fiware_entity_type: "Status"
    fiware_attribute: "status"
    ros_field: "data"

  - ros_topic: "/bottle_detection/command"
    ros_msg_type: "std_msgs/String"
    fiware_entity: "BottleDetectionJob:processor-01"
    fiware_entity_type: "BottleDetectionJob"
    fiware_attribute: "command"
    ros_field: "data"

  # DDS-native AI detector first step (AI_BACKEND=ros2): the detector publishes a
  # simple status string on `/bottle_detection/status_json` (leaf is `status_json`,
  # not the reserved `status` leaf). Maps to BottleDetectionJob:processor-01.status.
  # Full result JSON / image URLs / base64 / pick pose are NOT migrated yet and stay
  # on the NGSI-v2 backend.
  - ros_topic: "/bottle_detection/status_json"
    ros_msg_type: "std_msgs/String"
    fiware_entity: "BottleDetectionJob:processor-01"
    fiware_entity_type: "BottleDetectionJob"
    fiware_attribute: "status"
    ros_field: "data"

  - ros_topic: "/task_pack_bottle/stage"
    ros_msg_type: "std_msgs/String"
    fiware_entity: "TaskPackBottle:operator-01"
    fiware_entity_type: "TaskPackBottle"
    fiware_attribute: "stage"
    ros_field: "data"