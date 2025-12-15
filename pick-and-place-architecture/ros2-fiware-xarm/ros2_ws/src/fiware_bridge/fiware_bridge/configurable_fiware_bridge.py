#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import requests
import yaml
import os
from importlib import import_module
from ament_index_python.packages import get_package_share_directory


class ConfigurableFiwareBridge(Node):
    def __init__(self):
        super().__init__('configurable_fiware_bridge')

        self.declare_parameter('config_file', '')
        self.declare_parameter('fiware_host', '')
        self.declare_parameter('fiware_port', 0)

        config_file = self.get_parameter(
            'config_file').get_parameter_value().string_value

        if not config_file:
            try:
                pkg_share = get_package_share_directory('fiware_bridge')
                config_file = os.path.join(
                    pkg_share, 'config', 'bridge_config.yaml')
            except Exception:
                config_file = os.path.join(
                    os.path.dirname(__file__),
                    '..', 'config', 'bridge_config.yaml'
                )

        self.config = self.load_config(config_file)

        fiware_host = self.get_parameter(
            'fiware_host').get_parameter_value().string_value
        fiware_port = self.get_parameter(
            'fiware_port').get_parameter_value().integer_value

        if fiware_host:
            self.config['fiware']['host'] = fiware_host
            self.get_logger().info(f'FIWARE host overridden to: {fiware_host}')
        if fiware_port > 0:
            self.config['fiware']['port'] = fiware_port
            self.get_logger().info(f'FIWARE port overridden to: {fiware_port}')

        self.orion_url = f"http://{self.config['fiware']['host']}:{self.config['fiware']['port']}/v2"
        self.fiware_headers = {
            # 'Content-Type': 'application/json',
            'Fiware-Service': self.config['fiware']['service'],
            'Fiware-ServicePath': self.config['fiware']['service_path']
        }

        self.ros_publishers = {}
        self.ros_subscribers = {}
        self.message_classes = {}

        self.setup_ros_to_fiware_bridges()
        self.setup_fiware_to_ros_bridges()

        polling_interval = self.config.get('polling_interval', 1.0)
        self.timer = self.create_timer(
            polling_interval, self.poll_fiware_entities)

        self.get_logger().info('Configurable FIWARE Bridge started!')
        self.get_logger().info(f'Loaded config from: {config_file}')
        self.log_configuration()

    def load_config(self, config_file):
        try:
            with open(config_file, 'r') as f:
                config = yaml.safe_load(f)
            self.get_logger().info(f'Configuration loaded from {config_file}')
            return config
        except Exception as e:
            self.get_logger().error(f'Failed to load config: {str(e)}')
            raise

    def log_configuration(self):
        self.get_logger().info('=== Bridge Configuration ===')
        self.get_logger().info(
            f'FIWARE: {self.config["fiware"]["host"]}:{self.config["fiware"]["port"]}')
        self.get_logger().info(f'Service: {self.config["fiware"]["service"]}')

        self.get_logger().info('FIWARE → ROS2 mappings:')
        for mapping in self.config.get('fiware_to_ros', []):
            self.get_logger().info(
                f'  {mapping["fiware_entity"]}.{mapping["fiware_attribute"]} → {mapping["ros_topic"]}')

        self.get_logger().info('ROS2 → FIWARE mappings:')
        for mapping in self.config.get('ros_to_fiware', []):
            entity = mapping["fiware_entity"]
            if 'fiware_attribute' in mapping:
                self.get_logger().info(
                    f'  {mapping["ros_topic"]} → {entity}.{mapping["fiware_attribute"]}')
            else:
                attrs = [a['name']
                         for a in mapping.get('fiware_attributes', [])]
                self.get_logger().info(
                    f'  {mapping["ros_topic"]} → {entity}.[{", ".join(attrs)}]')

    def get_message_class(self, msg_type_str):
        if msg_type_str in self.message_classes:
            return self.message_classes[msg_type_str]

        try:
            package, message = msg_type_str.split('/')

            module = import_module(f'{package}.msg')
            msg_class = getattr(module, message)

            self.message_classes[msg_type_str] = msg_class
            return msg_class
        except Exception as e:
            self.get_logger().error(
                f'Failed to load message type {msg_type_str}: {str(e)}')
            return None

    def setup_fiware_to_ros_bridges(self):
        for mapping in self.config.get('fiware_to_ros', []):
            topic = mapping['ros_topic']
            msg_type = mapping['ros_msg_type']

            msg_class = self.get_message_class(msg_type)
            if msg_class:
                publisher = self.create_publisher(msg_class, topic, 10)
                self.ros_publishers[topic] = {
                    'publisher': publisher,
                    'msg_class': msg_class,
                    'mapping': mapping
                }
                self.get_logger().info(
                    f'Created publisher: {topic} ({msg_type})')

    def setup_ros_to_fiware_bridges(self):
        for mapping in self.config.get('ros_to_fiware', []):
            topic = mapping['ros_topic']
            msg_type = mapping['ros_msg_type']

            msg_class = self.get_message_class(msg_type)
            if msg_class:
                callback = self.create_ros_callback(mapping)
                subscriber = self.create_subscription(
                    msg_class, topic, callback, 10)

                self.ros_subscribers[topic] = {
                    'subscriber': subscriber,
                    'mapping': mapping
                }
                self.get_logger().info(
                    f'Created subscriber: {topic} ({msg_type})')

                self.initialize_fiware_entity(mapping)

    def create_ros_callback(self, mapping):
        def callback(msg):
            self.handle_ros_message(msg, mapping)
        return callback

    def initialize_fiware_entity(self, mapping):
        entity_id = mapping['fiware_entity']
        entity_type = mapping['fiware_entity_type']
        url = f"{self.orion_url}/entities/{entity_id}"

        try:
            response = requests.get(url, headers=self.fiware_headers)
            if response.status_code == 404:
                entity = {
                    "id": entity_id,
                    "type": entity_type
                }

                if 'fiware_attribute' in mapping:
                    entity[mapping['fiware_attribute']] = {
                        "type": "Text",
                        "value": ""
                    }
                elif 'fiware_attributes' in mapping:
                    for attr in mapping['fiware_attributes']:
                        entity[attr['name']] = {
                            "type": "Number",
                            "value": 0.0
                        }

                create_url = f"{self.orion_url}/entities"
                response = requests.post(
                    create_url, json=entity, headers=self.fiware_headers)

                if response.status_code == 201:
                    self.get_logger().info(
                        f'Created FIWARE entity: {entity_id}')
                else:
                    self.get_logger().error(
                        f'Failed to create entity {entity_id}: {response.text}')
            else:
                self.get_logger().info(f'FIWARE entity exists: {entity_id}')
        except Exception as e:
            self.get_logger().error(
                f'Error initializing entity {entity_id}: {str(e)}')

    def poll_fiware_entities(self):
        for topic, pub_info in self.ros_publishers.items():
            mapping = pub_info['mapping']
            entity_id = mapping['fiware_entity']
            attribute = mapping['fiware_attribute']

            try:
                url = f"{self.orion_url}/entities/{entity_id}"
                response = requests.get(url, headers=self.fiware_headers)

                if response.status_code == 200:
                    data = response.json()
                    value = data.get(attribute, {}).get('value')

                    if value is not None:
                        if 'value_mapping' in mapping:
                            value = mapping['value_mapping'].get(
                                str(value), value)

                        msg = self.create_ros_message(
                            pub_info['msg_class'], value)
                        if msg:
                            pub_info['publisher'].publish(msg)
                            self.get_logger().debug(
                                f'{entity_id}.{attribute} → {topic}: {value}')

            except Exception as e:
                self.get_logger().error(f'Error polling {entity_id}: {str(e)}')

    def create_ros_message(self, msg_class, value):
        try:
            msg = msg_class()

            if hasattr(msg, 'data'):
                msg.data = value
            else:
                self.get_logger().warn(
                    f'Complex message type not fully supported: {msg_class}')

            return msg
        except Exception as e:
            self.get_logger().error(f'Error creating message: {str(e)}')
            return None

    def handle_ros_message(self, msg, mapping):
        entity_id = mapping['fiware_entity']
        url = f"{self.orion_url}/entities/{entity_id}/attrs"

        payload = {}

        try:
            if 'fiware_attribute' in mapping:
                field_name = mapping.get('ros_field', 'data')
                value = self.get_nested_attribute(msg, field_name)

                payload[mapping['fiware_attribute']] = {
                    "type": "Text" if isinstance(value, str) else "Number",
                    "value": value
                }

            elif 'fiware_attributes' in mapping:
                for attr_mapping in mapping['fiware_attributes']:
                    field_name = attr_mapping['ros_field']
                    value = self.get_nested_attribute(msg, field_name)

                    payload[attr_mapping['name']] = {
                        "type": "Number",
                        "value": float(value) if value is not None else 0.0
                    }

            response = requests.patch(
                url, json=payload, headers=self.fiware_headers)

            if response.status_code == 204:
                self.get_logger().debug(
                    f'{mapping["ros_topic"]} → {entity_id}: {payload}')
            else:
                self.get_logger().error(
                    f'Failed to update {entity_id}: {response.text}')

        except Exception as e:
            self.get_logger().error(f'Error handling ROS message: {str(e)}')

    def get_nested_attribute(self, obj, attr_path):
        try:
            parts = attr_path.split('.')
            value = obj
            for part in parts:
                value = getattr(value, part)
            return value
        except Exception as e:
            self.get_logger().error(
                f'Error getting attribute {attr_path}: {str(e)}')
            return None


def main(args=None):
    rclpy.init(args=args)

    node = ConfigurableFiwareBridge()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
