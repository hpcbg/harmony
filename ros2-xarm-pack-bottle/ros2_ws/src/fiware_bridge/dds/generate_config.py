#!/usr/bin/env python3
"""Generate an Orion-LD DDS-bridge config from the fiware_bridge YAML mapping.

The custom Python node (`configurable_fiware_bridge.py`) and the optional
Orion-LD built-in DDS bridge must never drift, so both are driven from the same
source of truth: `fiware_bridge/config/bridge_config.yaml`.

This script reads that YAML and emits `context_broker_config.json` (the file
Orion-LD reads when started with `-wip dds -mongocOnly`). Scalar `std_msgs`
topics (String/Bool/Int32/Int64/Float32/Float64) are emitted — the generic DDS
bridge maps each via DDS dynamic-type discovery as `.<attribute>.value.data`
(String->JSON string, Bool->JSON bool, Int->JSON number; validated end-to-end).
Non-`std_msgs` types (`custom_interfaces/*`) are not representable and stay on the
node backend. Node-only transforms (`value_mapping`/`decode_base64`) are not
reproduced on the DDS path — affected topics are emitted but flagged. See
`docs/dds_full_integration_plan.md` and `docs/bridge_inventory.md`.

Topic-name rule: ROS 2 `/<a>/<b>`  ->  DDS `rt/<a>/<b>`.
The `domain` MUST equal the project's ROS_DOMAIN_ID (default 0).

Usage:
    python3 generate_config.py                 # uses ../config/bridge_config.yaml
    python3 generate_config.py --config X --output Y --domain 0
"""

import argparse
import json
import os
import sys

import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_CONFIG = os.path.join(HERE, '..', 'config', 'bridge_config.yaml')
DEFAULT_OUTPUT = os.path.join(HERE, 'context_broker_config.json')

# Scalar std_msgs types the generic Orion-LD DDS bridge maps as `.value.data`.
# Validated on fiware/orion-ld:1.13.0-PRE-1835 (dds-full-integration branch):
# String -> JSON string, Bool -> JSON bool, Int32 -> JSON number, both directions.
# The mapping is structural (DDS dynamic-type discovery), so any single-member
# `std_msgs` scalar works the same way; extend this set as types are validated.
ELIGIBLE_STD_MSGS = {
    'std_msgs/String',
    'std_msgs/Bool',
    'std_msgs/Int32',
    'std_msgs/Int64',
    'std_msgs/Float32',
    'std_msgs/Float64',
}

# Node-only transforms the DDS bridge cannot reproduce; topics using these are
# still emitted but flagged so the DDS path is wired with NATIVE values upstream.
NODE_ONLY_TRANSFORM_KEYS = ('value_mapping', 'decode_base64')


def ngsild_ids(entity_id, entity_type):
    """Return (entityType, entityId-URN) for an NGSI-v2 short id.

    - If an explicit type is given (ros_to_fiware has `fiware_entity_type`),
      use it and prefix the whole id:  Status + SystemSkillPickAndPlace
      -> urn:ngsi-ld:Status:SystemSkillPickAndPlace
    - Otherwise derive the type from the id's own prefix (the part before the
      first ':'):  VoiceCommand:operator-1 -> type VoiceCommand,
      urn:ngsi-ld:VoiceCommand:operator-1
    """
    if entity_type:
        return entity_type, f"urn:ngsi-ld:{entity_type}:{entity_id}"
    etype = entity_id.split(':', 1)[0]
    return etype, f"urn:ngsi-ld:{entity_id}"


def dds_topic(ros_topic):
    return 'rt/' + ros_topic.lstrip('/')


def build_topics(cfg):
    topics = {}
    skipped = []
    transform_warnings = []

    def add(mapping, direction):
        ros_topic = mapping['ros_topic']
        msg_type = mapping.get('ros_msg_type', '')
        if msg_type not in ELIGIBLE_STD_MSGS:
            skipped.append((ros_topic, msg_type, direction))
            return
        entity_type, urn = ngsild_ids(
            mapping['fiware_entity'], mapping.get('fiware_entity_type'))
        topics[dds_topic(ros_topic)] = {
            'entityType': entity_type,
            'entityId': urn,
            'attribute': mapping['fiware_attribute'],
        }
        used = [k for k in NODE_ONLY_TRANSFORM_KEYS if k in mapping]
        if used:
            transform_warnings.append((ros_topic, msg_type, used))

    for m in cfg.get('ros_to_fiware', []):
        add(m, 'pub->FIWARE')
    for m in cfg.get('fiware_to_ros', []):
        add(m, 'FIWARE->sub')

    return topics, skipped, transform_warnings


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', default=DEFAULT_CONFIG,
                        help='Path to bridge_config.yaml (source of truth)')
    parser.add_argument('--output', default=DEFAULT_OUTPUT,
                        help='Path to write context_broker_config.json')
    parser.add_argument('--domain', type=int,
                        default=int(os.environ.get('ROS_DOMAIN_ID', '0')),
                        help='DDS domain id (must equal ROS_DOMAIN_ID; default 0)')
    parser.add_argument('--transport', default='udp')
    args = parser.parse_args()

    with open(args.config) as f:
        cfg = yaml.safe_load(f)

    topics, skipped, transform_warnings = build_topics(cfg)

    if not topics:
        print('WARNING: no eligible std_msgs topics found — nothing to bridge '
              'on the DDS path.', file=sys.stderr)

    out = {
        'dds': {
            'ddsmodule': {
                'dds': {
                    'domain': args.domain,
                    'transport': args.transport,
                }
            },
            'ngsild': {
                'topics': topics,
            }
        }
    }

    with open(args.output, 'w') as f:
        json.dump(out, f, indent=2)
        f.write('\n')

    print(f"Wrote {args.output}")
    print(f"  domain: {args.domain}  transport: {args.transport}")
    print(f"  DDS-eligible (scalar std_msgs) topics: {len(topics)}")
    for dds_t, spec in topics.items():
        print(f"    {dds_t} -> {spec['entityId']}.{spec['attribute']}")
    if transform_warnings:
        print(f"  WARNING — node-only transforms NOT reproduced on the DDS path "
              f"({len(transform_warnings)}); wire native values upstream:")
        for ros_t, mt, used in transform_warnings:
            print(f"    {ros_t} ({mt}): {', '.join(used)}")
    if skipped:
        print(f"  Skipped (not a scalar std_msgs type, stay on node backend): "
              f"{len(skipped)}")
        for ros_t, mt, direction in skipped:
            print(f"    {ros_t} ({mt or 'unknown'}, {direction})")


if __name__ == '__main__':
    main()
