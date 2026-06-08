#!/usr/bin/env python3
"""
Update GPS spoof SDF parameters from YAML config before launching simulation.
"""
import argparse
import os
import sys
import yaml
from xml.etree import ElementTree as ET


def load_config(yaml_path, scenario=None):
    """Load attack parameters from YAML, optionally using a named scenario."""
    with open(yaml_path, 'r') as f:
        config = yaml.safe_load(f)

    params = config['attack'].copy()

    # Override with scenario if specified
    if scenario and scenario in config.get('scenarios', {}):
        scenario_params = config['scenarios'][scenario]
        params.update(scenario_params)
        print(f"[Config] Using scenario: {scenario}")

    return params


def update_sdf(sdf_path, params):
    """Update GPS spoof SDF file with parameters from config."""
    tree = ET.parse(sdf_path)
    root = tree.getroot()

    # Find the gps_plugin element
    plugin = root.find(".//plugin[@name='gps_plugin']")
    if plugin is None:
        raise RuntimeError(f"Could not find gps_plugin in {sdf_path}")

    # Mapping from config keys to SDF element names
    param_map = {
        'offset_x': 'gpsSpoofOffsetX',
        'offset_y': 'gpsSpoofOffsetY',
        'offset_z': 'gpsSpoofOffsetZ',
        'drift_x': 'gpsSpoofDriftX',
        'drift_y': 'gpsSpoofDriftY',
        'drift_z': 'gpsSpoofDriftZ',
        'start_time': 'gpsSpoofStart',
        'end_time': 'gpsSpoofEnd',
        'smooth_duration': 'gpsSmoothTransitionDuration',
        'smooth_exit_duration': 'gpsSmoothExitDuration',
        'hold_after_end': 'gpsSpoofHoldAfterEnd',
        'takeoff_z_threshold': 'gpsSpoofTakeoffZ',
    }

    updated = []
    for key, elem_name in param_map.items():
        if key not in params:
            continue

        elem = plugin.find(elem_name)
        if elem is None:
            # Create element if not exists
            elem = ET.SubElement(plugin, elem_name)

        old_val = elem.text
        if isinstance(params[key], bool):
            new_val = str(params[key]).lower()
        else:
            new_val = str(params[key])
        elem.text = new_val

        if old_val != new_val:
            updated.append(f"  {elem_name}: {old_val} -> {new_val}")

    # Ensure spoofing is enabled
    enable_elem = plugin.find('gpsSpoofEnable')
    if enable_elem is None:
        enable_elem = ET.SubElement(plugin, 'gpsSpoofEnable')
    enable_elem.text = 'true'

    # Write back
    children = list(plugin)
    if children:
        plugin.text = '\n          '
        for child in children[:-1]:
            child.tail = '\n          '
        children[-1].tail = '\n        '
    if hasattr(ET, 'indent'):
        ET.indent(tree, space='  ')
    tree.write(sdf_path, encoding='utf-8', xml_declaration=True)

    if updated:
        print(f"[SDF Update] Modified {len(updated)} parameters in {sdf_path}:")
        for line in updated:
            print(line)
    else:
        print(f"[SDF Update] No changes needed in {sdf_path}")


def main():
    parser = argparse.ArgumentParser(
        description="Update GPS spoof SDF from YAML config before simulation."
    )
    parser.add_argument(
        '--config',
        default='/home/lxx/LPV_ws/src/lpv_attack_sim/config/gps_spoof_params.yaml',
        help='Path to YAML config file'
    )
    parser.add_argument(
        '--scenario',
        default=None,
        help='Named scenario from config (mild/moderate/severe/asymmetric)'
    )
    parser.add_argument(
        '--sdf',
        default='/home/lxx/PX4_Firmware/Tools/sitl_gazebo/models/gps_spoof/gps_spoof.sdf',
        help='Path to GPS spoof SDF file'
    )
    args = parser.parse_args()

    if not os.path.exists(args.config):
        print(f"Error: Config file not found: {args.config}", file=sys.stderr)
        sys.exit(1)

    if not os.path.exists(args.sdf):
        print(f"Error: SDF file not found: {args.sdf}", file=sys.stderr)
        sys.exit(1)

    try:
        params = load_config(args.config, args.scenario)
        update_sdf(args.sdf, params)
        print("[Success] SDF updated. Ready to launch simulation.")
    except Exception as e:
        print(f"Error: {e}", file=sys.stderr)
        sys.exit(1)


if __name__ == '__main__':
    main()
