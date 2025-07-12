import math
import yaml
import os

joint_limits = {
    'elbow_joint': {
        'has_position_limits': True,
        'min_position': math.radians(0),
        'max_position': math.radians(80),
        'has_acceleration_limits': True,
        'max_acceleration': .2,
        'min_acceleration': .2,
    },
    'shoulder_joint': {
        'has_position_limits': True,
        'min_position': math.radians(70),
        'max_position': math.radians(150),
        'has_acceleration_limits': True,
        'max_acceleration': .2,
        'min_acceleration': .2,
    },
    'world_to_base': {
        'has_position_limits': True,
        'min_position': math.radians(20),   # -π/2
        'max_position': math.radians(170),    # π/2
        'has_acceleration_limits': True,
        'max_acceleration': .2,
        'min_acceleration': .2,
    },
    'wrist': {
        'has_position_limits': True,
        'min_position': math.radians(60),
        'max_position': math.radians(120),
        'has_acceleration_limits': True,
        'max_acceleration': .2,
        'min_acceleration': .2,
    },
}

cur_dir = os.path.dirname(os.path.abspath(__file__))
yaml_path = cur_dir +"/joint_limits.yaml"
with open(yaml_path, "w+") as f:
    yaml.dump({"joint_limits": joint_limits}, f, default_flow_style=False)
print(f"Generated {yaml_path}", flush=True)