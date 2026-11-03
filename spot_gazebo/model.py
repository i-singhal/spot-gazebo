"""Mapping between SDK joint abbreviations and the upstream Spot URDF names."""
LEGS = {'fl': 'front_left', 'fr': 'front_right', 'hl': 'rear_left', 'hr': 'rear_right'}
AXES = {'hx': 'hip_x', 'hy': 'hip_y', 'kn': 'knee'}
JOINT_NAMES = {f'{leg}_{axis}': f'{prefix}_{suffix}'
               for leg, prefix in LEGS.items() for axis, suffix in AXES.items()}
URDF_TO_ALIAS = {value: key for key, value in JOINT_NAMES.items()}
STAND_HEIGHT = .55
SIT_HEIGHT = .27
STAND_HIP, STAND_KNEE = .65, -1.3
SIT_HIP, SIT_KNEE = 1.16, -2.32
