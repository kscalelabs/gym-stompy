from pathlib import Path

### Simulation envs fixed constants
DT = 0.02  # 0.02 ms -> 1/0.2 = 50 hz
FPS = 50

# NOTE: must be absolute path
ASSETS_DIR = Path(__file__).parent.resolve() / "assets"

# JOINTS = [
#     "joint_right_arm_1_x8_1_dof_x8",
#     "joint_right_arm_1_x8_2_dof_x8",
#     "joint_right_arm_1_x6_1_dof_x6",
#     "joint_right_arm_1_x6_2_dof_x6",
#     "joint_right_arm_1_x4_1_dof_x4",
#     "joint_right_arm_1_hand_1_x4_1_dof_x4",
#     "joint_right_arm_1_hand_1_slider_1",
#     "joint_right_arm_1_hand_1_slider_2",
#     # "joint_right_arm_1_hand_1_x4_2_dof_x4",
#     "joint_left_arm_2_x8_1_dof_x8",
#     "joint_left_arm_2_x8_2_dof_x8",
#     "joint_left_arm_2_x6_1_dof_x6",
#     "joint_left_arm_2_x6_2_dof_x6",
#     "joint_left_arm_2_x4_1_dof_x4",
#     "joint_left_arm_2_hand_1_x4_1_dof_x4",
#     "joint_left_arm_2_hand_1_slider_1",
#     "joint_left_arm_2_hand_1_slider_2",
#     # "joint_left_arm_2_hand_1_x4_2_dof_x4",
# ]
JOINTS = [
    # absolute joint position
    "left_arm_waist",
    "left_arm_shoulder",
    "left_arm_elbow",
    "left_arm_forearm_roll",
    "left_arm_wrist_angle",
    "left_arm_wrist_rotate",
    # normalized gripper position 0: close, 1: open
    "left_arm_gripper",
    # absolute joint position
    "right_arm_waist",
    "right_arm_shoulder",
    "right_arm_elbow",
    "right_arm_forearm_roll",
    "right_arm_wrist_angle",
    "right_arm_wrist_rotate",
    # normalized gripper position 0: close, 1: open
    "right_arm_gripper",
]

ACTIONS = [
    # position and quaternion for end effector
    "left_arm_waist",
    "left_arm_shoulder",
    "left_arm_elbow",
    "left_arm_forearm_roll",
    "left_arm_wrist_angle",
    "left_arm_wrist_rotate",
    # normalized gripper position (0: close, 1: open)
    "left_arm_gripper",
    "right_arm_waist",
    "right_arm_shoulder",
    "right_arm_elbow",
    "right_arm_forearm_roll",
    "right_arm_wrist_angle",
    "right_arm_wrist_rotate",
    # normalized gripper position (0: close, 1: open)
    "right_arm_gripper",
]

START_ARM_POSE = [
    # left arm (6dof)
    -1.7,
    -1.6,
    -0.34,
    -1.6,
    -1.4,
    -1.7,
    # left gripper
    0.0,
    0.0,
    # right arm (6dof)
    1.7,
    1.6,
    0.34,
    1.6,
    1.4,
    -0.26,
    # right gripper
    0.0,
    0.0,
]


# Left finger position limits (qpos[7]), right_finger = -1 * left_finger
MASTER_GRIPPER_POSITION_OPEN = 0.02417
MASTER_GRIPPER_POSITION_CLOSE = 0.01244
PUPPET_GRIPPER_POSITION_OPEN = 0.05800
PUPPET_GRIPPER_POSITION_CLOSE = 0.01844

# Gripper joint limits (qpos[6])
MASTER_GRIPPER_JOINT_OPEN = 0.3083
MASTER_GRIPPER_JOINT_CLOSE = -0.6842
PUPPET_GRIPPER_JOINT_OPEN = 1.4910
PUPPET_GRIPPER_JOINT_CLOSE = -0.6213

MASTER_GRIPPER_JOINT_MID = (MASTER_GRIPPER_JOINT_OPEN + MASTER_GRIPPER_JOINT_CLOSE) / 2

############################ Helper functions ############################


def normalize_master_gripper_position(x):
    return (x - MASTER_GRIPPER_POSITION_CLOSE) / (
        MASTER_GRIPPER_POSITION_OPEN - MASTER_GRIPPER_POSITION_CLOSE
    )


def normalize_puppet_gripper_position(x):
    return (x - PUPPET_GRIPPER_POSITION_CLOSE) / (
        PUPPET_GRIPPER_POSITION_OPEN - PUPPET_GRIPPER_POSITION_CLOSE
    )


def unnormalize_master_gripper_position(x):
    return x * (MASTER_GRIPPER_POSITION_OPEN - MASTER_GRIPPER_POSITION_CLOSE) + MASTER_GRIPPER_POSITION_CLOSE


def unnormalize_puppet_gripper_position(x):
    return x * (PUPPET_GRIPPER_POSITION_OPEN - PUPPET_GRIPPER_POSITION_CLOSE) + PUPPET_GRIPPER_POSITION_CLOSE


def convert_position_from_master_to_puppet(x):
    return unnormalize_puppet_gripper_position(normalize_master_gripper_position(x))


def normalizer_master_gripper_joint(x):
    return (x - MASTER_GRIPPER_JOINT_CLOSE) / (MASTER_GRIPPER_JOINT_OPEN - MASTER_GRIPPER_JOINT_CLOSE)


def normalize_puppet_gripper_joint(x):
    return (x - PUPPET_GRIPPER_JOINT_CLOSE) / (PUPPET_GRIPPER_JOINT_OPEN - PUPPET_GRIPPER_JOINT_CLOSE)


def unnormalize_master_gripper_joint(x):
    return x * (MASTER_GRIPPER_JOINT_OPEN - MASTER_GRIPPER_JOINT_CLOSE) + MASTER_GRIPPER_JOINT_CLOSE


def unnormalize_puppet_gripper_joint(x):
    return x * (PUPPET_GRIPPER_JOINT_OPEN - PUPPET_GRIPPER_JOINT_CLOSE) + PUPPET_GRIPPER_JOINT_CLOSE


def convert_join_from_master_to_puppet(x):
    return unnormalize_puppet_gripper_joint(normalizer_master_gripper_joint(x))


def normalize_master_gripper_velocity(x):
    return x / (MASTER_GRIPPER_POSITION_OPEN - MASTER_GRIPPER_POSITION_CLOSE)


def normalize_puppet_gripper_velocity(x):
    return x / (PUPPET_GRIPPER_POSITION_OPEN - PUPPET_GRIPPER_POSITION_CLOSE)


def convert_master_from_position_to_joint(x):
    return (
        normalize_master_gripper_position(x) * (MASTER_GRIPPER_JOINT_OPEN - MASTER_GRIPPER_JOINT_CLOSE)
        + MASTER_GRIPPER_JOINT_CLOSE
    )


def convert_master_from_joint_to_position(x):
    return unnormalize_master_gripper_position(
        (x - MASTER_GRIPPER_JOINT_CLOSE) / (MASTER_GRIPPER_JOINT_OPEN - MASTER_GRIPPER_JOINT_CLOSE)
    )


def convert_puppet_from_position_to_join(x):
    return (
        normalize_puppet_gripper_position(x) * (PUPPET_GRIPPER_JOINT_OPEN - PUPPET_GRIPPER_JOINT_CLOSE)
        + PUPPET_GRIPPER_JOINT_CLOSE
    )


def convert_puppet_from_joint_to_position(x):
    return unnormalize_puppet_gripper_position(
        (x - PUPPET_GRIPPER_JOINT_CLOSE) / (PUPPET_GRIPPER_JOINT_OPEN - PUPPET_GRIPPER_JOINT_CLOSE)
    )
