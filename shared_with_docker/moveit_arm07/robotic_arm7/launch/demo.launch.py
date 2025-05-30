from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("robotic_arm", package_name="robotic_arm7").to_moveit_configs()
    return generate_demo_launch(moveit_config)
