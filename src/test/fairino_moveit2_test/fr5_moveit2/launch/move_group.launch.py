from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("fr5_v6_robot", package_name="fr5_moveit2").to_moveit_configs()
    return generate_move_group_launch(moveit_config)
