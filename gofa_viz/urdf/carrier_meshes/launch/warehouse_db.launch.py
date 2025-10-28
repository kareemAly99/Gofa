from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_warehouse_db_launch


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("gofa_with_carrier", package_name="carrier_meshes").to_moveit_configs()
    return generate_warehouse_db_launch(moveit_config)
