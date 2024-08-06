import os;
from launch import LaunchDescription;
from launch_ros.actions import Node;
from ament_index_python.packages import get_package_share_directory;


def generate_launch_description() -> LaunchDescription:
    ld: LaunchDescription = LaunchDescription();

    package_name: str = "r2ps_system_controller";
    package_shared_directory: str = get_package_share_directory(package_name);

    r2ps_system_controller: Node = Node(
        package=package_name,
        executable=package_name,
        name=package_name,
        output="screen",
        parameters=[]
    );

    ld.add_action(r2ps_system_controller);

    return ld;
