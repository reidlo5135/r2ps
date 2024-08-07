import os;
from launch import LaunchDescription;
from launch_ros.actions import Node;
from ament_index_python.packages import get_package_share_directory;
from launch.actions import IncludeLaunchDescription;
from launch.launch_description_sources import PythonLaunchDescriptionSource;


def generate_launch_description() -> LaunchDescription:
    ld: LaunchDescription = LaunchDescription();

    r2ps_system_controller_package: str = "r2ps_system_controller";
    r2ps_system_controller_dir: str = get_package_share_directory(r2ps_system_controller_package);
    r2ps_system_controller_launch_file: str = os.path.join(r2ps_system_controller_dir, "launch", f"{r2ps_system_controller_package}.launch.py");
    r2ps_system_controller_launch: IncludeLaunchDescription = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            r2ps_system_controller_launch_file
        )
    );

    r2ps_launch_operator_package: str = "r2ps_launch_operator";
    r2ps_launch_operator_dir: str = get_package_share_directory(r2ps_launch_operator_package);
    r2ps_launch_operator_lauch_file: str = os.path.join(r2ps_launch_operator_dir, "launch", f"{r2ps_launch_operator_package}.launch.py");
    r2ps_launch_operator_launch: IncludeLaunchDescription = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            r2ps_launch_operator_lauch_file
        )   
    );  

    ld.add_action(r2ps_system_controller_launch);
    ld.add_action(r2ps_launch_operator_launch);

    return ld;

