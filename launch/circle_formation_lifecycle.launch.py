from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import LifecycleNode
import os

def generate_launch_description():
    ld = LaunchDescription()
    pkg = FindPackageShare('quadrotor_formation').find('quadrotor_formation')

    # start gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                FindPackageShare('gazebo_ros').find('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            )
        )
    )

    # summon quadrotors
    summon_quadrotors = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg, 'launch', 'summon_quadrotors.launch.py')
        ),
        launch_arguments={'x_min': '-15', 'x_max': '15', 'y_min': '-15', 'y_max': '15'}.items()
    )

    # start circle formation lifecycle node
    circle_formation_lifecycle = LifecycleNode(
        package='quadrotor_formation',
        executable='circle_formation_lifecycle_node',
        name='circle_formation_lifecycle_node',
        output='screen',
        namespace=''
    )

    ld.add_action(gazebo)
    ld.add_action(summon_quadrotors)
    ld.add_action(circle_formation_lifecycle)

    return ld