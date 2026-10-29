from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    num_goals = LaunchConfiguration("num_goals")
    capture_images = LaunchConfiguration("capture_images")
    image_dir = LaunchConfiguration("image_dir")

    return LaunchDescription([
        DeclareLaunchArgument("num_goals", default_value="0",
                              description="Number of goals to reach (0 = infinite)"),
        DeclareLaunchArgument("capture_images", default_value="true",
                              description="Save a camera image on each reached goal"),
        DeclareLaunchArgument("image_dir", default_value="~/ros2_rover/captured_images",
                              description="Directory for captured images"),
        Node(
            package="rover_navigation",
            executable="explorer_node",
            name="explorer_node",
            output="screen",
            parameters=[{
                "use_sim_time": True,
                "num_goals": num_goals,
                "capture_images": capture_images,
                "image_dir": image_dir,
            }],
        ),
    ])
