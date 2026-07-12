from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "input_topic", default_value="/sim/ground_truth"
            ),
            DeclareLaunchArgument(
                "output_topic", default_value="/cg410/odometry"
            ),
            DeclareLaunchArgument("publish_rate", default_value="20.0"),
            DeclareLaunchArgument("frame_id", default_value="map"),
            DeclareLaunchArgument("child_frame_id", default_value="base_link"),
            DeclareLaunchArgument(
                "position_noise_std", default_value="0.05"
            ),
            DeclareLaunchArgument("z_noise_std", default_value="0.0"),
            DeclareLaunchArgument("yaw_noise_std_deg", default_value="0.5"),
            DeclareLaunchArgument(
                "velocity_noise_std", default_value="0.02"
            ),
            DeclareLaunchArgument(
                "angular_velocity_noise_std", default_value="0.0"
            ),
            DeclareLaunchArgument("seed", default_value="42"),
            DeclareLaunchArgument(
                "use_ground_truth_stamp", default_value="true"
            ),
            Node(
                package="ins_simulator",
                executable="ins_simulator",
                name="ins_simulator",
                output="screen",
                parameters=[
                    {
                        "input_topic": LaunchConfiguration("input_topic"),
                        "output_topic": LaunchConfiguration("output_topic"),
                        "publish_rate": ParameterValue(
                            LaunchConfiguration("publish_rate"),
                            value_type=float,
                        ),
                        "frame_id": LaunchConfiguration("frame_id"),
                        "child_frame_id": LaunchConfiguration(
                            "child_frame_id"
                        ),
                        "position_noise_std": ParameterValue(
                            LaunchConfiguration("position_noise_std"),
                            value_type=float,
                        ),
                        "z_noise_std": ParameterValue(
                            LaunchConfiguration("z_noise_std"),
                            value_type=float,
                        ),
                        "yaw_noise_std_deg": ParameterValue(
                            LaunchConfiguration("yaw_noise_std_deg"),
                            value_type=float,
                        ),
                        "velocity_noise_std": ParameterValue(
                            LaunchConfiguration("velocity_noise_std"),
                            value_type=float,
                        ),
                        "angular_velocity_noise_std": ParameterValue(
                            LaunchConfiguration(
                                "angular_velocity_noise_std"
                            ),
                            value_type=float,
                        ),
                        "seed": ParameterValue(
                            LaunchConfiguration("seed"), value_type=int
                        ),
                        "use_ground_truth_stamp": ParameterValue(
                            LaunchConfiguration("use_ground_truth_stamp"),
                            value_type=bool,
                        ),
                    }
                ],
            ),
        ]
    )
