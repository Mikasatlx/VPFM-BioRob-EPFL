from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bed1_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("furniture_descriptions"),
                    "urdf/hospital_bed.urdf.xacro",
                ]
            ),
            " ",
            "prefix:=h_bed1_ ",
            "connected_to:='' ",
            "xyz:='0 -1.5 0' ",
            "rpy:='1.570796327 0 1.570796327' ",
            "fixed:='1' ",
        ]
    )
    bed1_description = {"robot_description": bed1_description_content}

    bed2_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("furniture_descriptions"),
                    "urdf/hospital_bed.urdf.xacro",
                ]
            ),
            " ",
            "prefix:=h_bed2_ ",
            "connected_to:='' ",
            "xyz:='0 1.5 0' ",
            "rpy:='1.570796327 0 1.570796327' ",
            "fixed:='1' ",
        ]
    )
    bed2_description = {"robot_description": bed2_description_content}

    bed3_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("furniture_descriptions"),
                    "urdf/hospital_bed.urdf.xacro",
                ]
            ),
            " ",
            "prefix:=h_bed3_ ",
            "connected_to:='' ",
            "xyz:='2 -1.5 0' ",
            "rpy:='1.570796327 0 1.570796327' ",
            "fixed:='1' ",
        ]
    )
    bed3_description = {"robot_description": bed3_description_content}

    bed4_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("furniture_descriptions"),
                    "urdf/hospital_bed.urdf.xacro",
                ]
            ),
            " ",
            "prefix:=h_bed4_ ",
            "connected_to:='' ",
            "xyz:='2 1.5 0' ",
            "rpy:='1.570796327 0 1.570796327' ",
            "fixed:='1' ",
        ]
    )
    bed4_description = {"robot_description": bed4_description_content}

    qolo_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("furniture_descriptions"), "urdf/qolo.urdf.xacro"]
            ),
            " ",
            "prefix:=qolo_ ",
            "connected_to:='' ",
            "xyz:='-1.5 0 0.2' ",
            "rpy:='0 0 0' ",
            "fixed:='1' ",
        ]
    )
    qolo_description = {"robot_description": qolo_description_content}

    bed1_state_pub_node = Node(
        package="robot_state_publisher",
        namespace="bed1",
        executable="robot_state_publisher",
        output="both",
        parameters=[bed1_description],
    )

    bed2_state_pub_node = Node(
        package="robot_state_publisher",
        namespace="bed2",
        executable="robot_state_publisher",
        output="both",
        parameters=[bed2_description],
    )

    bed3_state_pub_node = Node(
        package="robot_state_publisher",
        namespace="bed3",
        executable="robot_state_publisher",
        output="both",
        parameters=[bed3_description],
    )

    bed4_state_pub_node = Node(
        package="robot_state_publisher",
        namespace="bed4",
        executable="robot_state_publisher",
        output="both",
        parameters=[bed4_description],
    )

    qolo_state_pub_node = Node(
        package="robot_state_publisher",
        namespace="qolo",
        executable="robot_state_publisher",
        output="both",
        parameters=[qolo_description],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=[
            "-d",
            PathJoinSubstitution(
                [
                    FindPackageShare("furniture_descriptions"),
                    "rviz/multiple_hospital.rviz",
                ]
            ),
        ],
        output="log",
    )

    nodes = [
        bed1_state_pub_node,
        bed2_state_pub_node,
        bed3_state_pub_node,
        bed4_state_pub_node,
        qolo_state_pub_node,
        rviz_node,
    ]

    return LaunchDescription(nodes)
