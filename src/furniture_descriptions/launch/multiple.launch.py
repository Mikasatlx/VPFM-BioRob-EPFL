from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    table_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("furniture_descriptions"), "urdf/table.urdf.xacro"]
            ),
            " ",
            "prefix:=table_ ",
            "connected_to:='' ",
            "xyz:='0 -0.2 0' ",
            "rpy:='0 0 0' ",
            "fixed:='1' ",
        ]
    )
    table_description = {"robot_description": table_description_content}

    chair1_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("furniture_descriptions"), "urdf/chair.urdf.xacro"]
            ),
            " ",
            "prefix:=chair1_ ",
            "connected_to:='' ",
            "xyz:='-0.4 -0.5 0' ",
            "rpy:='0 0 1.570796327' ",
            "fixed:='1' ",
        ]
    )
    chair1_description = {"robot_description": chair1_description_content}

    chair2_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("furniture_descriptions"), "urdf/chair.urdf.xacro"]
            ),
            " ",
            "prefix:=chair2_ ",
            "connected_to:='' ",
            "xyz:='0.4 -0.5 0' ",
            "rpy:='0 0 1.570796327' ",
            "fixed:='1' ",
        ]
    )
    chair2_description = {"robot_description": chair2_description_content}

    chair3_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("furniture_descriptions"), "urdf/chair.urdf.xacro"]
            ),
            " ",
            "prefix:=chair3_ ",
            "connected_to:='' ",
            "xyz:='-0.4 0.5 0' ",
            "rpy:='0 0 -1.570796327' ",
            "fixed:='1' ",
        ]
    )
    chair3_description = {"robot_description": chair3_description_content}

    chair4_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("furniture_descriptions"), "urdf/chair.urdf.xacro"]
            ),
            " ",
            "prefix:=chair4_ ",
            "connected_to:='' ",
            "xyz:='0.4 0.5 0' ",
            "rpy:='0 0 -1.570796327' ",
            "fixed:='1' ",
        ]
    )
    chair4_description = {"robot_description": chair4_description_content}

    wheelchair_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("furniture_descriptions"), "urdf/wheelchair.urdf.xacro"]
            ),
            " ",
            "prefix:=wheelchair_ ",
            "connected_to:='' ",
            "xyz:='-1.5 0 0' ",
            "rpy:='0 0 0' ",
            "fixed:='1' ",
        ]
    )
    wheelchair_description = {"robot_description": wheelchair_description_content}

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

    table_state_pub_node = Node(
        package="robot_state_publisher",
        namespace="table",
        executable="robot_state_publisher",
        output="both",
        parameters=[table_description],
    )

    chair1_state_pub_node = Node(
        package="robot_state_publisher",
        namespace="chair1",
        executable="robot_state_publisher",
        output="both",
        parameters=[chair1_description],
    )

    chair2_state_pub_node = Node(
        package="robot_state_publisher",
        namespace="chair2",
        executable="robot_state_publisher",
        output="both",
        parameters=[chair2_description],
    )

    chair3_state_pub_node = Node(
        package="robot_state_publisher",
        namespace="chair3",
        executable="robot_state_publisher",
        output="both",
        parameters=[chair3_description],
    )

    chair4_state_pub_node = Node(
        package="robot_state_publisher",
        namespace="chair4",
        executable="robot_state_publisher",
        output="both",
        parameters=[chair4_description],
    )

    wheelchair_state_pub_node = Node(
        package="robot_state_publisher",
        namespace="wheelchair",
        executable="robot_state_publisher",
        output="both",
        parameters=[wheelchair_description],
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
                [FindPackageShare("furniture_descriptions"), "rviz/multiple_obj.rviz"]
            ),
        ],
        output="log",
    )

    nodes = [
        table_state_pub_node,
        chair1_state_pub_node,
        chair2_state_pub_node,
        chair3_state_pub_node,
        chair4_state_pub_node,
        # wheelchair_state_pub_node,
        qolo_state_pub_node,
        rviz_node,
    ]

    return LaunchDescription(nodes)
