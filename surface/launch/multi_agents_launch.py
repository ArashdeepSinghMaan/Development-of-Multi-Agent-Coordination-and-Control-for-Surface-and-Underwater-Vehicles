from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    agent_names = [f"my_lrauv_{i}" for i in range(1, 9)]

    decision_nodes = [
        Node(
            package="surface",
            executable="decision_making",
            name="decision_making",
            namespace=agent,
            parameters=[{"robot_name": agent}],
            output="screen",
            remappings=[
                ("allocated_tasks", "/global/allocated_tasks"),  # GLOBAL remap
                ("task_allocation_discussion", "/global/task_allocation_discussion"),
            ],
        )
        for agent in agent_names
    ]

    # Example aggregator_node definition (if needed)
    aggregator_node = [
        Node(
            package="surface",
            executable="aggregator_node",
            name="aggregator",
            output="screen",
        )
    ]
    task_publisher = [
        Node(
            package="surface",
            executable="task_publisher",
            name="task_publisher",
            output="screen",
        )
    ]
    
    nav_nodes = [
    Node(
        package="surface",
        executable="task_allocated_multi_agent",
        name="navigation_with_decision",
        namespace=agent,
        parameters=[{"robot_name": agent}],
        output="screen",
    )
    for agent in agent_names
]



    return LaunchDescription(decision_nodes + aggregator_node +task_publisher + nav_nodes)
