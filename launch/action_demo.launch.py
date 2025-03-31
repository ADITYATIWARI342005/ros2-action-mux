from launch import LaunchDescription  
from launch_ros.actions import Node  

def generate_launch_description():  
    """Generate the launch description for the ROS 2 action multiplexing example."""  
    
    # Create a list of nodes to be included in the launch description  
    nodes = [  
        Node(  
            package='ros2_action_mux',  
            executable='action_server',  
            name='action_server',  
            output='screen',  
            # The action server handles action requests  
        ),  
        Node(  
            package='ros2_action_mux',  
            executable='action_client',  
            name='action_client',  
            output='screen',  
            parameters=[  
                {'trigger_topic': '/trigger_topic'}  # Topic to trigger actions  
            ]  
        ),  
        Node(  
            package='ros2_action_mux',  
            executable='string_publisher',  
            name='string_publisher',  
            output='screen',  
            parameters=[  
                {'topic': '/trigger_topic'},  # Topic for publishing messages  
                {'publish_rate': 0.5},  # Frequency of publishing messages (in Hz)  
                {'message_prefix': 'Request'}  # Prefix for the published messages  
            ]  
        ),  
        Node(  
            package='ros2_action_mux',  
            executable='generic_subscriber',  
            name='generic_subscriber',  
            output='screen',  
            parameters=[  
                {'topics': ['/trigger_topic']}  # Subscribe to the specified topic  
            ]  
        )  
    ]  
    
    # Return the complete launch description  
    return LaunchDescription(nodes)  