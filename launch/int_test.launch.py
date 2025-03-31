from launch import LaunchDescription  
from launch_ros.actions import Node  

def generate_launch_description():  
    """Generate the launch description for ROS 2 integer publisher and subscriber."""  
    
    # Define the list of nodes to be launched  
    nodes = [  
        Node(  
            package='ros2_action_mux',  
            executable='int_publisher',  
            name='int_publisher',  
            output='screen',  
            parameters=[  
                {'topic': '/int_topic'},          # Topic to publish integers  
                {'publish_rate': 1.0},           # Publish rate in Hz  
                {'start_value': 42},             # Initial value for publishing  
                {'increment': 5}                 # Increment value for each message  
            ]  
        ),  
        Node(  
            package='ros2_action_mux',  
            executable='generic_subscriber',  
            name='generic_subscriber',  
            output='screen',  
            parameters=[  
                {'topics': ['/int_topic']}       # Topic to subscribe to for receiving integers  
            ]  
        )  
    ]  
    
    # Return the configured launch description with the nodes  
    return LaunchDescription(nodes)  