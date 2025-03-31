from launch import LaunchDescription  
from launch_ros.actions import Node  

def generate_launch_description():  
    """Generate the launch description for a string publisher and subscriber in ROS 2."""  
    
    # Define the list of nodes that will be launched  
    nodes = [  
        Node(  
            package='ros2_action_mux',  
            executable='string_publisher',  
            name='string_publisher',  
            output='screen',  
            parameters=[  
                {'topic': '/trigger_topic'},          # Topic for publishing strings  
                {'publish_rate': 1.0},                # Frequency at which to publish (in Hz)  
                {'message_prefix': 'String Test'}     # Prefix for the messages being published  
            ]  
        ),  
        Node(  
            package='ros2_action_mux',  
            executable='generic_subscriber',  
            name='generic_subscriber',  
            output='screen',  
            parameters=[  
                {'topics': ['/trigger_topic']}         # Subscribe to messages from the specified topic  
            ]  
        )  
    ]  
    
    # Return the constructed launch description containing the specified nodes  
    return LaunchDescription(nodes)  