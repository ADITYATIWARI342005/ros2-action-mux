from launch import LaunchDescription  
from launch_ros.actions import Node  

def generate_launch_description():  
    """Generate the launch description for a pose publisher and subscriber in ROS 2."""  
    
    # Define the list of nodes to launch  
    nodes = [  
        Node(  
            package='ros2_action_mux',  
            executable='pose_publisher',  
            name='pose_publisher',  
            output='screen',  
            parameters=[  
                {'topic': '/pose_topic'},           # Topic for publishing pose data  
                {'publish_rate': 1.0},              # Frequency at which to publish poses (in Hz)  
                {'circular_motion': True}            # Flag to indicate if motion should be circular  
            ]  
        ),  
        Node(  
            package='ros2_action_mux',  
            executable='generic_subscriber',  
            name='generic_subscriber',  
            output='screen',  
            parameters=[  
                {'topics': ['/pose_topic']}          # Subscribe to pose data published on this topic  
            ]  
        )  
    ]  
    
    # Return the constructed launch description containing the nodes  
    return LaunchDescription(nodes)  