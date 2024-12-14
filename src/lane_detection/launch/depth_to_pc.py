from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

   
    pcl_node = Node(
        package='depth_image_proc',
        plugin='depth_image_proc::PointCloudXyzNode',
        name='point_cloud_xyzr_node',
        remappings=[
            ('depth_registered/image_rect', '/zed/zed_node/depth/depth_registered'),
            ('points', 'depth_registered/points'),
        ],
    )
    launchDescriptionObject = LaunchDescription()

    launchDescriptionObject.add_action(pcl_node)

                

    return launchDescriptionObject