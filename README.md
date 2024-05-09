# camera_lidar_fusion
This is a custom package to fuse the lidar point cloud and camera images ina way to fuse the color from camera images with the lidar point clouds.

The function pcl2pxl.py is the function which projects the 3d lidar points 2d

The fusion2pcl2.py usues this projection to extract respective color and add into the custom PointCloud2 data field.
Custom PointCloud2 has PointField (x, y, z, rgba) 

To use this code with simulation or real sensors:
    1. Customise the pcl2pxl.py as per setup.
    2. In the fusion2pcl2.py adjust the topic names.
    3. Dependency is only numpy and ROS sensor_msgs

My Setup:
    ROS Noetic
    Gazebo 11
