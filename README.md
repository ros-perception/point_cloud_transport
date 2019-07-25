# ASSIGNMENT
# Requirements
Implement and evaluate practical point cloud compression for ROS, with C++ and Python API.\
Use plugin interface ala http://wiki.ros.org/image_transport or at least a compatible API for possible later extensions.\
All work needed should be using dedicated publishers or subscribers from interface package "point_cloud_transport", with optional configuration.

By default, apply the common ROS tweaks for float32 "rgb" and "rgba" fields as described in RViz documentation and source code. For sake of compression, interpret such fields as three/four separate uint8 channels and reconstruct single float32 after decompression.
If time permits, implement converting republishers as in image_transport (raw / compressed / theora) to accommodate plain point cloud subscribers / publishers.

# References
### SW
Draco 3D data compression\
https://github.com/google/draco \
PCL (better to avoid as dependency in final package) compression tutorial\
http://pointclouds.org/documentation/tutorials/compression.php \
(Open 3D Graphics Compression (Open3DGC), for reference\ 
https://github.com/amd/rest3d/tree/master/server/o3dgc, https://github.com/KhronosGroup/glTF/wiki/Open-3D-Graphics-Compression) 
### ROS
ROS message type definition\ 
http://docs.ros.org/melodic/api/sensor_msgs/html/msg/PointCloud2.html \
Image and PointCloud2 performance issue on ROS2 python\
https://discourse.ros.org/t/image-and-pointcloud2-performance-issue-on-ros2-python/5391
### Papers
Zhang (2014): Point cloud attribute compression with graph transform\
https://ieeexplore.ieee.org/abstract/document/7025414 \
Kammerl (2012): Real-time compression of point cloud streams\
https://ieeexplore.ieee.org/abstract/document/6224647 \
Schnabel (2006): Octree-based Point-cloud Compression\
http://citeseerx.ist.psu.edu/viewdoc/download?doi=10.1.1.71.5637&rep=rep1&type=pdf \
Gumhold (2005): Predictive Point-Cloud Compression\
http://www.cs.technion.ac.il/~zachik/publications/2005_pccomp-ik.pdf
### Data (bag files)
https://google-cartographer-ros.readthedocs.io/en/latest/demos.html
