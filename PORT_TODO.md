## TODO
- [] start roscpp --> rclcpp conversion of transport
--> Put CRAS into this repo temporarily to get it to build
--> Do what it takes to get the rclcpp code to build without the point_cloud_codec files
--> Figure out how to update ros serialization and shape shifter usage
--> Fix the point_cloud_codec files
- [] c++ codebase builds with cmake (minus tests)
--> Create a node to test with simulated pointclouds
--> bag the pointcloud data
--> Test point_cloud_codec files with bagged point cloud data
- [] c++ codebase runs with test pointcloudsc
--> Fix c++ tests
- [] tests build and pass
- [] rospy --> rclpy conversion
--> Update python rclpy files
--> Update python bindings
- [] python codebase runs with test pointclouds

