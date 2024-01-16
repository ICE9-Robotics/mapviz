#include <ros/ros.h>

#include "origin_transform/origin_transform.h"
#include "origin_transform/OriginTransformConfig.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "origin_transform_node");

    ROS_INFO("origin_transform_node initializing...");
    origin_transform::OriginTransform originTransform;
    ROS_INFO("origin_transform_node initialized!");

    ros::spin();
    return 0;
}
