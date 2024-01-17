#include "origin_transform/origin_transform.h"

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

using namespace origin_transform;

OriginTransform::OriginTransform() :
    nh_("~"),
    tf_timeout(ros::Duration(0)),
    tf_listener(tf_buffer),
    utm_transform_ok(false),
    transform_ok(false)
{
    double _tf_timeout, tf_pub_rate;
    nh_.param<std::string>("utm_frame", utmFrame, "utm");
    nh_.param<std::string>("map_frame", mapFrame, "map");
    nh_.param<std::string>("mapviz_fixed_frame", mapvizFrame, "origin");
    nh_.param<double>("tf_timeout", _tf_timeout, 1.0);
    nh_.param<double>("tf_publish_rate", tf_pub_rate, 10.0);
    nh_.param<double>("offset_x", offset_x, 0.0);
    nh_.param<double>("offset_y", offset_y, 0.0);
    nh_.param<double>("offset_yaw", offset_yaw, 0.0);
    tf_timeout.fromSec(_tf_timeout);

    ROS_INFO("utm_frame: %s", utmFrame.c_str());
    ROS_INFO("map_frame: %s", mapFrame.c_str());
    ROS_INFO("mapviz_fixed_frame: %s", mapvizFrame.c_str());
    ROS_INFO("tf_timeout: %.3f", tf_timeout.toSec());
    ROS_INFO("tf_publish_rate: %.1f Hz", tf_pub_rate);
    ROS_INFO("offset_x: %.3f", offset_x);
    ROS_INFO("offset_y: %.3f", offset_y);
    ROS_INFO("offset_yaw: %.3f", offset_yaw);

    dynamicReconfigCallbackType = boost::bind(&OriginTransform::dynamicReconfigCallback, this, _1, _2);
    dynamicReconfigServer.setCallback(dynamicReconfigCallbackType);
    
    OriginTransformConfig config;
    config.offset_x = 0;
    config.offset_y = 0;
    config.offset_yaw = 0;
    dynamicReconfigServer.setConfigDefault(config);

    periodic_timer = nh_.createTimer(ros::Duration(1.0 / tf_pub_rate), &OriginTransform::periodicUpdate, this);
}

OriginTransform::~OriginTransform()
{
}

void OriginTransform::periodicUpdate(const ros::TimerEvent &)
{
    if (!utm_transform_ok)
    {
        try
        {
            getUtmTransform(ros::Time(0));
        }
        catch (tf2::TransformException &ex)
        {
            ROS_WARN("%s",ex.what());
        }
    }
    if (!transform_ok)
    {
        computeTransform();
    }
    if (transform_ok)
    { 
        publishTransform();
    }
}

void OriginTransform::getUtmTransform(ros::Time time)
{
    tf2::Transform trans;
    if (!tf_buffer.canTransform(utmFrame, mapFrame, time))
    {
        ROS_WARN_THROTTLE(5, "Cannot get UTM transform");
        return;
    }
    tf2::fromMsg(tf_buffer.lookupTransform(utmFrame, mapFrame, time).transform, trans);
    tf_quat = trans.getRotation();
    utm_transform_ok = true;
    transform_ok = false;
    ROS_INFO("Got UTM rotation: [%f, %f, %f, %f]", tf_quat.getX(), tf_quat.getY(), tf_quat.getZ(), tf_quat.getW());
}

void OriginTransform::computeTransform()
{
    if (!utm_transform_ok)
    {
        return;
    }
    // rotate tf_quat by offset_yaw
    tf2::Quaternion offset_quat;
    offset_quat.setRPY(0.0, 0.0, offset_yaw);
    tf2::Quaternion q;
    q = tf_quat * offset_quat;
    q.normalize();

    tf_transform.setOrigin(tf2::Vector3(offset_x, offset_y, 0.0));
    tf_transform.setRotation(q);
    transform_ok = true;
    ROS_INFO("Transform computed for x_offset = %.3f, y_offset = %.3f, yaw_offset = %.3f", offset_x, offset_y, offset_yaw);
}

void OriginTransform::publishTransform()
{
    geometry_msgs::TransformStamped static_transformStamped;
    static_transformStamped.header.stamp = ros::Time::now();
    static_transformStamped.header.frame_id = mapvizFrame;
    static_transformStamped.child_frame_id = mapFrame;
    static_transformStamped.transform.translation = tf2::toMsg(tf_transform.getOrigin());
    static_transformStamped.transform.rotation = tf2::toMsg(tf_transform.getRotation());
    ROS_INFO_ONCE("Publishing transform @%f: \ntranslation: x = %f, y = %f, z = %f\nrotation: x = %f, y = %f, z = %f, w = %f", 
            static_transformStamped.header.stamp.toSec(),
            static_transformStamped.transform.translation.x, 
            static_transformStamped.transform.translation.y, 
            static_transformStamped.transform.translation.z, 
            static_transformStamped.transform.rotation.x, 
            static_transformStamped.transform.rotation.y, 
            static_transformStamped.transform.rotation.z, 
            static_transformStamped.transform.rotation.w);

    static_broadcaster.sendTransform(static_transformStamped);
}

void OriginTransform::dynamicReconfigCallback(origin_transform::OriginTransformConfig &config, uint32_t level)
{
    ROS_INFO("Reconfigure Request: x_offset = %f, y_offset = %f, yaw_offset = %f", config.offset_x, config.offset_y, config.offset_yaw);
    offset_x = config.offset_x;
    offset_y = config.offset_y;
    offset_yaw = config.offset_yaw;
    transform_ok = false;
}