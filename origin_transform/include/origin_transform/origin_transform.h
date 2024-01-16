#ifndef MAPVIZ_ORIGIN_TRANSFORM_H_
#define MAPVIZ_ORIGIN_TRANSFORM_H_

#include <ros/ros.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <dynamic_reconfigure/server.h>

#include "origin_transform/OriginTransformConfig.h"

namespace origin_transform {

class OriginTransform
{
  public:
    OriginTransform();
    ~OriginTransform();

    void periodicUpdate(const ros::TimerEvent &);
    void getUtmTransform(ros::Time time);
    void computeTransform();
    void publishTransform();

    void dynamicReconfigCallback(origin_transform::OriginTransformConfig &config, uint32_t level);

  // private:
    ros::NodeHandle nh_;
    std::string utmFrame;
    std::string mapFrame;
    std::string mapvizFrame;
    ros::Duration tf_timeout;
    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener;
    tf2::Quaternion tf_quat;
    tf2::Transform tf_transform;
    ros::Timer periodic_timer;
    tf2_ros::StaticTransformBroadcaster static_broadcaster;

    dynamic_reconfigure::Server<origin_transform::OriginTransformConfig>::CallbackType dynamicReconfigCallbackType;
    dynamic_reconfigure::Server<origin_transform::OriginTransformConfig> dynamicReconfigServer;

    double offset_x;
    double offset_y;
    double offset_yaw;
    bool utm_transform_ok;
    bool transform_ok;
};

}
#endif  // MAPVIZ_ORIGIN_TRANSFORM_H_