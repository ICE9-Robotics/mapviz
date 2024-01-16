#include <mapviz_plugins/ugi_basestation_plugin.h>
#include <dynamic_reconfigure/Reconfigure.h>


namespace mapviz_plugins
{

UgiBaseStationPluginDynamicParam::UgiBaseStationPluginDynamicParam(ros::NodeHandle nh):
nh_(nh),
origin_transform_param_client_("/origin_transform_node"),
mjpeg_cam_param_client_("/mjpeg_cam"),
local_planner_param_client_("/slam_planner_node/DBLocalPlannerROS"),
is_origin_transform_update_pending_(false),
is_mjpeg_cam_update_pending_(false),
is_local_planner_update_pending_(false),
is_origin_transform_received(false),
is_mjpeg_cam_received(false),
is_local_planner_received(false)
{
    getLatestFromSrv();
    origin_transform_update_timer_ = nh_.createTimer(ros::Duration(1.0), &UgiBaseStationPluginDynamicParam::originTransformTimerCallback, this);
    mjpeg_cam_update_timer_ = nh_.createTimer(ros::Duration(1.0), &UgiBaseStationPluginDynamicParam::mjpegCamTimerCallback, this);
    local_planner_update_timer_ = nh_.createTimer(ros::Duration(1.0), &UgiBaseStationPluginDynamicParam::moveBaseTimerCallback, this);
}

UgiBaseStationPluginDynamicParam::~UgiBaseStationPluginDynamicParam()
{
}

void UgiBaseStationPluginDynamicParam::setOriginTransformParam(const origin_transform::OriginTransformConfig &config)
{
    dynamic_reconfigure::ReconfigureResponse srv_resp;
    boost::mutex::scoped_lock lock(origin_transform_param_mutex_);
    config.__toMessage__(srv_resp.config);
    origin_transform_pending_config_.__fromMessage__(srv_resp.config);
    is_origin_transform_update_pending_ = true;
}

bool UgiBaseStationPluginDynamicParam::getOriginTransformParam(origin_transform::OriginTransformConfig &config)
{   
    if (!is_origin_transform_received)
    {
        getLatestFromSrv();
    }
    if (!is_origin_transform_received)
    {
        return false;
    }
    boost::mutex::scoped_lock lock(origin_transform_param_mutex_);
    config = origin_transform_latest_config_;
    return true;
}

void UgiBaseStationPluginDynamicParam::setMjpegCamParam(const mjpeg_cam::mjpeg_camConfig &config)
{
    ROS_INFO("setMjpegCamParam");
    dynamic_reconfigure::ReconfigureResponse srv_resp;
    boost::mutex::scoped_lock lock(mjpeg_cam_param_mutex_);
    config.__toMessage__(srv_resp.config);
    mjpeg_cam_pending_config_.__fromMessage__(srv_resp.config);
    is_mjpeg_cam_update_pending_ = true;
    ROS_INFO("got config: exposure: %d, brightness: %d, auto: %s", mjpeg_cam_pending_config_.exposure, mjpeg_cam_pending_config_.brightness, mjpeg_cam_pending_config_.autoexposure ? "true" : "false");
}

bool UgiBaseStationPluginDynamicParam::getMjpegCamParam(mjpeg_cam::mjpeg_camConfig &config)
{
    if (!is_mjpeg_cam_received)
    {
        getLatestFromSrv();
    }
    if (!is_mjpeg_cam_received)
    {
        return false;
    }
    boost::mutex::scoped_lock lock(mjpeg_cam_param_mutex_);
    config = mjpeg_cam_latest_config_;
    return true;
}

void UgiBaseStationPluginDynamicParam::setLocalPlannerParam(const improved_local_planner::DBLocalPlannerReconfigureConfig &config)
{
    dynamic_reconfigure::ReconfigureResponse srv_resp;
    boost::mutex::scoped_lock lock(local_planner_param_mutex_);
    config.__toMessage__(srv_resp.config);
    local_planner_pending_config_.__fromMessage__(srv_resp.config);
    is_local_planner_update_pending_ = true;
}

bool UgiBaseStationPluginDynamicParam::getLocalPlannerParam(improved_local_planner::DBLocalPlannerReconfigureConfig &config)
{   
    if (!is_local_planner_received)
    {
        getLatestFromSrv();
    }
    if (!is_local_planner_received)
    {
        return false;
    }
    boost::mutex::scoped_lock lock(local_planner_param_mutex_);
    config = local_planner_latest_config_;
    return true;
}

void UgiBaseStationPluginDynamicParam::getLatestFromSrv()
{
    if (is_local_planner_received && is_mjpeg_cam_received && is_origin_transform_received)
    {
        return;
    }
    dynamic_reconfigure::ReconfigureRequest srv_req;
    dynamic_reconfigure::ReconfigureResponse srv_resp;
    if (!is_origin_transform_received && ros::service::call("/origin_transform_node/set_parameters", srv_req, srv_resp))
    {
        boost::mutex::scoped_lock lock(origin_transform_param_mutex_);
        origin_transform_latest_config_.__fromMessage__(srv_resp.config);
        is_origin_transform_received = true;
        origin_transform_param_mutex_.unlock();
    }
    if (!is_mjpeg_cam_received && ros::service::call("/mjpeg_cam/set_parameters", srv_req, srv_resp))
    {
        boost::mutex::scoped_lock lock(mjpeg_cam_param_mutex_);
        mjpeg_cam_latest_config_.__fromMessage__(srv_resp.config);
        is_mjpeg_cam_received = true;
        mjpeg_cam_param_mutex_.unlock();
    }
    if (!is_local_planner_received && ros::service::call("/slam_planner_node/DBLocalPlannerROS/set_parameters", srv_req, srv_resp))
    {
        boost::mutex::scoped_lock lock(local_planner_param_mutex_);
        local_planner_latest_config_.__fromMessage__(srv_resp.config);
        is_local_planner_received = true;
        local_planner_param_mutex_.unlock();
    }
}

void UgiBaseStationPluginDynamicParam::originTransformTimerCallback(const ros::TimerEvent &)
{
    ROS_INFO("originTransformTimerCallback");
    boost::mutex::scoped_lock lock(origin_transform_param_mutex_);
    if (!is_origin_transform_update_pending_)
    {
        return;
    }

    ROS_INFO("originTransformTimerCallback2");
    is_origin_transform_update_pending_ = false;
    if (!origin_transform_param_client_.setConfiguration(origin_transform_pending_config_))
    {
        return;
    }

    dynamic_reconfigure::Reconfigure srv;
    origin_transform_pending_config_.__toMessage__(srv.response.config);
    origin_transform_latest_config_.__fromMessage__(srv.response.config);
    is_origin_transform_received = true;
}

void UgiBaseStationPluginDynamicParam::mjpegCamTimerCallback(const ros::TimerEvent &)
{   
    ROS_INFO("mjpegCamTimerCallback");
    boost::mutex::scoped_lock lock(mjpeg_cam_param_mutex_);
    if (!is_mjpeg_cam_update_pending_)
    {
        return;
    }

    ROS_INFO("mjpegCamTimerCallback2");
    is_mjpeg_cam_update_pending_ = false;
    if (!mjpeg_cam_param_client_.setConfiguration(mjpeg_cam_pending_config_))
    {
        return;
    }

    dynamic_reconfigure::Reconfigure srv;
    mjpeg_cam_pending_config_.__toMessage__(srv.response.config);
    mjpeg_cam_latest_config_.__fromMessage__(srv.response.config);
    is_origin_transform_received = true;
}

void UgiBaseStationPluginDynamicParam::moveBaseTimerCallback(const ros::TimerEvent &)
{
    boost::mutex::scoped_lock lock(local_planner_param_mutex_);
    if (!is_local_planner_update_pending_)
    {
        return;
    }
    is_local_planner_update_pending_ = false;
    if (!local_planner_param_client_.setConfiguration(local_planner_pending_config_))
    {
        return;
    }

    dynamic_reconfigure::Reconfigure srv;
    local_planner_pending_config_.__toMessage__(srv.response.config);
    local_planner_latest_config_.__fromMessage__(srv.response.config);
    is_origin_transform_received = true;
}
}