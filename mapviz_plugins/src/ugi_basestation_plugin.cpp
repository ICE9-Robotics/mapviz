// *****************************************************************************
//
// Copyright (c) 2015, Southwest Research Institute® (SwRI®)
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of Southwest Research Institute® (SwRI®) nor the
//       names of its contributors may be used to endorse or promote products
//       derived from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL Southwest Research Institute® BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
// OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
// DAMAGE.
//
// *****************************************************************************

#include <mapviz_plugins/ugi_basestation_plugin.h>

// QT libraries
#include <QDateTime>

// ROS libraries
#include <geometry_msgs/PolygonStamped.h>
#include <std_srvs/Trigger.h>
#include <rtkgps_odom_matcher/SetPose.h>
#include <unitree_legged_msgs/SetUnitreeHLMode.h>

// Declare plugin
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mapviz_plugins::UgiBaseStationPlugin, mapviz::MapvizPlugin)

namespace stu = swri_transform_util;

namespace mapviz_plugins
{

  UgiBaseStationPlugin::UgiBaseStationPlugin() : config_widget_(new QWidget()),
                                                 move_base_client_("move_base", true),
                                                 monitoring_action_state_(false),
                                                 map_canvas_(NULL),
                                                 is_mouse_down_(false),
                                                 selected_waypoint_(-1),
                                                 max_ms_(Q_INT64_C(500)),
                                                 max_distance_(2.0),
                                                 waypoint_frame_("map")
  {
    ui_.setupUi(config_widget_);

    // Set background white
    QPalette p(config_widget_->palette());
    p.setColor(QPalette::Background, Qt::white);
    config_widget_->setPalette(p);

    ui_.colorButtonWpColor->setColor(Qt::green);

    QObject::connect(ui_.pushButtonNavAbort, SIGNAL(clicked()), this, SLOT(on_pushButtonNavAbort_clicked()));
    QObject::connect(ui_.pushButtonNavSetGoal, SIGNAL(toggled(bool)), this, SLOT(on_pushButtonNavSetGoal_toggled(bool)));

    QObject::connect(ui_.pushButtonMatcherFlip, SIGNAL(clicked()), this, SLOT(on_pushButtonMatcherFlip_clicked()));
    QObject::connect(ui_.pushButtonMatcherStart, SIGNAL(toggled(bool)), this, SLOT(on_pushButtonMatcherStart_toggled(bool)));
    QObject::connect(ui_.pushButtonMatcherReset, SIGNAL(clicked()), this, SLOT(on_pushButtonMatcherReset_clicked()));

    QObject::connect(ui_.pushButtonPatrolDrawWp, SIGNAL(toggled(bool)), this, SLOT(on_pushButtonPatrolDrawWp_toggled(bool)));
    QObject::connect(ui_.pushButtonPatrolSendWp, SIGNAL(clicked()), this, SLOT(on_pushButtonPatrolSendWp_clicked()));
    QObject::connect(ui_.pushButtonPatrolStart, SIGNAL(toggled(bool)), this, SLOT(on_pushButtonPatrolStart_toggled(bool)));
    QObject::connect(ui_.pushButtonPatrolAbortClear, SIGNAL(clicked()), this, SLOT(on_pushButtonPatrolAbortClear_clicked()));

    QObject::connect(ui_.pushButtonModeIdle, SIGNAL(clicked()), this, SLOT(on_pushButtonModeIdle_clicked()));
    QObject::connect(ui_.pushButtonModeUp, SIGNAL(clicked()), this, SLOT(on_pushButtonModeUp_clicked()));
    QObject::connect(ui_.pushButtonModeDown, SIGNAL(clicked()), this, SLOT(on_pushButtonModeDown_clicked()));
    QObject::connect(ui_.pushButtonModeDamp, SIGNAL(clicked()), this, SLOT(on_pushButtonModeDamp_clicked()));
    QObject::connect(ui_.pushButtonModeRecover, SIGNAL(clicked()), this, SLOT(on_pushButtonModeRecover_clicked()));
    QObject::connect(ui_.pushButtonModeLock, SIGNAL(toggled(bool)), this, SLOT(on_pushButtonModeLock_toggled(bool)));

    QObject::connect(ui_.pushButtonSettingRevert, SIGNAL(clicked()), this, SLOT(on_pushButtonSettingRevert_clicked()));
    QObject::connect(ui_.pushButtonSettingRestore, SIGNAL(clicked()), this, SLOT(on_pushButtonSettingRestore_clicked()));
    QObject::connect(ui_.pushButtonSettingApply, SIGNAL(clicked()), this, SLOT(on_pushButtonSettingApply_clicked()));


    matcher_start_srv_client_ = nh_.serviceClient<std_srvs::Trigger>("start_match");
    matcher_stop_srv_client_ = nh_.serviceClient<std_srvs::Trigger>("stop_match");
    matcher_flip_srv_client_ = nh_.serviceClient<std_srvs::Trigger>("flip_match");
    matcher_reset_srv_client_ = nh_.serviceClient<std_srvs::Trigger>("reset_match");
    path_ready_srv_client_ = nh_.serviceClient<std_srvs::Trigger>("path_ready");
    path_stop_srv_client_ = nh_.serviceClient<std_srvs::Trigger>("path_stop");
    path_reset_srv_client_ = nh_.serviceClient<std_srvs::Trigger>("path_reset");
    uhl_mode_srv_client_ = nh_.serviceClient<unitree_legged_msgs::SetUnitreeHLMode>("set_unitree_high_level_mode");

    polygon_pub_ = nh_.advertise<geometry_msgs::PolygonStamped>("polygon", 1, true);
    waypoints_sub_ = nh_.subscribe("waypoints", 5, &UgiBaseStationPlugin::waypointsCallback, this);
    diagnostics_sub_ = nh_.subscribe("unitree/diagnostics", 5, &UgiBaseStationPlugin::diagnosticsCallback, this);

    slow_timer_ = nh_.createTimer(ros::Duration(1.0), &UgiBaseStationPlugin::slowTimerCallback, this);
    fast_timer_ = nh_.createTimer(ros::Duration(0.1), &UgiBaseStationPlugin::fastTimerCallback, this);

    on_pushButtonSettingRevert_clicked();
  }

  UgiBaseStationPlugin::~UgiBaseStationPlugin()
  {
    if (map_canvas_)
    {
      map_canvas_->removeEventFilter(this);
    }
  }

  bool UgiBaseStationPlugin::Initialize(QGLWidget *canvas)
  {
    map_canvas_ = static_cast<mapviz::MapCanvas *>(canvas);
    map_canvas_->installEventFilter(this);
    initialized_ = true;
    return true;
  }

  QWidget *UgiBaseStationPlugin::GetConfigWidget(QWidget *parent)
  {
    config_widget_->setParent(parent);
    return config_widget_;
  }

  void UgiBaseStationPlugin::Draw(double x, double y, double scale)
  {
    if (ui_.pushButtonNavSetGoal->isChecked())
    {
      DrawNavGoal(x, y, scale);
      return;
    }
    DrawWaypoints(x, y, scale);
  }

  void UgiBaseStationPlugin::DrawNavGoal(double x, double y, double scale)
  {
    std::array<QPointF, 7> arrow_points;
    arrow_points[0] = QPointF(10, 0);
    arrow_points[1] = QPointF(6, -2.5);
    arrow_points[2] = QPointF(6.5, -1);
    arrow_points[3] = QPointF(0, -1);
    arrow_points[4] = QPointF(0, 1);
    arrow_points[5] = QPointF(6.5, 1);
    arrow_points[6] = QPointF(6, 2.5);

    if (is_mouse_down_)
    {
      QPointF transformed_points[7];
      for (size_t i = 0; i < 7; i++)
      {
        tf::Vector3 point(arrow_points[i].x(), arrow_points[i].y(), 0);
        point *= scale * 10;
        point = tf::Transform(tf::createQuaternionFromYaw(arrow_angle_)) * point;
        transformed_points[i] = QPointF(point.x() + arrow_tail_position_.x(),
                                        point.y() + arrow_tail_position_.y());
      }
      glColor3f(0.1, 0.9, 0.1);
      glLineWidth(2);
      glBegin(GL_TRIANGLE_FAN);
      for (const QPointF &point : transformed_points)
      {
        glVertex2d(point.x(), point.y());
      }
      glEnd();

      glColor3f(0.0, 0.6, 0.0);
      glBegin(GL_LINE_LOOP);
      for (const QPointF &point : transformed_points)
      {
        glVertex2d(point.x(), point.y());
      }
      glEnd();
    }
  }

  void UgiBaseStationPlugin::DrawWaypoints(double x, double y, double scale)
  {
    stu::Transform transform;
    if (!tf_manager_->GetTransform(target_frame_, waypoint_frame_, transform))
    {
      PrintWarningHelper(ui_.status_patrol, "Waiting for transform");
      return;
    }

    // Transform polygon
    for (size_t i = 0; i < vertices_.size(); i++)
    {
      transformed_vertices_[i] = transform * vertices_[i];
    }

    glLineWidth(5);
    const QColor color = ui_.colorButtonWpColor->color();
    glColor4d(color.redF(), color.greenF(), color.blueF(), 0.8);
    glBegin(GL_LINE_STRIP);

    for (const auto &vertex : transformed_vertices_)
    {
      glVertex2d(vertex.x(), vertex.y());
    }

    glEnd();

    glBegin(GL_LINES);

    glColor4d(color.redF(), color.greenF(), color.blueF(), 0.4);

    if (transformed_vertices_.size() > 2)
    {
      glVertex2d(transformed_vertices_.front().x(), transformed_vertices_.front().y());
      glVertex2d(transformed_vertices_.back().x(), transformed_vertices_.back().y());
    }

    glEnd();

    // Draw vertices
    glPointSize(20);
    glBegin(GL_POINTS);

    glColor4d(color.redF(), color.greenF(), color.blueF(), 0.8);
    for (const auto &vertex : transformed_vertices_)
    {
      glVertex2d(vertex.x(), vertex.y());
    }
    glEnd();

    if (vertices_.size() == 0)
    {
      PrintInfoHelper(ui_.status_patrol, "OK");
    }
  }

  void UgiBaseStationPlugin::LoadConfig(const YAML::Node &node, const std::string &path)
  {
    if (node["color"])
    {
      std::string color;
      node["color"] >> color;
      ui_.colorButtonWpColor->setColor(QColor(color.c_str()));
    }
  }

  void UgiBaseStationPlugin::SaveConfig(YAML::Emitter &emitter, const std::string &path)
  {
    std::string color = ui_.colorButtonWpColor->color().name().toStdString();
    emitter << YAML::Key << "color" << YAML::Value << color;
  }

  bool UgiBaseStationPlugin::eventFilter(QObject *object, QEvent *event)
  {
    switch (event->type())
    {
    case QEvent::MouseButtonPress:
      return handleMousePress(static_cast<QMouseEvent *>(event));
    case QEvent::MouseButtonRelease:
      return handleMouseRelease(static_cast<QMouseEvent *>(event));
    case QEvent::MouseMove:
      return handleMouseMove(static_cast<QMouseEvent *>(event));
    default:
      return false;
    }
  }

  bool UgiBaseStationPlugin::handleMousePress(QMouseEvent *event)
  {
    if (ui_.pushButtonNavSetGoal->isChecked())
    {
      return handleMousePressNavGoal(event);
    }
    if (ui_.pushButtonPatrolDrawWp->isChecked())
    {
      return handleMousePressWaypoints(event);
    }
  }

  bool UgiBaseStationPlugin::handleMousePressNavGoal(QMouseEvent *event)
  {

    if (event->button() == Qt::LeftButton)
    {
      is_mouse_down_ = true;
      arrow_angle_ = 0;
#if QT_VERSION >= 0x050000
      arrow_tail_position_ = map_canvas_->MapGlCoordToFixedFrame(event->localPos());
#else
      arrow_tail_position_ = map_canvas_->MapGlCoordToFixedFrame(event->posF());
#endif
      return true;
    }
    return false;
  }

  bool UgiBaseStationPlugin::handleMousePressWaypoints(QMouseEvent *event)
  {
    selected_waypoint_ = -1;
    int closest_point = 0;
    double closest_distance = std::numeric_limits<double>::max();

#if QT_VERSION >= 0x050000
    QPointF point = event->localPos();
#else
    QPointF point = event->posF();
#endif
    stu::Transform transform;
    if (tf_manager_->GetTransform(target_frame_, waypoint_frame_, transform))
    {
      for (size_t i = 0; i < vertices_.size(); i++)
      {
        tf::Vector3 vertex = vertices_[i];
        vertex = transform * vertex;

        QPointF transformed = map_canvas_->FixedFrameToMapGlCoord(QPointF(vertex.x(), vertex.y()));

        double distance = QLineF(transformed, point).length();

        if (distance < closest_distance)
        {
          closest_distance = distance;
          closest_point = static_cast<int>(i);
        }
      }
    }

    if (event->button() == Qt::LeftButton)
    {
      if (closest_distance < 15)
      {
        selected_waypoint_ = closest_point;
        return true;
      }
      else
      {
        is_mouse_down_ = true;
#if QT_VERSION >= 0x050000
        mouse_down_pos_ = event->localPos();
#else
        mouse_down_pos_ = event->posF();
#endif
        mouse_down_time_ = QDateTime::currentMSecsSinceEpoch();
        return false;
      }
    }
    else if (event->button() == Qt::RightButton)
    {
      if (closest_distance < 15)
      {
        vertices_.erase(vertices_.begin() + closest_point);
        transformed_vertices_.resize(vertices_.size());
        return true;
      }
    }

    return false;
  }

  bool UgiBaseStationPlugin::handleMouseMove(QMouseEvent *event)
  {
    if (ui_.pushButtonNavSetGoal->isChecked())
    {
      return handleMouseMoveNavGoal(event);
    }
    if (ui_.pushButtonPatrolDrawWp->isChecked())
    {
      return handleMouseMoveWaypoints(event);
    }
  }

  bool UgiBaseStationPlugin::handleMouseMoveNavGoal(QMouseEvent *event)
  {
    if (is_mouse_down_)
    {
#if QT_VERSION >= 0x050000
      QPointF head_pos = map_canvas_->MapGlCoordToFixedFrame(event->localPos());
#else
      QPointF head_pos = map_canvas_->MapGlCoordToFixedFrame(event->posF());
#endif
      arrow_angle_ = atan2(head_pos.y() - arrow_tail_position_.y(),
                           head_pos.x() - arrow_tail_position_.x());
    }
    return false;
  }

  bool UgiBaseStationPlugin::handleMouseMoveWaypoints(QMouseEvent *event)
  {
    if (selected_waypoint_ >= 0 && static_cast<size_t>(selected_waypoint_) < vertices_.size())
    {
#if QT_VERSION >= 0x050000
      QPointF point = event->localPos();
#else
      QPointF point = event->posF();
#endif
      stu::Transform transform;
      if (tf_manager_->GetTransform(waypoint_frame_, target_frame_, transform))
      {
        QPointF transformed = map_canvas_->MapGlCoordToFixedFrame(point);
        tf::Vector3 position(transformed.x(), transformed.y(), 0.0);
        position = transform * position;
        vertices_[selected_waypoint_].setY(position.y());
        vertices_[selected_waypoint_].setX(position.x());
      }

      return true;
    }
    return false;
  }

  bool UgiBaseStationPlugin::handleMouseRelease(QMouseEvent *event)
  {
    if (ui_.pushButtonNavSetGoal->isChecked())
    {
      return handleMouseReleaseNavGoal(event);
    }
    if (ui_.pushButtonPatrolDrawWp->isChecked())
    {
      return handleMouseReleaseWaypoints(event);
    }
  }

  bool UgiBaseStationPlugin::handleMouseReleaseNavGoal(QMouseEvent *event)
  {
    if (!is_mouse_down_)
    {
      return false;
    }

    is_mouse_down_ = false;

    bool set_nav_goal_checked = ui_.pushButtonNavSetGoal->isChecked();
    if (!set_nav_goal_checked)
    {
      return false;
    }

    tf::Quaternion quat = tf::createQuaternionFromYaw(arrow_angle_);

    if (set_nav_goal_checked)
    {

      move_base_msg_.action_goal.header.frame_id = target_frame_;
      move_base_msg_.action_goal.header.stamp = ros::Time::now();
      move_base_msg_.action_goal.goal_id.stamp = move_base_msg_.action_goal.header.stamp;
      move_base_msg_.action_goal.goal_id.id = "mapviz_goal";
      move_base_msg_.action_goal.goal.target_pose.header = move_base_msg_.action_goal.header;

      geometry_msgs::Pose &pose = move_base_msg_.action_goal.goal.target_pose.pose;
      pose.position.x = arrow_tail_position_.x();
      pose.position.y = arrow_tail_position_.y();
      pose.position.z = 0.0;
      tf::quaternionTFToMsg(quat, pose.orientation);

      move_base_client_.sendGoal(move_base_msg_.action_goal.goal);
      ui_.pushButtonNavSetGoal->setChecked(false);
      monitoring_action_state_ = true;
    }
    return true;
  }

  bool UgiBaseStationPlugin::handleMouseReleaseWaypoints(QMouseEvent *event)
  {
    if (selected_waypoint_ >= 0 && static_cast<size_t>(selected_waypoint_) < vertices_.size())
    {
#if QT_VERSION >= 0x050000
      QPointF point = event->localPos();
#else
      QPointF point = event->posF();
#endif
      stu::Transform transform;
      if (tf_manager_->GetTransform(waypoint_frame_, target_frame_, transform))
      {
        QPointF transformed = map_canvas_->MapGlCoordToFixedFrame(point);
        tf::Vector3 position(transformed.x(), transformed.y(), 0.0);
        position = transform * position;
        vertices_[selected_waypoint_].setX(position.x());
        vertices_[selected_waypoint_].setY(position.y());
      }

      selected_waypoint_ = -1;

      PrintWarningHelper(ui_.status_patrol, "Not published");
      is_waypoints_ok_ = false;
      return true;
    }
    else if (is_mouse_down_)
    {
#if QT_VERSION >= 0x050000
      qreal distance = QLineF(mouse_down_pos_, event->localPos()).length();
#else
      qreal distance = QLineF(mouse_down_pos_, event->posF()).length();
#endif
      qint64 msecsDiff = QDateTime::currentMSecsSinceEpoch() - mouse_down_time_;

      // Only fire the event if the mouse has moved less than the maximum distance
      // and was held for shorter than the maximum time..  This prevents click
      // events from being fired if the user is dragging the mouse across the map
      // or just holding the cursor in place.
      if (msecsDiff < max_ms_ && distance <= max_distance_)
      {
#if QT_VERSION >= 0x050000
        QPointF point = event->localPos();
#else
        QPointF point = event->posF();
#endif

        QPointF transformed = map_canvas_->MapGlCoordToFixedFrame(point);
        ROS_INFO("mouse point at %f, %f -> %f, %f", point.x(), point.y(), transformed.x(), transformed.y());

        stu::Transform transform;
        tf::Vector3 position(transformed.x(), transformed.y(), 0.0);

        if (tf_manager_->GetTransform(waypoint_frame_, target_frame_, transform))
        {
          position = transform * position;
          vertices_.push_back(position);
          transformed_vertices_.resize(vertices_.size());
          ROS_INFO("Adding vertex at %lf, %lf %s", position.x(), position.y(), waypoint_frame_.c_str());
        }
      }
    }
    is_mouse_down_ = false;

    PrintWarningHelper(ui_.status_patrol, "Not published");
    is_waypoints_ok_ = false;
    return false;
  }

  void UgiBaseStationPlugin::waypointsCallback(const geometry_msgs::PoseArray::ConstPtr& msg)
  {
    if (vertices_.size() == msg->poses.size())
    {
      is_waypoints_ok_ = true;
      PrintInfoHelper(ui_.status_patrol, "Feedback OK");
    }
    else
    {
      is_waypoints_ok_ = false;
      PrintErrorHelper(ui_.status_patrol, "Waypoints not received correctly");
      ROS_ERROR_STREAM(vertices_.size() << "  " << msg->poses.size());
    }
  }

  void UgiBaseStationPlugin::diagnosticsCallback(const unitree_diagnostics_msgs::Diagnostics::ConstPtr& msg)
  {
    diag_info_.timestamp = msg->header.stamp;
    diag_info_.batterySoC = msg->batterySoC;
    diag_info_.commandVelocity = msg->commandVelocity;
    diag_info_.commandYawSpeed = msg->commandYawSpeed;
    diag_info_.gpsStatusTs = msg->gpsStatusTimestamp;
    diag_info_.gpsStatusDescription = msg->gpsStatusDescription;
    diag_info_.highStateTs = msg->highStateTimestamp;
    diag_info_.velocity = msg->velocity;
    diag_info_.yawSpeed = msg->yawSpeed;
    diag_info_.mode = msg->mode;
  }

  void UgiBaseStationPlugin::slowTimerCallback(const ros::TimerEvent &)
  {
    bool connected = move_base_client_.isServerConnected();
    ui_.pushButtonNavAbort->setEnabled(connected);
    ui_.pushButtonNavSetGoal->setEnabled(connected);
    ui_.pushButtonPatrolStart->setEnabled(connected);

    if (!connected)
    {
      PrintErrorHelper(ui_.status_nav, "[move_base] server not connected");
    }
    else if (!monitoring_action_state_)
    {
      PrintInfoHelper(ui_.status_nav, "Ready to send command");
    }
    else
    {
      actionlib::SimpleClientGoalState state = move_base_client_.getState();
      switch (state.state_)
      {
      case actionlib::SimpleClientGoalState::PENDING:
        PrintWarningHelper(ui_.status_nav, state.toString());
        break;

      case actionlib::SimpleClientGoalState::PREEMPTED:
        PrintWarningHelper(ui_.status_nav, state.toString());
        monitoring_action_state_ = false;
        break;

      case actionlib::SimpleClientGoalState::ACTIVE:
        PrintInfoHelper(ui_.status_nav, state.toString());
        break;

      case actionlib::SimpleClientGoalState::SUCCEEDED:
        PrintInfoHelper(ui_.status_nav, state.toString());
        monitoring_action_state_ = false;
        break;

      case actionlib::SimpleClientGoalState::REJECTED:
      case actionlib::SimpleClientGoalState::ABORTED:
      case actionlib::SimpleClientGoalState::LOST:
      case actionlib::SimpleClientGoalState::RECALLED:
        PrintErrorHelper(ui_.status_nav, state.toString());
        monitoring_action_state_ = false;
        break;
      }
    }
  }

  void UgiBaseStationPlugin::fastTimerCallback(const ros::TimerEvent &)
  {
    if (diag_info_.mode == 5 || diag_info_.mode == 7)
    {
      ui_.pushButtonModeDamp->setEnabled(true);
    } 
    else if (!ui_.pushButtonModeLock->isChecked())
    {
      ui_.pushButtonModeDamp->setEnabled(false);
    }

    switch (diag_info_.mode)
    {
    case 0:
      ui_.status_mode->setText("Idle");
      break;
    case 1:
      ui_.status_mode->setText("Force Stand");
      break;
    case 2:
      ui_.status_mode->setText("Walk");
      break;
    case 3:
      ui_.status_mode->setText("Walk towards Target");
      break;
    case 5:
      ui_.status_mode->setText("Stand Down");
      break;
    case 6:
      ui_.status_mode->setText("Stand Up");
      break;
    case 7:
      ui_.status_mode->setText("Soft Stop");
      break;
    case 8:
      ui_.status_mode->setText("Recovery");
      break;
    case 9:
      ui_.status_mode->setText("Backflip");
      break;
    case 10:
      ui_.status_mode->setText("Yaw Jump");
    case 11:
      ui_.status_mode->setText("Praying");
      break;
    default:
      ui_.status_mode->setText("Unknown");
      break;
    }

    ros::Time now = ros::Time::now();
    double diagTime = (now - diag_info_.timestamp).toSec();
    double highStateTime = (now - diag_info_.highStateTs).toSec();
    double gpsTime = (now - diag_info_.gpsStatusTs).toSec();

    if (diagTime > 10.0)
    {
      ui_.diag_network_status -> setText("No data, last activity " + QString::number(diagTime, 'f', 1) + "s ago");
      ui_.diag_network_status -> setStyleSheet("QLabel { background-color : red; color : white; }");
    }
    else if (diagTime > 1.0)
    {
      ui_.diag_network_status -> setText("Delayed, last activity " + QString::number(diagTime, 'f', 1) + "s delay");
      ui_.diag_network_status -> setStyleSheet("QLabel { background-color : orange; color : white; }");
    }
    else
    {
      ui_.diag_network_status -> setText("OK, last activity " + QString::number(diagTime, 'f', 1) + "s ago");
      ui_.diag_network_status -> setStyleSheet("QLabel { background-color : green; color : white; }");
    }
    if (highStateTime > 10.0)
    {
      ui_.diag_robot_conn -> setText("No data, last activity " + QString::number(highStateTime, 'f', 1) + "s ago");
      ui_.diag_robot_conn -> setStyleSheet("QLabel { background-color : red; color : white; }");
    }
    else if (highStateTime > 1.0)
    {
      ui_.diag_robot_conn -> setText("Delayed, last activity " + QString::number(highStateTime, 'f', 1) + "s ago");
      ui_.diag_robot_conn -> setStyleSheet("QLabel { background-color : orange; color : white; }");
    }
    else
    {
      ui_.diag_robot_conn -> setText("OK, last activity " + QString::number(highStateTime, 'f', 1) + "s ago");
      ui_.diag_robot_conn -> setStyleSheet("QLabel { background-color : green; color : white; }");
    }
    if (gpsTime > 10.0)
    {
      ui_.diag_gps_conn -> setText("No data, last activity " + QString::number(gpsTime, 'f', 1) + "s ago");
      ui_.diag_gps_conn -> setStyleSheet("QLabel { background-color : red; color : white; }");
    }
    else if (gpsTime > 2.0)
    {
      ui_.diag_gps_conn -> setText("Delayed, last activity " + QString::number(gpsTime, 'f', 1) + "s ago");
      ui_.diag_gps_conn -> setStyleSheet("QLabel { background-color : orange; color : white; }");
    }
    else
    {
      ui_.diag_gps_conn -> setText("OK, last activity " + QString::number(gpsTime, 'f', 1) + "s ago");
      ui_.diag_gps_conn -> setStyleSheet("QLabel { background-color : green; color : white; }");
    }
    
    ui_.diag_vel -> setText(QString::number(diag_info_.velocity, 'f', 3) + " | " 
                                + QString::number(diag_info_.commandVelocity, 'f', 3));
    ui_.diag_yaw -> setText(QString::number(diag_info_.yawSpeed * 180/3.14159, 'f', 1) + " | " 
                                + QString::number(diag_info_.commandYawSpeed * 180/3.14159, 'f', 1));
    ui_.diag_battery -> setText(QString::number(diag_info_.batterySoC, 'f', 1));
    if (diag_info_.batterySoC < 20)
    {
      ui_.diag_battery -> setStyleSheet("QLabel { background-color : red; color : white; }");
    }
    else if (diag_info_.batterySoC < 50)
    {
      ui_.diag_battery -> setStyleSheet("QLabel { background-color : orange; color : white; }");
    }
    else
    {
      ui_.diag_battery -> setStyleSheet("QLabel { background-color : green; color : white; }");
    }
    ui_.diag_gps_quality -> setText(QString::fromStdString(diag_info_.gpsStatusDescription));
    if (diag_info_.gpsStatusDescription == "No fix")
    {
      ui_.diag_gps_quality -> setStyleSheet("QLabel { background-color : red; color : white; }");
    }
    else if (diag_info_.gpsStatusDescription == "RTK fixed" || diag_info_.gpsStatusDescription == "RTK float" || diag_info_.gpsStatusDescription == "RTK fixed/float") 
    {
      ui_.diag_gps_quality -> setStyleSheet("QLabel { background-color : green; color : white; }");
    }
    else
    {
      ui_.diag_gps_quality -> setStyleSheet("QLabel { background-color : orange; color : white; }");
    }
  }

  void UgiBaseStationPlugin::on_pushButtonNavSetGoal_toggled(bool checked)
  {
    bool patrolDrawWp_checked = ui_.pushButtonPatrolDrawWp->isChecked();
    if (checked)
    {
      if (patrolDrawWp_checked)
      {
        ui_.pushButtonPatrolDrawWp->setChecked(false);
      }
      QPixmap cursor_pixmap = QPixmap(":/images/green_arrow_cursor.png");
      QApplication::setOverrideCursor(QCursor(cursor_pixmap));
    }
    if (!checked)
    {
      QApplication::restoreOverrideCursor();
    }
  }

  void UgiBaseStationPlugin::on_pushButtonNavAbort_clicked()
  {
    move_base_client_.cancelGoal();
  }

  void UgiBaseStationPlugin::on_pushButtonMatcherFlip_clicked()
  {
    std_srvs::Trigger trigger;
    QApplication::setOverrideCursor(Qt::WaitCursor);
    if (matcher_flip_srv_client_.call(trigger))
    {
      PrintInfoHelper(ui_.status_matcher, "Flip match successful");
    }
    else
    {
      PrintErrorHelper(ui_.status_matcher, "Failed to flip match");
    }
    QApplication::restoreOverrideCursor();
  }

  void UgiBaseStationPlugin::on_pushButtonMatcherStart_toggled(bool checked)
  {
    std_srvs::Trigger trigger;
    QApplication::setOverrideCursor(Qt::WaitCursor);
    if (checked)
    {
      if (matcher_start_srv_client_.call(trigger))
      {
        PrintInfoHelper(ui_.status_matcher, "Started");
      }
      else
      {
        PrintErrorHelper(ui_.status_matcher, "Failed to start");
      }
    }
    else
    {
      if (matcher_stop_srv_client_.call(trigger))
      {
        PrintInfoHelper(ui_.status_matcher, "Stopped");
      }
      else
      {
        PrintErrorHelper(ui_.status_matcher, "Failed to stop");
      }
    }
    QApplication::restoreOverrideCursor();
  }

  void UgiBaseStationPlugin::on_pushButtonMatcherReset_clicked()
  {
    std_srvs::Trigger trigger;
    QApplication::setOverrideCursor(Qt::WaitCursor);
    if (matcher_reset_srv_client_.call(trigger))
    {
      PrintInfoHelper(ui_.status_matcher, "Reset successful");
    }
    else
    {
      PrintErrorHelper(ui_.status_matcher, "Failed to reset");
    }
    QApplication::restoreOverrideCursor();
  }

  void UgiBaseStationPlugin::on_pushButtonPatrolDrawWp_toggled(bool checked)
  {
    bool navSetGoal_checked = ui_.pushButtonNavSetGoal->isChecked();
    if (checked)
    {
      if (navSetGoal_checked)
      {
        ui_.pushButtonNavSetGoal->setChecked(false);
      }
      QPixmap cursor_pixmap = QPixmap(":/images/location_icon_cursor.png");
      QApplication::restoreOverrideCursor();
      QApplication::setOverrideCursor(QCursor(cursor_pixmap));
    }
    if (!checked)
    {
      QApplication::restoreOverrideCursor();
    }
  }

  void UgiBaseStationPlugin::on_pushButtonPatrolSendWp_clicked()
  {
    geometry_msgs::PolygonStamped polygon;
    polygon.header.stamp = ros::Time::now();
    polygon.header.frame_id = waypoint_frame_;

    for (const auto &vertex : vertices_)
    {
      geometry_msgs::Point32 point;
      point.x = vertex.x();
      point.y = vertex.y();
      point.z = 0;
      polygon.polygon.points.push_back(point);
    }

    polygon_pub_.publish(polygon);

    PrintWarningHelper(ui_.status_patrol, "Published, waiting for feedback");
  }

  void UgiBaseStationPlugin::on_pushButtonPatrolStart_toggled(bool checked)
  {
    QApplication::setOverrideCursor(Qt::WaitCursor);
    if (checked)
    {
      if (!is_waypoints_ok_)
      {
        ui_.pushButtonPatrolStart->setChecked(false);
        return;
      }
      std_srvs::Trigger trigger;
      if (path_ready_srv_client_.call(trigger))
      {
        PrintInfoHelper(ui_.status_patrol, "Started");
      }
      else
      {
        PrintErrorHelper(ui_.status_patrol, "Failed to start");
      }
    }
    else
    {
      std_srvs::Trigger trigger;
      if (path_stop_srv_client_.call(trigger))
      {
        PrintInfoHelper(ui_.status_patrol, "Stopped");
      }
      else
      {
        PrintErrorHelper(ui_.status_patrol, "Failed to stop");
      }
    }
    QApplication::restoreOverrideCursor();
  }

  void UgiBaseStationPlugin::on_pushButtonPatrolAbortClear_clicked()
  {
    if (ui_.pushButtonPatrolStart->isChecked())
    {
      ui_.pushButtonPatrolStart->setChecked(false);
    }

    vertices_.clear();
    transformed_vertices_.clear();

    std_srvs::Trigger trigger;
    QApplication::setOverrideCursor(Qt::WaitCursor);
    if (path_reset_srv_client_.call(trigger))
    {
      PrintInfoHelper(ui_.status_patrol, "Reset successful");
    }
    else
    {
      PrintErrorHelper(ui_.status_patrol, "Failed to reset");
    }
    QApplication::restoreOverrideCursor();
  }

  void UgiBaseStationPlugin::on_pushButtonModeIdle_clicked()
  {
    unitree_legged_msgs::SetUnitreeHLMode mode;
    mode.request.mode = 0;
    QApplication::setOverrideCursor(Qt::WaitCursor);
    uhl_mode_srv_client_.call(mode);
    QApplication::restoreOverrideCursor();
  }

  void UgiBaseStationPlugin::on_pushButtonModeUp_clicked()
  {
    unitree_legged_msgs::SetUnitreeHLMode mode;
    mode.request.mode = 6;
    QApplication::setOverrideCursor(Qt::WaitCursor);
    uhl_mode_srv_client_.call(mode);
    QApplication::restoreOverrideCursor();
  }

  void UgiBaseStationPlugin::on_pushButtonModeDown_clicked()
  {
    unitree_legged_msgs::SetUnitreeHLMode mode;
    mode.request.mode = 5;
    QApplication::setOverrideCursor(Qt::WaitCursor);
    uhl_mode_srv_client_.call(mode);
    QApplication::restoreOverrideCursor();
  }

  void UgiBaseStationPlugin::on_pushButtonModeDamp_clicked()
  {
    unitree_legged_msgs::SetUnitreeHLMode mode;
    mode.request.mode = 7;
    QApplication::setOverrideCursor(Qt::WaitCursor);
    uhl_mode_srv_client_.call(mode);
    QApplication::restoreOverrideCursor();
  }

  void UgiBaseStationPlugin::on_pushButtonModeRecover_clicked()
  {
    unitree_legged_msgs::SetUnitreeHLMode mode;
    mode.request.mode = 8;
    QApplication::setOverrideCursor(Qt::WaitCursor);
    uhl_mode_srv_client_.call(mode);
    QApplication::restoreOverrideCursor();
  }

  void UgiBaseStationPlugin::on_pushButtonModeLock_toggled(bool checked)
  {
    if (checked)
    {
      ui_.pushButtonModeDamp->setEnabled(true);
    }
    else if (diag_info_.mode != 5)
    {
      ui_.pushButtonModeDamp->setEnabled(false);
    }
  }

  void UgiBaseStationPlugin::on_pushButtonSettingRestore_clicked()
  {
    ui_.doubleSpinBoxFwVel->setValue(0.3);
    ui_.doubleSpinBoxBwVel->setValue(0.12);
    ui_.doubleSpinBoxYawSpd->setValue(0.3);
    ui_.doubleSpinBoxLinearAcc->setValue(0.4);
    ui_.doubleSpinBoxYawAcc->setValue(0.3);
    ui_.doubleSpinBoxCamExp->setValue(140);
    ui_.doubleSpinBoxCamBright->setValue(70);
    ui_.checkBoxCamAutoExp->setChecked(true);
  }

  void UgiBaseStationPlugin::on_pushButtonSettingRevert_clicked()
  {
    int result_flag = 0;
    dynamic_reconfigure::ReconfigureRequest srv_req;
    dynamic_reconfigure::ReconfigureResponse srv_resp;

    QApplication::setOverrideCursor(Qt::WaitCursor);
    if (ros::service::call("/slam_planner_node/DBLocalPlannerROS/set_parameters", srv_req, srv_resp))
    {
      result_flag += 1;
    }
    QApplication::restoreOverrideCursor();

    for (const auto &param : srv_resp.config.doubles)
    {
      if (param.name == "max_vel_x")
      {
        ui_.doubleSpinBoxFwVel->setValue(param.value);
      }
      else if (param.name == "max_vel_x_backwards")
      {
        ui_.doubleSpinBoxBwVel->setValue(param.value);
      }
      else if (param.name == "max_vel_theta")
      {
        ui_.doubleSpinBoxYawSpd->setValue(param.value);
      }
      else if (param.name == "acc_lim_x")
      {
        ui_.doubleSpinBoxLinearAcc->setValue(param.value);
      }
      else if (param.name == "acc_lim_theta")
      {
        ui_.doubleSpinBoxYawAcc->setValue(param.value);
      }
    }
    QApplication::setOverrideCursor(Qt::WaitCursor);
    if (ros::service::call("/mjpeg_cam/set_parameters", srv_req, srv_resp))
    {
      result_flag += 2;
    }
    QApplication::restoreOverrideCursor();

    for (const auto &param : srv_resp.config.ints)
    {
      if (param.name == "exposure")
      {
        ui_.doubleSpinBoxCamExp->setValue(param.value);
      }
      else if (param.name == "brightness")
      {
        ui_.doubleSpinBoxCamBright->setValue(param.value);
      }
    }
    ui_.checkBoxCamAutoExp->setChecked(srv_resp.config.bools[0].value);

    if (result_flag != 3)
    {
      PrintErrorHelper(ui_.status_settings, "Failed to contact server");
    }
    else
    {
      PrintInfoHelper(ui_.status_settings, "OK");
    }
  }

  void UgiBaseStationPlugin::on_pushButtonSettingApply_clicked()
  {
    int result_flag = 0;
    dynamic_reconfigure::ReconfigureRequest srv_req;
    dynamic_reconfigure::ReconfigureResponse srv_resp;
    dynamic_reconfigure::DoubleParameter double_param;
    dynamic_reconfigure::IntParameter int_param;
    dynamic_reconfigure::BoolParameter bool_param;
    dynamic_reconfigure::Config conf;

    double_param.name = "max_vel_x";
    double_param.value = ui_.doubleSpinBoxFwVel->value();
    conf.doubles.push_back(double_param);

    double_param.name = "max_vel_x_backwards";
    double_param.value = ui_.doubleSpinBoxBwVel->value();
    conf.doubles.push_back(double_param);

    double_param.name = "max_vel_theta";
    double_param.value = ui_.doubleSpinBoxYawSpd->value();
    conf.doubles.push_back(double_param);

    double_param.name = "acc_lim_x";
    double_param.value = ui_.doubleSpinBoxLinearAcc->value();
    conf.doubles.push_back(double_param);

    double_param.name = "acc_lim_theta";
    double_param.value = ui_.doubleSpinBoxYawAcc->value();
    conf.doubles.push_back(double_param);
    
    srv_req.config = conf;

    QApplication::setOverrideCursor(Qt::WaitCursor);
    if (ros::service::call("/slam_planner_node/DBLocalPlannerROS/set_parameters", srv_req, srv_resp))
    {
      result_flag += 1;
    }
    QApplication::restoreOverrideCursor();

    conf.doubles.clear();

    int_param.name = "exposure";
    int_param.value = ui_.doubleSpinBoxCamExp->value();
    conf.ints.push_back(int_param);

    int_param.name = "brightness";
    int_param.value = ui_.doubleSpinBoxCamBright->value();
    conf.ints.push_back(int_param);

    bool_param.name = "auto_exposure";
    bool_param.value = ui_.checkBoxCamAutoExp->isChecked();
    conf.bools.push_back(bool_param);
    
    srv_req.config = conf;

    QApplication::setOverrideCursor(Qt::WaitCursor);
    if (ros::service::call("/mjpeg_cam/set_parameters", srv_req, srv_resp))
    {
      result_flag += 2;
    }
    QApplication::restoreOverrideCursor();

    if (result_flag != 3)
    {
      PrintErrorHelper(ui_.status_settings, "Failed to contact servier");
    }
    else
    {
      PrintInfoHelper(ui_.status_settings, "OK");
    }
  }
}
