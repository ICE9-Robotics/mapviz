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
    QObject::connect(ui_.pushButtonMatcherFlip, SIGNAL(clicked()), this, SLOT(on_pushButtonFlip_clicked()));
    QObject::connect(ui_.pushButtonMatcherStart, SIGNAL(toggled(bool)), this, SLOT(on_pushButtonStartStop_toggled(bool)));
    QObject::connect(ui_.pushButtonPatrolDrawWp, SIGNAL(toggled(bool)), this, SLOT(on_pushButtonPatrolDrawWp_toggled(bool)));
    QObject::connect(ui_.pushButtonPatrolSendWp, SIGNAL(clicked()), this, SLOT(on_pushButtonPatrolSendWp_clicked()));
    QObject::connect(ui_.pushButtonPatrolStart, SIGNAL(toggled(bool)), this, SLOT(on_pushButtonPatrolStart_toggled(bool)));
    QObject::connect(ui_.pushButtonPatrolAbortClear, SIGNAL(clicked()), this, SLOT(on_pushButtonPatrolAbortClear_clicked()));

    matcher_start_srv_client_ = nh_.serviceClient<std_srvs::Trigger>("start_match");
    matcher_stop_srv_client_ = nh_.serviceClient<std_srvs::Trigger>("stop_match");
    matcher_flip_srv_client_ = nh_.serviceClient<std_srvs::Trigger>("flip_match");
    path_ready_srv_client_ = nh_.serviceClient<std_srvs::Trigger>("path_ready");
    path_stop_srv_client_ = nh_.serviceClient<std_srvs::Trigger>("path_stop");
    path_reset_srv_client_ = nh_.serviceClient<std_srvs::Trigger>("path_reset");

    polygon_pub_ = nh_.advertise<geometry_msgs::PolygonStamped>("polygon", 1, true);
    waypoints_sub_ = nh_.subscribe("waypoints", 5, &UgiBaseStationPlugin::waypointsCallback, this);
    diagnostics_sub_ = nh_.subscribe("unitree/diagnostics", 5, &UgiBaseStationPlugin::diagnosticsCallback, this);

    slow_timer_ = nh_.createTimer(ros::Duration(1.0), &UgiBaseStationPlugin::slowTimerCallback, this);
    fast_timer_ = nh_.createTimer(ros::Duration(0.1), &UgiBaseStationPlugin::fastTimerCallback, this);
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

    glLineWidth(1);
    const QColor color = ui_.colorButtonWpColor->color();
    glColor4d(color.redF(), color.greenF(), color.blueF(), 1.0);
    glBegin(GL_LINE_STRIP);

    for (const auto &vertex : transformed_vertices_)
    {
      glVertex2d(vertex.x(), vertex.y());
    }

    glEnd();

    glBegin(GL_LINES);

    glColor4d(color.redF(), color.greenF(), color.blueF(), 0.25);

    if (transformed_vertices_.size() > 2)
    {
      glVertex2d(transformed_vertices_.front().x(), transformed_vertices_.front().y());
      glVertex2d(transformed_vertices_.back().x(), transformed_vertices_.back().y());
    }

    glEnd();

    // Draw vertices
    glPointSize(9);
    glBegin(GL_POINTS);

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
      PrintInfo("Feedback OK");
    }
    else
    {
      is_waypoints_ok_ = false;
      PrintError("Waypoints not received correctly");
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
    ros::Time now = ros::Time::now();
    double TIME_NO_NETWORK = 10.0;
    double TIME_LONG_DELAY = 1.0;
    double diagTime = (now - diag_info_.timestamp).toSec();
    double highStateTime = (now - diag_info_.highStateTs).toSec();
    double gpsTime = (now - diag_info_.gpsStatusTs).toSec();

    if (diagTime > TIME_NO_NETWORK)
    {
      ui_.diag_network_status -> setText("No data, last activity " + QString::number(diagTime, 'f', 1) + "s ago");
      ui_.diag_network_status -> setStyleSheet("QLabel { background-color : red; color : white; }");
    }
    else if (diagTime > TIME_LONG_DELAY)
    {
      ui_.diag_network_status -> setText("Delayed, last activity " + QString::number(diagTime, 'f', 1) + "s delay");
      ui_.diag_network_status -> setStyleSheet("QLabel { background-color : orange; color : white; }");
    }
    else
    {
      ui_.diag_network_status -> setText("OK, last activity " + QString::number(diagTime, 'f', 1) + "s ago");
      ui_.diag_network_status -> setStyleSheet("QLabel { background-color : green; color : white; }");
    }
    if (highStateTime > TIME_NO_NETWORK)
    {
      ui_.diag_robot_conn -> setText("No data, last activity " + QString::number(highStateTime, 'f', 1) + "s ago");
      ui_.diag_robot_conn -> setStyleSheet("QLabel { background-color : red; color : white; }");
    }
    else if (highStateTime > TIME_LONG_DELAY)
    {
      ui_.diag_robot_conn -> setText("Delayed, last activity " + QString::number(highStateTime, 'f', 1) + "s ago");
      ui_.diag_robot_conn -> setStyleSheet("QLabel { background-color : orange; color : white; }");
    }
    else
    {
      ui_.diag_robot_conn -> setText("OK, last activity " + QString::number(highStateTime, 'f', 1) + "s ago");
      ui_.diag_robot_conn -> setStyleSheet("QLabel { background-color : green; color : white; }");
    }
    if (gpsTime > TIME_NO_NETWORK)
    {
      ui_.diag_gps_conn -> setText("No data, last activity " + QString::number(gpsTime, 'f', 1) + "s ago");
      ui_.diag_gps_conn -> setStyleSheet("QLabel { background-color : red; color : white; }");
    }
    else if (gpsTime > TIME_LONG_DELAY)
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
    if (!checked && !patrolDrawWp_checked)
    {
      QApplication::restoreOverrideCursor();
      QApplication::setOverrideCursor(Qt::ArrowCursor);
    }
  }

  void UgiBaseStationPlugin::on_pushButtonNavAbort_clicked()
  {
    move_base_client_.cancelGoal();
  }

  void UgiBaseStationPlugin::on_pushButtonMatcherFlip_clicked()
  {
    std_srvs::Trigger trigger;
    matcher_flip_srv_client_.call(trigger);
  }

  void UgiBaseStationPlugin::on_pushButtonMatcherStart_toggled(bool checked)
  {
    std_srvs::Trigger trigger;
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
      QApplication::setOverrideCursor(QCursor(cursor_pixmap));
    }
    if (!checked && !navSetGoal_checked)
    {
      QApplication::restoreOverrideCursor();
      QApplication::setOverrideCursor(Qt::ArrowCursor);
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
    if (!checked)
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
  }

  void UgiBaseStationPlugin::on_pushButtonPatrolAbortClear_clicked()
  {
    vertices_.clear();
    transformed_vertices_.clear();

    std_srvs::Trigger trigger;
    path_ready_srv_client_.call(trigger);
  }
}
