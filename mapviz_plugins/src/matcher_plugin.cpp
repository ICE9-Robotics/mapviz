// *****************************************************************************
//
// Copyright (c) 2021, Trinity University
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
// ARE DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// *****************************************************************************

#include <mapviz_plugins/matcher_plugin.h>
#include "rtkgps_odom_matcher/SetPose.h"


// Declare plugin
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mapviz_plugins::MatcherPlugin, mapviz::MapvizPlugin)

namespace mapviz_plugins
{

MatcherPlugin::MatcherPlugin() :
    config_widget_(new QWidget()),
    map_canvas_(nullptr),
    is_mouse_down_(false),
    monitoring_action_state_(false)
{
    ui_.setupUi(config_widget_);

    // Set background white
    QPalette p(config_widget_->palette());
    p.setColor(QPalette::Background, Qt::white);
    config_widget_->setPalette(p);

    // Set status text green
    ui_.status->setText("No GPS");
    QPalette p3(ui_.status->palette());
    p3.setColor(QPalette::Text, Qt::red);
    ui_.status->setPalette(p3);

    QObject::connect(ui_.pushButtonEstimate, SIGNAL(toggled(bool)),
                     this, SLOT(on_pushButtonEstimate_toggled(bool)));

    QObject::connect(ui_.pushButtonStartStop, SIGNAL(toggled(bool)), this, SLOT(on_pushButtonStartStop_toggled(bool)));

    timer_ = nh_.createTimer(ros::Duration(1.0), &MatcherPlugin::timerCallback, this);
    pose_service_client_ = nh_.serviceClient<rtkgps_odom_matcher::SetPose>("send_pose_est");
    start_service_client_ = nh_.serviceClient<std_srvs::Trigger>("start_match");
    stop_service_client_ = nh_.serviceClient<std_srvs::Trigger>("stop_match");
}

MatcherPlugin::~MatcherPlugin()
{
    if (map_canvas_)
    {
        map_canvas_->removeEventFilter(this);
    }
}

void MatcherPlugin::PrintError(const std::string& message)
{
    PrintErrorHelper( ui_.status, message);
}

void MatcherPlugin::PrintInfo(const std::string& message)
{
    PrintInfoHelper( ui_.status, message);
}

void MatcherPlugin::PrintWarning(const std::string& message)
{
    PrintWarningHelper( ui_.status, message);
}

QWidget* MatcherPlugin::GetConfigWidget(QWidget* parent)
{
    config_widget_->setParent(parent);
    return config_widget_;
}

bool MatcherPlugin::Initialize(QGLWidget* canvas)
{
    map_canvas_ = static_cast<mapviz::MapCanvas*>(canvas);
    map_canvas_->installEventFilter(this);

    initialized_ = true;
    return true;
}

bool MatcherPlugin::eventFilter(QObject *object, QEvent* event)
{
    switch (event->type())
    {
    case QEvent::MouseButtonPress:
        return handleMousePress(static_cast<QMouseEvent*>(event));
    case QEvent::MouseButtonRelease:
        return handleMouseRelease(static_cast<QMouseEvent*>(event));
    case QEvent::MouseMove:
        return handleMouseMove(static_cast<QMouseEvent*>(event));
    default:
        return false;
    }
}

void MatcherPlugin::LoadConfig(const YAML::Node& node, const std::string& path) 
{
}

void MatcherPlugin::SaveConfig(YAML::Emitter& emitter, const std::string& path) 
{
}

void MatcherPlugin::timerCallback(const ros::TimerEvent &)
{
  ui_.pushButtonEstimate->setEnabled( true );
  ui_.pushButtonStartStop->setEnabled( true );
}

bool MatcherPlugin::handleMousePress(QMouseEvent* event)
{
    bool pose_checked = ui_.pushButtonEstimate->isChecked();

    if( !pose_checked)
    {
        return false;
    }

    if (event->button() == Qt::LeftButton)
    {
        is_mouse_down_ = true;
        arrow_angle_ = 0;
#if QT_VERSION >= 0x050000
      arrow_tail_position_= map_canvas_->MapGlCoordToFixedFrame( event->localPos() );
#else
      arrow_tail_position_= map_canvas_->MapGlCoordToFixedFrame( event->posF() );
#endif
        return true;
    }
    return false;
}

bool MatcherPlugin::handleMouseMove(QMouseEvent* event)
{
    if (is_mouse_down_)
    {
#if QT_VERSION >= 0x050000
        QPointF head_pos = map_canvas_->MapGlCoordToFixedFrame( event->localPos() );
#else
        QPointF head_pos = map_canvas_->MapGlCoordToFixedFrame( event->posF() );
#endif
        arrow_angle_ = atan2( head_pos.y() - arrow_tail_position_.y(),
                              head_pos.x() - arrow_tail_position_.x() );
    }
    return false;
}

bool MatcherPlugin::handleMouseRelease(QMouseEvent* event)
{
    if( !is_mouse_down_ )
    {
        return false;
    }

    is_mouse_down_ = false;

    bool pose_checked = ui_.pushButtonEstimate->isChecked();
    if( !pose_checked )
    {
        return false;
    }


    if( pose_checked )
    {
      // Get angle/arrow_tail_position in fixed_frame
      tf::Quaternion quat_ff = tf::createQuaternionFromYaw(arrow_angle_);
      rtkgps_odom_matcher::SetPose setPose;

      // Here it is in the target_frame_
      setPose.request.pose.header.frame_id = target_frame_;
      setPose.request.pose.header.stamp = ros::Time::now();
      setPose.request.pose.pose.pose.position.x = arrow_tail_position_.x();
      setPose.request.pose.pose.pose.position.y = arrow_tail_position_.y();
      setPose.request.pose.pose.pose.position.z = 0.0;
      tf::quaternionTFToMsg( quat_ff, setPose.request.pose.pose.pose.orientation );

      if(pose_service_client_.call(setPose))
      {
        PrintInfo("Successful");
      }
      else
      {
        PrintError("Failed to send pose estimation");
      }

      ui_.pushButtonEstimate->setChecked(false);
    }
    return true;
}

void MatcherPlugin::Draw(double x, double y, double scale)
{
    std::array<QPointF, 7> arrow_points;
    arrow_points[0] = QPointF(10, 0);
    arrow_points[1] = QPointF(6, -2.5);
    arrow_points[2] = QPointF(6.5, -1);
    arrow_points[3] = QPointF(0, -1);
    arrow_points[4] = QPointF(0, 1);
    arrow_points[5] = QPointF(6.5, 1);
    arrow_points[6] = QPointF(6, 2.5);

    if( is_mouse_down_ )
    {
        QPointF transformed_points[7];
        for (size_t i=0; i<7; i++ )
        {
            tf::Vector3 point(arrow_points[i].x(), arrow_points[i].y(), 0);
            point *= scale*10;
            point = tf::Transform( tf::createQuaternionFromYaw(arrow_angle_)  ) * point;
            transformed_points[i] = QPointF(point.x() + arrow_tail_position_.x(),
                                            point.y() + arrow_tail_position_.y() );
        }
        glColor3f(0.1, 0.9, 0.1);
        glLineWidth(2);
        glBegin(GL_TRIANGLE_FAN);
        for (const QPointF& point: transformed_points )
        {
            glVertex2d(point.x(), point.y());
        }
        glEnd();

        glColor3f(0.0, 0.6, 0.0);
        glBegin(GL_LINE_LOOP);
        for (const QPointF& point: transformed_points )
        {
            glVertex2d(point.x(), point.y());
        }
        glEnd();
    }
}

void MatcherPlugin::on_pushButtonEstimate_toggled(bool checked)
{
    if(checked)
    {
      QPixmap cursor_pixmap = QPixmap(":/images/green-arrow.png");
      QApplication::setOverrideCursor(QCursor(cursor_pixmap));
    }
    else
    {
      QApplication::restoreOverrideCursor();
    }
}

void MatcherPlugin::on_pushButtonStartStop_toggled(bool checked)
{
    std_srvs::Trigger trigger;
    if(checked)
    {
        if (start_service_client_.call(trigger))
        {
            ui_.pushButtonStartStop->setText(" Stop Matcher");
            PrintInfo("Started");
        }
        else
        {
            PrintError("Failed to start");
        }
      
    }
    else
    {
        if (stop_service_client_.call(trigger))
        {
            ui_.pushButtonStartStop->setText(" Start Matcher");
            PrintInfo("Stopped");
        }
        else
        {
            PrintError("Failed to stop");
        }
    }
}

}
