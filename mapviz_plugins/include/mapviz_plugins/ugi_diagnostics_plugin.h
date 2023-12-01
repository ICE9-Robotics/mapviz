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

#ifndef MAPVIZ_PLUGINS_UGI_DIAGNOSTICS_PLUGIN_H
#define MAPVIZ_PLUGINS_UGI_DIAGNOSTICS_PLUGIN_H

#include <string>

#include <mapviz/mapviz_plugin.h>

#include <QObject>
#include <QString>
#include <QColor>
#include <QWidget>
#include <QGLWidget>
#include <QPainter>
#include <QFont>
#include <QStaticText>


#include <ros/ros.h>
#include <std_msgs/String.h>
#include <marti_common_msgs/StringStamped.h>
#include <unitree_diagnostics_msgs/Diagnostics.h>

// QT autogenerated files
#include "ui_ugi_diagnostics_config.h"

namespace mapviz_plugins
{
  struct DiagnosticsInfo
  {
    double highStateTs;
    int batterySoC;
    double velocity;
    double yawSpeed;
    double commandVelocity;
    double commandYawSpeed;
    double gpsStatusTs;
    std::string gpsStatusDescription;
  };

  class UgiDiagnosticsPlugin : public mapviz::MapvizPlugin
  {
    Q_OBJECT

  public:
    enum Anchor {
      TOP_LEFT,
      TOP_CENTER,
      TOP_RIGHT,
      CENTER_LEFT,
      CENTER,
      CENTER_RIGHT,
      BOTTOM_LEFT,
      BOTTOM_CENTER,
      BOTTOM_RIGHT
    };

    enum Units {
      PIXELS,
      PERCENT
    };

    UgiDiagnosticsPlugin();
    virtual ~UgiDiagnosticsPlugin();

    bool Initialize(QGLWidget* canvas);
    void Shutdown() {}

    void Draw(double x, double y, double scale);
    void Paint(QPainter* painter, double x, double y, double scale);

    void Transform() {}

    void LoadConfig(const YAML::Node& node, const std::string& path);
    void SaveConfig(YAML::Emitter& emitter, const std::string& path);

    QWidget* GetConfigWidget(QWidget* parent);

    bool SupportsPainting()
    {
      return true;
    }

  protected:
    void PaintText(QPainter* painter);
    void PrintError(const std::string& message);
    void PrintInfo(const std::string& message);
    void PrintWarning(const std::string& message);

  protected Q_SLOTS:
    void SelectColor();
    void SelectFont();
    void SelectTopic();
    void TopicEdited();
    void SetAnchor(QString anchor);
    void SetUnits(QString units);
    void SetOffsetX(int offset);
    void SetOffsetY(int offset);

  private:
    Ui::ugi_diagnostics_config ui_;
    QWidget* config_widget_;

    std::string topic_;
    Anchor anchor_;
    Units units_;
    int offset_x_;
    int offset_y_;

    ros::Subscriber diagnostics_sub_;
    bool has_message_;
    bool has_painted_;

    ros::Timer timer_;

    QColor color_;
    QFont font_;
    QStaticText message_;

    std::stringstream messageStream_;
    DiagnosticsInfo diagnoInfo_;

    void diagnosticsCallback(const unitree_diagnostics_msgs::Diagnostics::ConstPtr& msg);
    void periodUpdate(const ros::TimerEvent& event);

    std::string AnchorToString(Anchor anchor);
    std::string UnitsToString(Units units);

    static const char* ANCHOR_KEY;
    static const char* COLOR_KEY;
    static const char* FONT_KEY;
    static const char* OFFSET_X_KEY;
    static const char* OFFSET_Y_KEY;
    static const char* TOPIC_KEY;
    static const char* UNITS_KEY;
  };
}


#endif //MAPVIZ_PLUGINS_UGI_DIAGNOSTICS_PLUGIN_H
