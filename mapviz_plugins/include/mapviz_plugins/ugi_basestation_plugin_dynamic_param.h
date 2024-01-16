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

#ifndef MAPVIZ_PLUGINS_UGI_BASESTATION_PLUGIN_DYNAMIC_PARAM_H
#define MAPVIZ_PLUGINS_UGI_BASESTATION_PLUGIN_DYNAMIC_PARAM_H

#include <mapviz/mapviz_plugin.h>
#include <ros/ros.h>
#include <dynamic_reconfigure/Config.h>
#include <dynamic_reconfigure/client.h>
#include <origin_transform/OriginTransformConfig.h>

namespace mapviz_plugin{
    class UgiBaseStationPluginDynamicParam
    {
        public:
            UgiBaseStationPluginDynamicParam();
            ~UgiBaseStationPluginDynamicParam();

            void updateOriginTransformParam(const origin_transform::OriginTransformConfig &config);
            void getOriginTransformParam(origin_transform::OriginTransformConfig &config);

        private:
            dynamic_reconfigure::Client<origin_transform::OriginTransformConfig> origin_transform_param_client_;
            origin_transform::OriginTransformConfig origin_transform_pending_config_;
            origin_transform::OriginTransformConfig origin_transform_latest_config_;
            bool is_origin_transform_update_pending_;
            boost::mutex::scoped_lock origin_transform_config_lock_;
            ros::Timer origin_transform_update_timer_;
            
            void originTransformTimerCallback(const ros::TimerEvent& ev = ros::TimerEvent());
};
}

#endif  // MAPVIZ_PLUGINS_UGI_BASESTATION_PLUGIN_DYNAMIC_PARAM_H