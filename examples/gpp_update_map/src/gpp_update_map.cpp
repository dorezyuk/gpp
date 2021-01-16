/*
 * MIT License
 *
 * Copyright (c) 2021 Dima Dorezyuk
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include <gpp_update_map/gpp_update_map.hpp>

#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

namespace gpp_update_map {

/// @brief plugin name
constexpr char __name[] = "[gpp_update_map]: ";

// custom prints with the plugin name
#define GPP_DEBUG(args) ROS_DEBUG_STREAM(__name << args)
#define GPP_WARN(args) ROS_WARN_STREAM(__name << args)

bool
GppUpdateMap::preProcess(Pose& _start, Pose& _goal) {
  // just to be sure
  if (!map_) {
    GPP_WARN("map_ cannot be nullptr: call first initialize");
    return false;
  }

  // uppdate the map
  map_->updateMap();
  return true;
}

void
GppUpdateMap::initialize(const std::string& _name, Map* _map) {
  // we might check here for nullptr, but it maybe the case that the user sets
  // this on purpose...
  map_ = _map;
}

}  // namespace gpp_update_map

// register the plugin
PLUGINLIB_EXPORT_CLASS(gpp_update_map::GppUpdateMap,
                       gpp_interface::PrePlanningInterface);