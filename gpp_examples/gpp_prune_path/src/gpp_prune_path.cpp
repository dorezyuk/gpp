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

#include <gpp_prune_path/gpp_prune_path.hpp>

#include <pluginlib/class_list_macros.hpp>

namespace gpp_prune_path {

bool
GppPrunePath::postProcess(const Pose &_start, const Pose &_goal, Path &_path,
                          double &_cost) {
  // start and goal arguments are not really required here.
  // this class is a noop, if the defined skip is not a positive number
  if (skip_ <= 0)
    return true;

  // in order to prune the path, we need at least three poses - since we dont
  // want to change the start and goal pose.
  if (_path.size() < 2)
    return false;

  auto end = internal::prune(_path.begin(), _path.end(), skip_);
  _path.erase(end, _path.end());

  return true;
}

void
GppPrunePath::initialize(const std::string &_name, Map *_map) {
  // load the config
  ros::NodeHandle nh("~/" + _name);

  skip_ = nh.param("step", 2);
}
}  // namespace gpp_prune_path

// register the plugin
PLUGINLIB_EXPORT_CLASS(gpp_prune_path::GppPrunePath,
                       gpp_interface::PostPlanningInterface);
