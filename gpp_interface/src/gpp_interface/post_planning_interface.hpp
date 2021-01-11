/*
 * MIT License
 *
 * Copyright (c) 2020 Dima Dorezyuk
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

#pragma once

#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/PoseStamped.h>

#include <string>
#include <vector>

namespace gpp_interface {

/**
 * Post-Planning class will be run after the global planner
 *
 * Use this class to alter the output of your planner.
 * You can implement
 * - trajectory smoothing
 * - orientation adjustment
 * - etc.
 */
struct PostPlanningInterface {
  // define the interface types
  using Pose = geometry_msgs::PoseStamped;
  using Path = std::vector<Pose>;
  using Map = costmap_2d::Costmap2DROS;

  // polymorphism required for this class
  virtual ~PostPlanningInterface() = default;

  /**
   * @param _path output from a global planner
   *
   * @return true, if successful
   */
  virtual bool
  postProcess(Path &_path, double &_cost) = 0;

  /**
   * @param _name Name of the resource
   * @param _map Map on which the planning problem will be preformed
   */
  virtual void
  initialize(const std::string &_name, Map *_map) = 0;
};

}  // namespace gpp_interface