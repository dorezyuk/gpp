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

namespace gpp_interface {

/**
 * @brief A Pre-Planning class will be run before the planner.
 *
 * Use this class to alter the input for your planner.
 * You can implement
 * - a generic support for goal tolerances
 * - generic checks if the start/goal pose are within map bounds
 * - a way to update the global map just before the planning
 * - alter the map to add specific information for the planner
 * - etc.
 *
 * The class offers two methods. Following the standard-approach of
 * ros-navigation plugins, call first initialize to initialize the instance.
 * Then you can use the instance by calling preProcess
 *
 * @code{cpp}
 *
 * // dummy implementation
 * struct MyPrePlanning : public PrePlanningInterface{
 *  bool
 *  preProcess(Pose& _start, Pose& _goal, Map& _map, double _tolerance) override
 *  {}
 *
 *  void
 *  initialize(const std::string& _name) override {}
 * };
 *
 * // now create an instance
 * MyPrePlanning pre_planning;
 *
 * // init your instance
 * pre_planning.initialize("my_pre_planning");
 *
 * // use it
 * if(pre_planning.preProcess(...)) {
 *  std::cout << "pre_planning worked" << std::endl;
 * }
 * else {
 *  std::cerr << "pre_planning failed" << std::endl;
 * }
 *
 * @endcode
 *
 */
struct PrePlanningInterface {
  // define the interface types
  using Pose = geometry_msgs::PoseStamped;
  using Map = costmap_2d::Costmap2DROS;

  // polymorphism required for this class
  virtual ~PrePlanningInterface() = default;

  /**
   * @param _start Start pose for the planning problem
   * @param _goal Goal pose for the planning problem
   * @param _map Map on which the planning problem will be preformed
   * @param _tolerance tolerance for the goal pose
   *
   * @return true, if successful
   */
  virtual bool
  preProcess(Pose& _start, Pose& _goal, Map& _map, double _tolerance) = 0;

  virtual void
  initialize(const std::string& _name) = 0;
};

}  // namespace gpp_interface