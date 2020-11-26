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