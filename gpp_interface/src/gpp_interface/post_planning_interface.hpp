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
   * @param _name name of the resource
   * @param _map costmap containing the data
   */
  virtual void
  initialize(const std::string &_name, Map *_map) = 0;
};

}  // namespace gpp_interface