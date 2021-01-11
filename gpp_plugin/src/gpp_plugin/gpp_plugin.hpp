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

#include <gpp_interface/post_planning_interface.hpp>
#include <gpp_interface/pre_planning_interface.hpp>
#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mbf_costmap_core/costmap_planner.h>
#include <nav_core/base_global_planner.h>
#include <pluginlib/class_loader.h>
#include <ros/ros.h>

#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace gpp_plugin {

/**
 * @brief POD defining the meta information required to load a plugin
 *
 * This allows us to define the required tags (name and type) at compile time
 */
template <typename _Plugin>
struct PluginDefinition {
  using type = _Plugin;
  static const std::string package;
  static const std::string base_class;
};

// define shortcuts to the resource types
using gpp_interface::PostPlanningInterface;
using gpp_interface::PrePlanningInterface;
using mbf_costmap_core::CostmapPlanner;
using nav_core::BaseGlobalPlanner;

// clang-format off

// below the definition of the different plugins.
// we put it into the header, since this is part of the interface.
// extend here, if you want to load other types.

// Preplanning specialization
template <>
const std::string PluginDefinition<PrePlanningInterface>::package = "gpp_interface";

template <>
const std::string PluginDefinition<PrePlanningInterface>::base_class = "gpp_interface::PrePlanningInterface";

// Preplanning specialization
template <>
const std::string PluginDefinition<PostPlanningInterface>::package = "gpp_interface";

template <>
const std::string PluginDefinition<PostPlanningInterface>::base_class = "gpp_interface::PostPlanningInterface";

// nav-core specialization
template <>
const std::string PluginDefinition<BaseGlobalPlanner>::package = "nav_core";

template <>
const std::string PluginDefinition<BaseGlobalPlanner>::base_class = "nav_core::BaseGlobalPlanner";

// mbf-costmap-core specialization
template <>
const std::string PluginDefinition<CostmapPlanner>::package = "mbf_costmap_core";

template <>
const std::string PluginDefinition<CostmapPlanner>::base_class = "mbf_costmap_core::CostmapPlanner";

// clang-format on

// below the plugin loading machinery. the implementation is moved into cpp,
// since we provide the (only valid) template arguments at compile time.

/**
 * @brief Base class for loading a plugin with a valid PluginDefinition
 *
 * @tparam _Plugin Plugin-type. You need to provide a specialization of
 * the PluginDefinition for the _Plugin for this to work.
 *
 * The class firstly binds the pluginlib::ClassLoader to our PluginDefinition.
 * Secondly it defined our way to load the plugins (namely by returning a unique
 * ptr). This method is declared virtual, so we can load CostmapPlanner
 * under the interface of BaseGlobalPlanners.
 */
template <typename _Plugin>
struct PluginManager : public pluginlib::ClassLoader<_Plugin> {
  // the defintion with compile-time package and base_class tags
  using Definition = PluginDefinition<_Plugin>;
  using Base = pluginlib::ClassLoader<_Plugin>;

  // this will get you a linker error, if you try to load unknown plugins
  PluginManager() : Base(Definition::package, Definition::base_class) {}

  virtual pluginlib::UniquePtr<_Plugin>
  createCustomInstance(const std::string& _type) {
    return Base::createUniqueInstance(_type);
  }
};

/**
 * @brief Parameters defining how to execute a plugin.
 *
 * If on_success_break is set to true, and the plugin succeeds, the entire
 * plugin-group (pre-, post-planning or planning) succeeds.
 *
 * If on_failure_break is set to true, and the plugin fails, the entire
 * plugin-group fails.
 */
struct PluginParameter {
  std::string name;
  bool on_success_break = false;
  bool on_failure_break = true;
};

/**
 * @brief Common interface to a plugin group
 *
 * The class defines the ownership and storage of plugins.
 * Typically we own the plugin and store all of them in a vector.
 */
template <typename _Plugin>
struct PluginGroup {
  // this class owns the plugin
  using PluginPtr = typename pluginlib::UniquePtr<_Plugin>;

  using NamedPlugin = std::pair<PluginParameter, PluginPtr>;
  using PluginMap = std::vector<NamedPlugin>;

  inline const PluginMap&
  getPlugins() const noexcept {
    return plugins_;
  }

  inline const std::string&
  getName() const noexcept {
    return name_;
  }

  inline const bool&
  getDefaultValue() const noexcept {
    return default_value_;
  }

protected:
  bool default_value_;
  std::string name_ = "undefined";
  PluginMap plugins_;
};

/**
 * @brief Execution logic to run all plugins within one group
 *
 * @tparam _Plugin type of the plugin (PrePlanningInterface, etc)
 * @tparam _Functor functor taking the _Plugin-ref and returning true on success
 *
 * This function implements the main logic, how to map the result from plugins
 * within a group to the group result.
 *
 * @param _grp a group of plugins
 * @param _func a functor responsible for calling the plugin's main function.
 * @param _cancel boolean cancel flag.
 */
template <typename _Plugin, typename _Functor>
bool
_runPlugins(const PluginGroup<_Plugin>& _grp, const _Functor& _func,
            const std::atomic_bool& _cancel) {
  const auto& plugins = _grp.getPlugins();
  const std::string name = "[" + _grp.getName() + "]: ";
  for (const auto& plugin : plugins) {
    // allow the user to cancel the job
    if (_cancel) {
      ROS_INFO_STREAM(name << "cancelled");
      return false;
    }

    // tell my name
    ROS_INFO_STREAM(name << "runs " << plugin.first.name);

    // run the impl, but don't die
    if (!plugin.second || !_func(*plugin.second)) {
      // we have failed - we can either abort or ignore
      ROS_WARN_STREAM(name << "failed at " << plugin.first.name);
      if (plugin.first.on_failure_break)
        return false;
    }
    else if (plugin.first.on_success_break)
      return true;
  }
  return _grp.getDefaultValue();
}

/// @brief as _runPlugins but with a warning on failure
template <typename _Plugin, typename _Functor>
bool
runPlugins(const PluginGroup<_Plugin>& _grp, const _Functor& _func,
           const std::atomic_bool& _cancel) {
  const auto result = _runPlugins(_grp, _func, _cancel);
  // print a conditional warning
  ROS_WARN_STREAM_COND(!result, "[gpp]: failed at group " << _grp.getName());
  return result;
}
/**
 * @brief Loads an array of plugins.
 *
 * @section Usage
 *
 * The class offers two methods:
 * - load will try to read the types and names from the parameter server and
 *   load the specified plugins.
 * - getPlugins will return the loaded plugins.
 *
 * Both methods are not thread-safe. The concurrency management must be done
 * by the user.
 *
 * Code example:
 *
 * @code{cpp}
 * // you will need a note-handle
 * ros::NodeHandle nh("~");
 *
 * // create the manager
 * ArrayPluginManager<MyPlugin> manager;
 *
 * // load the plugins
 * try{
 *  // will throw if "my_resource_tag" does not specify an array
 *  manager.load("my_resource_tag", nh);
 * }
 * catch(std::invalid_argument& ex){
 *  std::cerr << "failed to load " << ex.what() << std::endl;
 *  return;
 * }
 *
 * // get the plugins and init them as you like
 * const auto& plugins = manager.getPlugins();
 * ...
 * @endcode
 *
 * @section Parameters
 *
 * The ros-parameter resource defined under the _resource argument (see
 * load-method) must be an array.
 * The array resquires two tags - 'name' and 'type'.
 * 'name' defines a unique descriptor which will be passed to a plugin.
 * 'type' defines the type of the plugin.
 * Both tags must have literal values.
 * Additionally it may contain boolean 'on_failure_break' (defaults to true) and
 * 'on_success_break' (defaults to false) tags. See PluginParameter for details.
 *
 * Code example:
 *
 * @code{yaml}
 * my_resource_tag:
 *  - {name: foo, type: a_valid_type}
 *  - {name: baz, type: another_type, on_failure_break: false, on_success_break:
 * true}
 * @endcode
 *
 * @section Remarks
 *
 * We don't need an explicit destructor here, since the destructors are
 * called in reverse order of the construction: so the ManagerInterface
 * is always destructed first.
 *
 * https://stackoverflow.com/questions/31518581/order-of-destruction-in-the-case-of-multiple-inheritance
 *
 * @tparam _Plugin as in PluginManager<_Plugin>
 *
 */
template <typename _Plugin>
struct ArrayPluginManager : public PluginManager<_Plugin>,
                            public PluginGroup<_Plugin> {
  void
  load(const std::string& _resource, ros::NodeHandle& _nh);
};

// compile time specification of the ArrayPluginManager
using PrePlanningManager = ArrayPluginManager<PrePlanningInterface>;
using PostPlanningManager = ArrayPluginManager<PostPlanningInterface>;
using GlobalPlannerManager = ArrayPluginManager<BaseGlobalPlanner>;

/**
 * @brief Wrappes the CostmapPlaner API into the traditional BaseGlobalPlanner
 * API.
 *
 * Implementation is very simpliar to move-base-flex wrapper class.
 * However, all publicly available planners stick to the BaseGlobalPlanner API,
 * so we treat it as default.
 */
struct BaseGlobalPlannerWrapper : public BaseGlobalPlanner {
  // define the interface types
  using Pose = geometry_msgs::PoseStamped;
  using Path = std::vector<Pose>;
  using Map = costmap_2d::Costmap2DROS;
  using ImplPlanner = pluginlib::UniquePtr<CostmapPlanner>;

  /// @brief our c'tor
  /// @param _impl a valid instance of the CostmapPlanner, which we will own
  /// @throw std::invalid_argument, if _impl is nullptr
  explicit BaseGlobalPlannerWrapper(ImplPlanner&& _impl);

  bool
  makePlan(const Pose& start, const Pose& goal, Path& plan) override;

  bool
  makePlan(const Pose& start, const Pose& goal, Path& plan,
           double& cost) override;

  void
  initialize(std::string name, Map* costmap_ros) override;

private:
  ImplPlanner impl_;
};

/**
 * @brief Loads either CostmapPlanner or BaseGlobalPlanner plugins under a
 * uniform interface.
 *
 * The usage is equivalent to the ArrayPluginManager, but you can pass
 * plugins derived form CostmapPLanner or BaseGlobalPlanner to it.
 */
struct CostmapPlannerManager : public GlobalPlannerManager {
  ~CostmapPlannerManager();

  // dont call this yourself
  pluginlib::UniquePtr<BaseGlobalPlanner>
  createCustomInstance(const std::string& _type) override;

private:
  PluginManager<CostmapPlanner> manager_;
};

/**
 * @brief Combine pre-planning, planning and post-planning to customize the
 * your path.
 *
 * The planner implements BaseGlobalPlanner and CostmapPlanner interfaces.
 *
 * @section Parameters
 *
 * Define the pre_planning plugins under the tag `pre_planning`. Those plugins
 * must adhere to the `gpp_interface::PrePlanning` interface. Define planning
 * plugins under the tag `planning` and the post-planning plugins under the tag
 * `post_planning`. Those plugins must adhere to `nav_core::BaseGlobalPlanner`
 * and `gpp_interface::PostPlanning`, respectively.
 *
 * The plugins under every tag (`pre_planning`, `planning` or `post_planning`)
 * must be defined as an array. Every element withing the array must have
 * the tags `name` and `type` - following the standard ros syntax for
 * pluginlib-loaded plugins.
 *
 * Below is a code example
 *
 * @code{yaml}
 *
 * # define the pre-planning plugins
 * pre_planning:
 * -  {name: first_pre_planning_name, type: first_pre_planning_type}
 * -  {name: second_pre_planning_name, type: second_pre_planning_type}
 *
 * # define the planning plugins
 * planning:
 * -  {name: first_planning_name, type: first_planning_type}
 * -  {name: second_planning_name, type: second_planning_type}
 *
 * # define the post-planning plugins
 * post_planning:
 * -  {name: first_post_planning_name, type: first_post_planning_type}
 * -  {name: second_post_planning_name, type: second_post_planning_type}
 *
 * # define the default values for the groups
 * pre_planning_default_value: true
 * planning_default_value: false
 * post_planning_default_value: true
 *
 * @endcode
 *
 */
struct GppPlugin : public BaseGlobalPlanner, public CostmapPlanner {
  // define our interface types
  using Pose = geometry_msgs::PoseStamped;
  using Path = std::vector<Pose>;
  using Map = costmap_2d::Costmap2DROS;

  bool
  makePlan(const Pose& _start, const Pose& _goal, Path& _plan) override;

  bool
  makePlan(const Pose& _start, const Pose& _goal, Path& _plan,
           double& _cost) override;

  uint32_t
  makePlan(const Pose& start, const Pose& goal, double tolerance, Path& plan,
           double& cost, std::string& message) override;
  void
  initialize(std::string name, Map* costmap_ros) override;

  bool
  cancel() override;

private:
  bool
  prePlanning(Pose& _start, Pose& _goal);

  bool
  postPlanning(Path& _path, double& _cost);

  bool
  globalPlanning(const Pose& _start, const Pose& _goal, Path& _plan,
                 double& _cost);

  std::atomic_bool cancel_;

  // nav_core conforming members
  std::string name_;
  Map* costmap_ = nullptr;

  PrePlanningManager pre_planning_;
  PostPlanningManager post_planning_;
  CostmapPlannerManager global_planning_;
};

}  // namespace gpp_plugin