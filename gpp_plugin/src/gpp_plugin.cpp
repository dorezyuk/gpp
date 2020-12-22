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

#include <gpp_plugin/gpp_plugin.hpp>

#include <pluginlib/class_list_macros.h>
#include <xmlrpcpp/XmlRpcException.h>
#include <xmlrpcpp/XmlRpcValue.h>

#include <stdexcept>
#include <utility>

namespace gpp_plugin {

// define name of the lib
constexpr char gpp_name__[] = "[gpp]: ";

// define named prints
#define GPP_DEBUG(_msg) ROS_DEBUG_STREAM(gpp_name__ << _msg)
#define GPP_INFO(_msg) ROS_INFO_STREAM(gpp_name__ << _msg)
#define GPP_WARN(_msg) ROS_WARN_STREAM(gpp_name__ << _msg)
#define GPP_ERROR(_msg) ROS_ERROR_STREAM(gpp_name__ << _msg)
#define GPP_FATAL(_msg) ROS_FATAL_STREAM(gpp_name__ << _msg)

// outcome definition for mbf_costmap_core based plugins
constexpr uint32_t MBF_SUCCESS = 0;
constexpr uint32_t MBF_FAILURE = 50;

/// @brief helper to get a string element with the tag _tag from _v
/// @throw XmlRpc::XmlRpcException if the tag is missing
inline std::string
_getStringElement(const XmlRpc::XmlRpcValue& _v, const std::string& _tag) {
  // we have to check manually, since XmlRpc would just return _tag if its
  // missing...
  if (!_v.hasMember(_tag))
    throw XmlRpc::XmlRpcException(_tag + " not found");

  return static_cast<std::string>(_v[_tag]);
}

/// @brief helper to get any value from _v under _tag.
/// If anything goes wrong, the function will fall-back to the _default value.
template <typename _T>
_T
_getElement(const XmlRpc::XmlRpcValue& _v, const std::string& _tag,
            const _T& _default) noexcept {
  // check if the tag is defined (see above for explanation)
  if (!_v.hasMember(_tag))
    return _default;

  // try to get the desired value
  try {
    return static_cast<_T>(_v[_tag]);
  }
  catch (XmlRpc::XmlRpcException& _ex) {
    return _default;
  }
}

template <typename _Plugin>
void
ArrayPluginManager<_Plugin>::load(const std::string& _resource,
                                  ros::NodeHandle& _nh) {
  // load the group parameters
  PluginGroup<_Plugin>::name_ = _resource;
  PluginGroup<_Plugin>::default_value_ =
      _nh.param(_resource + "_default_value", true);

  // we expect that _resource defines an array
  using namespace XmlRpc;
  XmlRpcValue raw;

  // load the data from the param server
  if (!_nh.getParam(_resource, raw)) {
    GPP_DEBUG("no parameter " << _nh.getNamespace() << "/" << _resource);
    return;
  }

  if (raw.getType() != XmlRpcValue::TypeArray) {
    GPP_WARN("invalid type for " << _resource);
    return;
  }

  // will throw if not XmlRpcValue::TypeArray
  const auto size = raw.size();

  // clear the old data and allocate space
  PluginGroup<_Plugin>::plugins_.clear();
  PluginGroup<_Plugin>::plugins_.reserve(size);

  // note: size raw.size() returns int
  for (int ii = 0; ii != size; ++ii) {
    const auto& element = raw[ii];

    try {
      // will throw if the tags are missing or not convertable to std::string
      const auto type = _getStringElement(element, "type");
      const auto name = _getStringElement(element, "name");

      // will throw if the loading fails
      // mind the "this"
      auto plugin = this->createCustomInstance(type);

      // assemble the parameter struct
      PluginParameter param;
      param.name = name;
      param.on_failure_break = _getElement(element, "on_failure_break", true);
      param.on_success_break = _getElement(element, "on_success_break", false);
      // this should not throw anymore
      PluginGroup<_Plugin>::plugins_.emplace_back(param, std::move(plugin));

      // notify the user
      GPP_INFO("Successfully loaded " << type << " under the name " << name);
    }
    catch (XmlRpcException& ex) {
      GPP_WARN("failed to read the tag: " << ex.getMessage());
    }
    catch (pluginlib::LibraryLoadException& ex) {
      GPP_WARN("failed to load the library: " << ex.what());
    }
    catch (pluginlib::CreateClassException& ex) {
      GPP_WARN("failed to create the class: " << ex.what());
    }
  }
}

BaseGlobalPlannerWrapper::BaseGlobalPlannerWrapper(ImplPlanner&& _impl) :
    impl_(std::move(_impl)) {
  // check once so we don't have to check everytime we call makePlan
  if (!impl_)
    throw std::invalid_argument("nullptr is not supported");
}

bool
BaseGlobalPlannerWrapper::makePlan(const Pose& start, const Pose& goal,
                                   Path& plan) {
  double cost;
  return makePlan(start, goal, plan, cost);
}

bool
BaseGlobalPlannerWrapper::makePlan(const Pose& start, const Pose& goal,
                                   Path& plan, double& cost) {
  std::string message;
  return impl_->makePlan(start, goal, 0, plan, cost, message) == MBF_SUCCESS;
}

void
BaseGlobalPlannerWrapper::initialize(std::string _name, Map* _map) {
  impl_->initialize(_name, _map);
}

CostmapPlannerManager::~CostmapPlannerManager() { plugins_.clear(); }

inline void
_default_deleter(BaseGlobalPlanner* impl) {
  delete impl;
}

pluginlib::UniquePtr<BaseGlobalPlanner>
CostmapPlannerManager::createCustomInstance(const std::string& _type) {
  // check if this type is know to us
  if (isClassAvailable(_type))
    return createUniqueInstance(_type);

  // delegate the construction to the helper manager
  auto impl_planner = manager_.createCustomInstance(_type);
  // according to the pluginlib::UniquePtr, we have to privide a deleter
  // function. here, we just pass a default-deleter, since its not bound to
  // the class-loader
  return pluginlib::UniquePtr<BaseGlobalPlanner>{
      new BaseGlobalPlannerWrapper(std::move(impl_planner)), _default_deleter};
}

/// @brief helper to initialize the pre-planning plugins
void
_initPrePlanning(ros::NodeHandle& _nh, PrePlanningManager& _pre) {
  // load the plugins
  _pre.load("pre_planning", _nh);

  // init the plugins
  const auto& plugins = _pre.getPlugins();
  for (const auto& plugin : plugins)
    plugin.second->initialize(plugin.first.name);
}

using costmap_2d::Costmap2DROS;

/// @brief helper to initialize the post-planning plugins
void
_initPostPlanning(ros::NodeHandle& _nh, Costmap2DROS* _costmap,
                  PostPlanningManager& _post) {
  // load the plugins
  _post.load("post_planning", _nh);

  // init the plugins
  const auto& plugins = _post.getPlugins();
  for (const auto& plugin : plugins)
    plugin.second->initialize(plugin.first.name, _costmap);
}

/// @brief helper to initialize the planning plugins
void
_initPlanning(ros::NodeHandle& _nh, Costmap2DROS* _costmap,
              CostmapPlannerManager& _planner) {
  // load the plugins
  _planner.load("planning", _nh);

  // init the plugins
  const auto& plugins = _planner.getPlugins();
  for (const auto& plugin : plugins)
    plugin.second->initialize(plugin.first.name, _costmap);
}

void
GlobalPlannerPipeline::initialize(std::string _name, Map* _costmap) {
  name_ = _name;
  costmap_ = _costmap;

  // init the plugins
  ros::NodeHandle nh("~" + name_);
  tolerance_ = nh.param("tolerance", 0.1);

  // load the plugins from the param-server
  _initPrePlanning(nh, pre_planning_);
  _initPostPlanning(nh, costmap_, post_planning_);
  _initPlanning(nh, costmap_, global_planning_);
}

bool
GlobalPlannerPipeline::prePlanning(Pose& _start, Pose& _goal,
                                   double _tolerance) {
  auto pre_planning = [&](PrePlanningInterface& _plugin) {
    return _plugin.preProcess(_start, _goal, *costmap_, _tolerance);
  };
  return runPlugins(pre_planning_, pre_planning, cancel_);
}

bool
GlobalPlannerPipeline::postPlanning(Path& _path, double& _cost) {
  auto post_planning = [&](PostPlanningInterface& _plugin) {
    return _plugin.postProcess(_path, _cost);
  };
  return runPlugins(post_planning_, post_planning, cancel_);
}

bool
GlobalPlannerPipeline::globalPlanning(const Pose& _start, const Pose& _goal,
                                      Path& _plan, double& _cost) {
  // run all global planners... typically only one should be loaded.
  auto planning = [&](BaseGlobalPlanner& _plugin) {
    return _plugin.makePlan(_start, _goal, _plan, _cost);
  };
  return runPlugins(global_planning_, planning, cancel_);
}

bool
GlobalPlannerPipeline::makePlan(const Pose& _start, const Pose& _goal,
                                Path& _plan) {
  double cost;
  return makePlan(_start, _goal, _plan, cost);
}

bool
GlobalPlannerPipeline::makePlan(const Pose& _start, const Pose& _goal,
                                Path& _plan, double& _cost) {
  std::string message;
  // clang-format off
  return makePlan(_start, _goal, tolerance_, _plan, _cost, message) == MBF_SUCCESS;
  // clang-format on
}

uint32_t
GlobalPlannerPipeline::makePlan(const Pose& _start, const Pose& _goal,
                                const double _tolerance, Path& _plan,
                                double& _cost, std::string& _message) {
  // reset cancel flag
  cancel_ = false;

  // local copies since we might alter the poses
  Pose start = _start;
  Pose goal = _goal;
  _plan.clear();

  // pre-planning
  if (!prePlanning(start, goal, _tolerance))
    return MBF_FAILURE;

  // planning
  if (!globalPlanning(start, goal, _plan, _cost))
    return MBF_FAILURE;

  // post-planning
  if (!postPlanning(_plan, _cost))
    return MBF_FAILURE;

  return MBF_SUCCESS;
}

bool
GlobalPlannerPipeline::cancel() {
  GPP_INFO("cancelling");
  cancel_ = true;
  return true;
}

}  // namespace gpp_plugin

// register for both interfaces
PLUGINLIB_EXPORT_CLASS(gpp_plugin::GlobalPlannerPipeline,
                       nav_core::BaseGlobalPlanner);

PLUGINLIB_EXPORT_CLASS(gpp_plugin::GlobalPlannerPipeline,
                       mbf_costmap_core::CostmapPlanner);
