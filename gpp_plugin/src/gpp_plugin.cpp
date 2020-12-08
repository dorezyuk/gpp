#include <gpp_plugin/gpp_plugin.hpp>

#include <pluginlib/class_list_macros.h>

namespace gpp_plugin {

// define name of the lib
constexpr char gpp_name__[] = "[gpp]: ";

// define named prints
#define GPP_DEBUG(_msg) ROS_DEBUG_STREAM(gpp_name__ << _msg)
#define GPP_INFO(_msg) ROS_INFO_STREAM(gpp_name__ << _msg)
#define GPP_WARN(_msg) ROS_WARN_STREAM(gpp_name__ << _msg)
#define GPP_ERROR(_msg) ROS_ERROR_STREAM(gpp_name__ << _msg)
#define GPP_FATAL(_msg) ROS_FATAL_STREAM(gpp_name__ << _msg)

// outcome definition and checks
constexpr uint32_t SUCCESS = 0;
constexpr uint32_t FAILURE = 50;

/// @brief helper to get a string element with the tag _tag from _v
/// @throw XmlRpc::XmlRpcException if the tag is missing
inline std::string
getStringElement(const XmlRpc::XmlRpcValue& _v, const std::string& _tag) {
  // we have to check manually, since XmlRpc would just return _tag if its
  // missing...
  if (!_v.hasMember(_tag))
    throw XmlRpc::XmlRpcException(_tag + " not found");

  return static_cast<std::string>(_v[_tag]);
}

template <typename _Plugin>
void
ArrayPluginManager<_Plugin>::load(const std::string& _resource,
                                  ros::NodeHandle& _nh) {
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
  ManagerInterface<_Plugin>::plugins_.clear();
  ManagerInterface<_Plugin>::plugins_.reserve(size);

  // note: size raw.size() returns int
  for (int ii = 0; ii != size; ++ii) {
    const auto& element = raw[ii];

    try {
      // will throw if the tags are missing or not convertable to std::string
      const auto type = getStringElement(element, "type");
      const auto name = getStringElement(element, "name");

      // will throw if the loading fails
      // mind the "this"
      auto plugin = this->createCustomInstance(type);

      // this should not throw anymore
      ManagerInterface<_Plugin>::plugins_.emplace_back(name, std::move(plugin));

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
  return impl_->makePlan(start, goal, 0, plan, cost, message) == SUCCESS;
}

void
BaseGlobalPlannerWrapper::initialize(std::string _name, Map* _map) {
  impl_->initialize(_name, _map);
}

CostmapPlannerManager::~CostmapPlannerManager() { plugins_.clear(); }

inline void
default_deleter(BaseGlobalPlanner* impl) {
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
      new BaseGlobalPlannerWrapper(std::move(impl_planner)), default_deleter};
}

void
initPrePlanning(ros::NodeHandle& _nh, PrePlanningManager& _pre) {
  // load the plugins
  _pre.load("pre_planning", _nh);

  // init the plugins
  const auto& plugins = _pre.getPlugins();
  for (const auto& plugin : plugins)
    plugin.second->initialize(plugin.first);
}

using costmap_2d::Costmap2DROS;

void
initPostPlanning(ros::NodeHandle& _nh, Costmap2DROS* _costmap,
                 PostPlanningManager& _post) {
  // load the plugins
  _post.load("post_planning", _nh);

  // init the plugins
  const auto& plugins = _post.getPlugins();
  for (const auto& plugin : plugins)
    plugin.second->initialize(plugin.first, _costmap);
}

void
initPlanning(ros::NodeHandle& _nh, Costmap2DROS* _costmap,
             CostmapPlannerManager& _planner) {
  // load the plugins
  _planner.load("planning", _nh);

  // init the plugins
  const auto& plugins = _planner.getPlugins();
  for (const auto& plugin : plugins)
    plugin.second->initialize(plugin.first, _costmap);
}

void
GlobalPlannerPipeline::initialize(std::string _name, Map* _costmap) {
  name_ = _name;
  costmap_ = _costmap;

  // init the plugins
  ros::NodeHandle nh("~" + name_);
  tolerance_ = nh.param("tolerance", 0.1);

  // load the plugins from the param-server
  initPrePlanning(nh, pre_planning_);
  initPostPlanning(nh, costmap_, post_planning_);
  initPlanning(nh, costmap_, global_planning_);
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
  return makePlan(_start, _goal, tolerance_, _plan, _cost, message) == SUCCESS;
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
    return FAILURE;

  // planning
  if (!globalPlanning(start, goal, _plan, _cost))
    return FAILURE;

  // post-planning
  if (!postPlanning(_plan, _cost))
    return FAILURE;

  return SUCCESS;
}

bool
GlobalPlannerPipeline::prePlanning(Pose& _start, Pose& _goal,
                                   double _tolerance) {
  const auto& plugins = pre_planning_.getPlugins();
  for (const auto& plugin : plugins)
    if (cancel_ ||
        !plugin.second->preProcess(_start, _goal, *costmap_, _tolerance)) {
      GPP_ERROR("pre-planning failed at " << plugin.first);
      return false;
    }

  return true;
}

bool
GlobalPlannerPipeline::postPlanning(Path& _path, double& _cost) {
  const auto& plugins = post_planning_.getPlugins();
  for (const auto& plugin : plugins)
    if (cancel_ || !plugin.second->postProcess(_path, _cost)) {
      GPP_WARN("post-planning failed at " << plugin.first);
      return false;
    }

  return true;
}

bool
GlobalPlannerPipeline::globalPlanning(const Pose& _start, const Pose& _goal,
                                      Path& _plan, double& _cost) {
  const auto& plugins = global_planning_.getPlugins();

  // the whole class does not make sense without a global planner
  if (plugins.empty()) {
    GPP_WARN("no planning plugin provided");
    return false;
  }

  // run all global planners... typically only one should be loaded.
  for (const auto& plugin : plugins)
    if (cancel_ || !plugin.second->makePlan(_start, _goal, _plan, _cost)) {
      GPP_WARN("planning failed at " << plugin.first);
      return false;
    }

  return true;
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
