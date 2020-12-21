# GlobalPlannerPipeline

The GlobalPlannerPipeline (gpp) is a [pluginlib](http://wiki.ros.org/pluginlib) based framework aiming to extend the default functionality of ros-based planners, implementing either the [nav_core::BaseGlobalPlanner](http://wiki.ros.org/nav_core?distro=noetic#BaseGlobalPlanner_C.2B-.2B-_API) or [mbf_costmap_core::CostmapPlanner](https://github.com/magazino/move_base_flex/blob/master/mbf_costmap_core/include/mbf_costmap_core/costmap_planner.h) interfaces.

## Overview

This project has two major components: [gpp_interface](gpp_interface) and [gpp_plugin](gpp_plugin).

The first part (`gpp_interface`) defines two new plugin-types:
`gpp_interface::PrePlanningInterface` and`gpp_interface::PostPlanningInterface`.
These plugins allow the user to separate common "auxiliary" functions from the planner implementation and reuse those.

The second part (`gpp_plugin`)` offers a [move_base](http://wiki.ros.org/move_base) and [move_base_flex](http://wiki.ros.org/move_base_flex) compatible global planner plugin.
This plugin implements the "pipeline" itself.
It will load and run an arbitrary number of pre-planning, planning and post-planning plugins.

## GppPlugin

![image](docs/schematic.svg)

The image above illustrates the "pipeline" concept.

On calling `makePlan`, the `gpp_plugin` will firstly run all pre-planning plugins. 
Those plugins may modify the input-parameters (start and goal poses and costmap).

The modified input-parameters are then passed to the planning plugins.
The user may define an arbitrary number of planning plugins - but must provide at least one.
The `gpp_plugin` will invoke all planning plugins, passing the generated path and cost from one planner to its successor.
This allows to create a "planner-chain" - a feature successfully used in the [moveit](https://moveit.ros.org/) framework.

Finally, the output from the planning step is passed to the post-planning plugins.
Every post-planning plugin may modify the generated path and cost.
Here, the user may implement post-processing steps, like path-smoothing or additional checks.

The `gpp_plugin` offers no possibility to recover from a plugin-failure;
This means the entire execution fails, if one of the loaded plugins fails.
Keep this in mind, when designing and implementing your pre- and post-planning plugins.

## PrePlanning

The pre-planning plugins should modify the input to the planning plugins.
The pre-planning plugins are allowed to
- alter the start and goal poses,
- alter the costmap.

This allows the user to implement for example a common tolerance function for all planners, or to update the global_costmap only when required.

## PostPlanning

The post-planning plugins should modify the output of the planning plugins.
The post-planning plugins are allowed to
- alter the path,
- alter the cost.

Here, the user may implement common post-processing steps like
- path-pruning or densifying,
- trajectory smoothing,
- trajectory oscillation checks,
- feasibility checks
