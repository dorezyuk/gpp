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
The `gpp_plugin` has loads three *groups* of child-plugins: the pre-planning, planning and post-planning group.
Every group may contain an arbitrary number of child-plugins.

On calling `makePlan`, the `gpp_plugin` firstly runs the pre-planning group.
The child-plugins within this group implement the `gpp_interface::PrePlanningInterface`.
Those plugins may modify the input-parameters (start and goal poses and costmap).

The modified input-parameters are then passed to the planning group.
The child-plugins within this group implement either the `nav_core::BaseGlobalPlanner` or the `mbf_costmap_core::CostmapPlanner` interface.
The user may define an arbitrary number of planning plugins - but must provide at least one.
The `gpp_plugin` will invoke all planning child-plugins, passing the generated path and cost from one planner to its successor.
This allows to create a "planner-chain" - a feature successfully used in the [moveit](https://moveit.ros.org/) framework.

Finally, the output from the planning group is passed to the post-planning group.
The child-plugins within this group implement the `gpp_interface::PostPlanningInterface`.
Every post-planning plugin may modify the generated path and cost.
Here, the user may implement post-processing steps, like path-smoothing or additional checks.

The `gpp_plugin` allows you also to customize what happens when a child-plugin finishes.
Every child-plugin has boolean `on_failure_break` and `on_success_break` flags.
- If a child-plugin fails and `on_failure_break` is set to true, the `gpp_plugin` fails.
- If a child-plugin fails and `on_failure_break` is set to false, its failure is ignored and the execution continues with the next plugin.
- If a child-plugin succeeds and `on_success_break` is set to true, the `gpp_plugin` continues with the next **group**
- If a child-plugin succeeds and `on_success_break` is set to false, the `gpp_plugin` continues with the next **plugin**

Additionally you may specify the default value for every group.
This value is returned if no "break" condition is triggered (`on_success_break` or `on_failure_break`).

### Behavior-Tree-Relation

You may set all the `on_failure_break`, `on_success_break` and default_values freely.
Nonetheless, it's maybe important to highlight that you can implement sequence and selector nodes known from [behavior-trees](https://en.wikipedia.org/wiki/Behavior_tree_(artificial_intelligence,_robotics_and_control)) with the mentioned parameters.

This works if all child-plugins within one group have the same values for `on_success_break` and `on_failure_break`.
The table below shows which values result in which behavior-tree node.

| flag             | sequence | selector |
|------------------|----------|----------|
| on_failure_break | true     | false    |
| on_success_break | false    | true     |
| default_value    | true     | false    |

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
