# GppInterface

This package defines two additional plugin-types: [PrePlanning](#preplanning) and [PostPlanning](#postplanning) plugins.
These plugins will be loaded and executed by the [GppPluing](../gpp_plugin).

## PrePlanning

The pre-planning plugins will be executed before the global-planners are run.
They should modify the input to the planning plugins.
The pre-planning plugins are allowed to
- alter the start and goal poses,

This allows the you to implement for example a common tolerance function for all planners, or to update the global_costmap only when required.

## PostPlanning

The post-planning plugins are run after the global planners.
The should modify the output of the planning plugins.
The post-planning plugins are allowed to
- alter the path,
- alter the cost.

Here, the user may implement common post-processing steps like
- path-pruning or densifying,
- trajectory smoothing,
- trajectory oscillation checks,
- feasibility checks