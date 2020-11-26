# GlobalPlannerPipeline

## Idea

This package allows the user to load an arbitrary number of `pre_planning` and
`post_planning` plugins and will run those plugins before and after the main planning step.
This allows the user to separate and reuse auxiliary functions for planning form the
planner implementation itself.

## Usage

The `gpp_plugin` will load three groups of plugins: `pre_planning`, `planning` and `post_planning`.
The plugins within each group are called sequentially.
The result from one plugin is passed to its successor.

The `gpp_plugin` will first run all `pre_planning` plugins, 
The `pre_planning` plugins may modify the input, 
- altering the start and goal poses,
- or performing operations on the costmap.

The `pre_planning` plugins may implement
- tolerance adjustments to the goal pose,
- feasibility checks on start and goal poses,
- costmap force-updates,
- etc.

The modified start and goal poses are then passed to the `planning` plugins.
The user may define an arbitrary number of `planning` plugins - but must provide at least one.
The `gpp_plugin` will invoke all `planning` plugins, passing the output
from one planner to its successor.
This allows to create a "planner-chain" - a feature commonly used in the [moveit](https://moveit.ros.org/) framework,
buy not supported by any global planner today.
However, there are multi-step navigation planners like [path_optimizer](https://github.com/LiJiangnanBit/path_optimizer), which may benefit from this feature.

The output from the planning-step is passed to the `post_planning` plugins.
These plugins take the planner output as input and may modify it.

At this step the user may implement
- path-pruning or densifying,
- trajectory smoothing,
- trajectory oscillation checks,
- etc.

The `gpp_plugin` offers no possibility to recover from a plugin-failure;
This means the entire execution fails, if one plugin (inside `pre_planning`, `planning` or `post_planning`) fails.
Keep this in mind, when designing and implementing your pre- and post-planning plugins.

## Parameters

The parameter interface follows the "standard" ros interface for loading plugins.
The user can define the plugins under the tags `pre_planning`, `planning` and `post_planning`.
Every group must be defined as an array;
Every array element must have the tags `name` and `type`.
Additionally the user may provide a metric tolerance.
This parameter not used if the `gpp_plugin` is used with [move_base_flex](https://github.com/magazino/move_base_flex) - 
(`move_base_flex` provides a tolerance as an argument to makePlan).

```yaml

# your planner definition
# we name our planner gpp
planner:
    - {name: gpp, type: gpp_plugin::GlobalPlannerPipeline}

# now the specification for the gpp_plugin
gpp:
    # not required if run from move_base_flex
    tolerance: 0.1

    # define the pre-planning plugins
    pre_planning:
    -  {name: first_pre_planning_name, type: first_pre_planning_type}
    -  {name: second_pre_planning_name, type: second_pre_planning_type}

    # define the planning plugins
    planning:
    -  {name: first_planning_name, type: first_planning_type}
    -  {name: second_planning_name, type: second_planning_type}

    # define the post-planning plugins
    post_planning:
    -  {name: first_post_planning_name, type: first_post_planning_type}
    -  {name: second_post_planning_name, type: second_post_planning_type}

```

## Build
