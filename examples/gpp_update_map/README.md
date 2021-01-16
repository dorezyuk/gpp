# GppUpdateMap

This package will update the map as a pre-planning action.
This allows you to implement "lazy" map-updates - re-rendering your global-costmap only when its required (before your planning).

## Behavior

This simple plugin only fails, if its `preProcess` function has been called before initializing it with a valid `costmap_2d::Costmap2DROS`.

## Config

The package does not require any configuration.
In order to activate it, add following into your GppPlugin-config:

```yaml

# the gpp-plugin definition in your move-base-flex
planners:
    -{ name: gpp, type: gpp_plugin::GppPlugin }

gpp:
    pre_planning:
        - {name: gpp_update_map, type: gpp_update_map::GppUpdateMap, on_failure_break: false}
    pre_planning_default_value: true
    # ... now continue with your planning and post_planning definitions

# since we are always updating the costmap before plannig we may reduce the global-costmap update-frequency. You cannot set the update-frequency to zero however, until https://github.com/ros-planning/navigation/pull/1072 is released.
global_costmap:
    update_frequency: 0.5
```