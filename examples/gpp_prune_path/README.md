# GppPrunePath

The GppPrunePath is a demo plugin, which reduces the number of elements within a path produced by a global planner.

## Behavior

The input path must contain at least two poses.
If the path is shorter, the plugin will fail.
The output will always contain the first and the last pose of the input path.
The intermediate poses will be pruned.

## Config

### ~\<name>\/step (int, 2)

Defined how many poses to skip.
Setting this value to N means that your output will contain every Nth value.
Setting this value to 3 for example, will skip two poses.
An input sequence of [1, 2, 3, 4, 5, 6, 7, 8...] would become [1, 4, 7, ...].
Values smaller than 2 will deactivate the pruning.

Below an example how to configure this plugin

```yaml
# this is for move-base-flex.
# activate the GppPlugin
plugins:
    - {name: gpp_plugin, type: gpp_plugin::GppPlugin}

# configure the gpp_plugin to use gpp_prune_path
gpp_plugin:
    # define the pre_planning and planning groups...
    # define the post_planning group:
    post_planning:
        - {name: gpp_prune_path, type: gpp_prune_path::GppPrunePath, on_failure_break: true}
    post_planning_default_value: True

# configure the gpp_prune_path
gpp_prune_path:
    step: 3
```
