# GppPlugin

As you may already know (see the [concept](../README.md#concept)), we call all plugins loaded by the GppPlugin as its *child-plugins* and group those child-plugins in three *groups* (pre-, global-, and post-planning groups).

Now its time to understand how each group is executed.

```python
for plugin in plugins:
    success = plugin.run()
    if(!success && plugin.on_failure_break):
        return False
    elif(success && plugin.on_success_break):
        return True
return default_value
```

Above is a pythonic pseudo-code, showing the loop for each plugin-group.
The "plugin" above has three attributes - the method `run` in line 2, which is just pseudo-code for executing the plugin.
Additionally it has the members `on_success_break` and `on_failure_break` (lines 3 and 5).
These members are specific to every child-plugin and allow you to customize the behaviour of your pipeline.
You may have "optional" child-plugins or might be satisfied if only one child-plugins returns a successful result.   
Finally there is a third unknown value: `default_value` at the last line.
This boolean value allows you to specify for each group what should happen, if the for-loop runs through without "preemptive" termination.

## Behavior-Tree-Relation

You may set all the `on_failure_break`, `on_success_break` and `default_value` freely.
Nonetheless, it's maybe important to highlight that you can implement sequence and selector nodes known from [behavior-trees](https://en.wikipedia.org/wiki/Behavior_tree_(artificial_intelligence,_robotics_and_control)) with the mentioned parameters.

If all child-plugins within one group have `on_success_break` set to true, `on_failure_break`. set to false and the `default_value` set to true, you will get a sequence-node.
If you invert the values, the group will behave as a selector-node.

<center>

| flag             | sequence | selector |
|------------------|----------|----------|
| on_failure_break | true     | false    |
| on_success_break | false    | true     |
| default_value    | true     | false    |

</center>

## Parameters

Now its time to look at how to define and load child-plugins and groups.
The groups begin with the tags `pre_planning`, `planning` and `post_planning`.
The child-plugins for those groups must be defined as a list.
The execution order within this list defines laster on the order of the execution.
Every child-plugin in these lists must have a `type` and `name` tag.
Additionally you can pass boolean `on_failure_break` and `on_success_break`.
Finally you may provide a boolean `default_value` for each group.

Below an example

```yaml
pre_planning:
    - {name: foo, type: bar, on_failure_break: True, on_success_break: False}
    - {name: baz, type: bum, on_failure_break: True, on_success_break: False}

    default_value: True
```

We load in our pre-planning group two child-plugins with the names foo and baz.
The `on_failure_break`, `on_success_break` and `default_value` are chosen such that the groups acts as a sequence node.

Below the detailed documentation.
The `<pre_planning|planning|post_planning>` tag means here, that you can define the parameter under each of those groups.

### ~\<name>\/<pre_planning|planning|post_planning> (list)

List, as defined above.

### ~\<name>\/<pre_planning|planning|post_planning>\/name (string)

Name of the plugin. The name will be passed to the child-plugin.

### ~\<name>\/<pre_planning|planning|post_planning>\/type (string)

Type of the plugin.
For the pre_planning group it must be resolvable to a plugin implementing the `gpp_interface::PrePlanningInterface`.
For the planning group must be resolvable to a plugin implementing either `nav_core::BaseGlobalPlanner` or `mbf_costmap_core::CostmapPlanner`.
For the post_planning group it must be resolvable to a plugin implementing the `gpp_interface::PostPlanningInterface`.

### ~\<name>\/<pre_planning|planning|post_planning>\/on_failure_break (bool, true)

Value defining if the group should break its execution if this child-plugin fails.

### ~\<name>\/<pre_planning|planning|post_planning>\/on_succuss_break (bool, false)

Value defining if the group should break its execution if this child-plugin succeeds.

### ~\<name>\/<pre_planning|planning|post_planning>\/default_value (bool, true)

Default outcome of the group.
