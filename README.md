# petri_net

Documentation can be found here: http://pnm.dondrup.net/

## Usage

The following descirbes a minimal example of how to use the Petri-Net framework. The benefits
of this framework will only really become apparent when the examples becom a little
more complex. For now, we will focus on the minal version of the system that shows some of its
functionality.

### Actions

This example will show case three different action types that are commonly used.

**ROS Action Servers**
The action used for both version of the ROS action server described here will be:

```
# goal
int32 value
---
# result
int32 result
---
# feedback
```

So the action has an integer called `value` as a goal parameter and the result contains
a int called `result`. There is no feedback. The message can be found [here](rpn_recipe_planner_msgs/action/SimpleTest.action).

The simplest method of using the Petri-Net framework is by just using vanilla ROS ActionServer nodes. A simple example used here can be found 
in the [simple_test.py](rpn_recipe_planner/scripts/example_servers/simple_test.py) and looks like this:

```python
import rospy
from actionlib import SimpleActionServer
from rpn_recipe_planner_msgs.msg import SimpleTestAction, SimpleTestResult


class SimpleTestServer(object):
    def __init__(self, name):
        self._ps = SimpleActionServer(
            name,
            SimpleTestAction,
            execute_cb=self.execute_cb,
            auto_start=False
        )
        self._ps.register_preempt_callback(self.preempt_cb)
        self._ps.start()

    def execute_cb(self, goal):
        print goal
        print "Got the value:", goal.value
        r = goal.value+1
        print "Increasing value by 1 and returning it as result"
        self._ps.set_succeeded(SimpleTestResult(r))

    def preempt_cb(self, *args):
        print "Nothing to preempt"


if __name__ == "__main__":
    rospy.init_node("test_server")
    s = SimpleTestServer(rospy.get_name())
    rospy.spin()
```

As you can see, the only thing this server does is taking in an int value using our `SimpleTest.action`, adding 1, and then returning the result in the 
`SimpleTest.action`. This is just a place holder for whatever action you might require to be fulfilled. In the finished net, this server will be started
and the net will wait until it is finished before continueing. It receeives input at start and produces output at the end. There is a second kind of server, 
however, in case your action server needs to communicate with an external knowledgebase.

**RPN Action Servers**

RPN is short for ROS Petri-Net. These servers provide functionality for the server to communicate with the Petri-Net at run time. this is useful if, for example,
you need to query an external knowledgebase such as an ontology or the user of an interactive system via dialogue. This global data base or exeternal knowledge 
base is shared between all petri-nets and can therefore also be used to exchange information between different subsystems. More on this later.

The [RPN server used here](rpn_recipe_planner/scripts/example_servers/test.py) uses the same action as the other server above and looks like this:

```python
import rospy
from rpn_recipe_planner_msgs.msg import SimpleTestAction, SimpleTestResult
from rpn_action_servers.rpn_simple_action_server import RPNSimpleActionServer
import json


class TestServer(object):
    def __init__(self, name):
        self._ps = RPNSimpleActionServer(
            name,
            SimpleTestAction,
            execute_cb=self.execute_cb,
            auto_start=False
        )
        self._ps.register_preempt_callback(self.preempt_cb)
        self._ps.start()

    def execute_cb(self, goal):
        print goal
        print "started"
        rospy.sleep(1.)
        print "TEST SERVER", "Saving value to external KB"
        print "TEST SERVER", self._ps.update_kb(type=RPNSimpleActionServer.UPDATE_REMOTE, attr="my_value", value=json.dumps(goal.value))
        rospy.sleep(1.)
        print "TEST SERVER", "Getting value from external KB"
        r = self._ps.query_kb(type=RPNSimpleActionServer.QUERY_REMOTE, attr="my_value")
        print "TEST SERVER", type(r), r
        r = json.loads(r.value)
        rospy.sleep(1.)
        print "TEST SERVER", "ended"
        #  Passing the value of the goal to the result
        self._ps.set_succeeded(SimpleTestResult(r))

    def preempt_cb(self, *args):
        pass


if __name__ == "__main__":
    rospy.init_node("rpn_test_server")
    t = TestServer(rospy.get_name())
    rospy.spin()
```

As you can see, instead of using a ROS `ActionServer` like above, we are using a `RPNSimpleActionServer`.These inherit from the corresponding ROS version, i.e.
`SimpleActionServer` and `ActionServer`, and offer some added funtionality. In principle, the server does nothing but take in the `value` and return the same 
thing as the `result` or the `SimpleTest.action` shown above. In between it show cases the special functionality of RPN servers which is updating and querying
the knowledge bases. For this purpose the RPN Servers provide `update_kb(type, attr, value, meta_info)` and `query_kb(type, attr, meta_info)`. Using ROS
services, these methods communicate with the both the local and global/external knowledge base. More on these later.

*Updating the KB* works by using something like this:

```python
self._ps.update_kb(type=RPNSimpleActionServer.UPDATE_REMOTE, attr="my_value", value=json.dumps(goal.value))
```

the `type` can be `UPDATE_` followed by either `REMOTE` (global KB), `LOCAL` (local KB), `ALL` (both KBs) which are constants provided by `RPNSimpleActionServer`.
The `attr` is the name of the field your value should be stored in. The `value` is the value you want store. This has to be of type str which is why json
is used here. The last argument (not shown here) is called `meta_info` and can be any kind of string. this can be used to pass along additional information
specific to your external KB
