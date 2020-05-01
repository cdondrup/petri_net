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
specific to your external KB.

*Querying the KB* works similar to updating it.

```python
r = self._ps.query_kb(type=RPNSimpleActionServer.QUERY_REMOTE, attr="my_value")
```

Again the `meta_info` parameter is not shown. This line of code queries the remote KB for something called `my_value`. It the returns a [`RPNQuery`](ros_petri_net_msgs/srv/RPNQuery.srv)
object. Using the `value` field you can get a string representation of the result. This can then be loaded using json to get the actual data structure.

**Knowledge Base Action**

The thrid type of action possible and used here is the KB action. This action work directly on the knowledge base. So actually we wouldn't have needed the server
described above to store out value in the external KB but we could have done that directly using a KB action. Why is this beneficial? Because we don't need to
implement a dedicted ROS node for this but can define these directly in the domain and plan. This action will be explained in more detail later when we have 
a look at the plan and domain files.

### Knowledge Bases

Having talked about them a little already, it is time to introduce the concept of the local and global/external knowledge base. Sadly, the naming convention 
hasn't been adhered to so the global KB is sometimes also called external. Both are the same thing though.

*Local KB* this KB is meant to automatically save the results and populate the goals of action servers. So in our example the `value` that is put into the action goal whe  either of the servers above is started comes from the local KB. Once the servers are finished, the `result` is saved in the local KB.
All this happens automaticall and the user doesn't need to concern themselves with that. This is meant to allow to pass information between different actions automatically. So if the goal parameter of an action server is called the same as the resul parameter of a different action server, then the information would be passed along automatically.
A little example, imagine you have a server `A` and a server `B`. Server `A` produces a random integer and returns it in its result message as `rnd_int`. Now server `B` is supposed to check if this random integer is odd or even. in order to do that, the goal message of server `B` has a field called `rnd_int`. Since both the output of server `A` and the input of server `B` are called the same, the local KB will take care of saving the output from `A` and passing it to the goal of `B` when it is started. Hence, the user does not need to concern themselves with having to pass the value between the servers but only needs to make sure that both parameters are called the same.

This local KB only exists during the runtime of the perti-net it is associated with. It can be initilised with prior knowledge. More on this later.

*Global KB* The global or external KB can be used for project specific purposes such as communication with an ontology, a dialogue system, etc. It can also be used to pass values between different actions in different petri nets or between different execution cycles of the same petri net. In contrast to the local KB, the global KB lives for the entire runtime of the Petri-Net Server. So if data is supposed to be stored persistently between runs of single nets, this can be used.

The most basic implementation can be found [here](rpn_recipe_planner/src/rpn_planner_kb/rpn_planner_knowledge_base.py) and looks like this:

```python
from pnp_kb.external_knowledge_base import ExternalKnowledgeBase


class RPKnowledgeBase(ExternalKnowledgeBase):
    def __init__(self, net_id):
        super(RPKnowledgeBase, self).__init__(net_id)

    def query(self, variable, meta_info=None):
        a = getattr(self, variable)
        print "+++ QUERY +++", variable, a
        return a

    def update(self, variable, value, meta_info=None):
        print "+++ UPDATE +++", variable, value
        setattr(self, variable, value)

```

It is kept as simple as possible by extending `pnp_kb.external_knowledge_base` which provides all the technical funsitonality for communication etc. The user only has to implement two methods: query and update. You can see an example implementation above that simple staroes any value that comes in in update as a memeber variable and then retrieves and returns it in the query method. The methods are called via the functionality of the RPN Action Servers described above and should be relatively self-explanatory.

### Petri-Net Definition

**Domains**

Petri-Nets are based on Plans and Domains. This is similar to PDDL style languages. An example domain can be found [here](rpn_recipe_planner/etc/domains/domain.yaml) and looks like this:

```yaml
instances: &instances
    - 
external_knowledge_base:
    module: "rpn_planner_kb.rpn_planner_knowledge_base"
    class:  "RPKnowledgeBase"
action_types:
    BaseAction: &base_action
        instances: *instances
    ROSAction: &ros_action
        <<: *base_action
        type:
            module: "rpn_ros_interface.action"
            class:  "ROSAtomicAction"
    RPNAction: &rpn_action
        <<: *base_action
        type:
            module: "rpn_ros_interface.rpn_action"
            class:  "RPNAtomicAction"
    KBAction: &kb_action
        <<: *base_action
        type:
            module: "pnp_actions.kb_action"
            class:  "KBAction"
actions:
    rpn_test_server:
        <<: *rpn_action
        params:
            - "value"
        preconditions:
            and:
                - Exists: [LocalQuery: "value"]
        effects:
            and:
                Exists: [LocalQuery: "result"]
    test_server:
        <<: *ros_action
        params:
          - "value"
        preconditions:
          and:
              - Exists: [LocalQuery: "value"]
    result_to_value:
        <<: *kb_action
        params:
            - "operation"

```
