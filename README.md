# petri_net

Documentation can be found here: http://pnm.dondrup.net/

## Usage

The following descirbes a minimal example of how to use the Petri-Net framework. The benefits
of this framework will only really become apparent when the examples becom a little
more complex. For now though, we will focus on the minimal version of the system that shows some of its
functionality.

### Actions

This example will showcase three different action types that are commonly used.

**ROS Action Servers**
The action used for both version of the ROS action server described here is the `SimpleTest.action`:

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
an int called `result`. There is no feedback. The message can be found [here](rpn_recipe_planner_msgs/action/SimpleTest.action).

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

As you can see, the only thing this server does is taking in an int `value` using our `SimpleTest.action`, adding 1, and then returning the `result` in the 
`SimpleTest.action`. This is just a place holder for whatever action you might require to be fulfilled. In the finished net, this server will be started
and the net will wait until it is finished before continueing. It receeives input at start and produces output at the end. There is a second kind of server, 
however, in case your action server needs to communicate with an external knowledgebase.

**RPN Action Servers**

RPN is short for ROS Petri-Net. These servers provide functionality for the server to communicate with the Petri-Net at run time. This is useful if, for example,
you need to query an external knowledgebase such as an ontology or the user of an interactive system via dialogue. This global data base or external knowledge 
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
thing as the `result` of the `SimpleTest.action` shown above. In between it showcases the special functionality of RPN servers which is updating and querying
the knowledge bases. For this purpose the RPN Servers provide `update_kb(type, attr, value, meta_info)` and `query_kb(type, attr, meta_info)`. Using ROS
services, these methods communicate with the both the local and global/external knowledge base. More on these later.

*Updating the KB* works by using something like this:

```python
self._ps.update_kb(type=RPNSimpleActionServer.UPDATE_REMOTE, attr="my_value", value=json.dumps(goal.value))
```

the `type` can be `UPDATE_` followed by either `REMOTE` (global KB), `LOCAL` (local KB), `ALL` (both KBs) which are constants provided by `RPNSimpleActionServer`.
The `attr` is the name of the field your value should be stored in, i.e. `my_value` in this case. The `value` is the data you want store. This has to be of type str which is why json
is used here. The last and optional argument (not shown here) is called `meta_info` and can be any kind of string. This can be used to pass along additional information specific to your external KB.

*Querying the KB* works similar to updating it.

```python
r = self._ps.query_kb(type=RPNSimpleActionServer.QUERY_REMOTE, attr="my_value")
```

Again the `meta_info` parameter is not shown. This line of code queries the remote KB for something called `my_value`. It the returns a [`RPNQuery`](ros_petri_net_msgs/srv/RPNQuery.srv)
object. Using the `value` field you can get a string representation of the result. This can then be loaded using json to get the actual data structure.

**Knowledge Base Action**

The thrid type of action possible and used here is the KB action. This action works directly on the knowledge base. So actually we wouldn't have needed the server
described above to store our `my_value` in the external KB but we could have done that directly using a KB action. Why is this beneficial? Because we don't need to
implement a dedicted ROS node for this but can define these directly in the domain and plan. This action will be explained in more detail later when we have 
a look at the plan and domain files.

### Knowledge Bases

Having talked about them a little already, it is time to introduce the concept of the local and global/external knowledge base. Sadly, the naming convention 
hasn't been adhered to so the global KB is sometimes also called external. Both are the same thing though.

*Local KB* this KB is meant to automatically save the results and populate the goals of action servers. So in our example the `value` that is put into the action goal when either of the servers above is started comes from the local KB. Once the servers are finished, the `result` is saved in the local KB.
All this happens automaticaly and the user doesn't need to concern themselves with that. This is meant to allow to pass information between different actions automaticaly. So if the goal parameter of an action server is called the same as the result parameter of a different action server, then the information would be passed along by the system.
A little example, imagine you have a server `A` and a server `B`. Server `A` produces a random integer and returns it in its result message as `rnd_int`. Now server `B` is supposed to check if this random integer is odd or even. In order to do that, the goal message of server `B` has a field called `rnd_int`. Since both the output of server `A` and the input of server `B` are called the same, the local KB will take care of saving the output from `A` and passing it to the goal of `B` when it is started. Hence, the user does not need to concern themselves with having to pass the value between the servers but only needs to make sure that both parameters are called the same.

This local KB only exists during the runtime of the perti-net it is associated with. It can be initialised with prior knowledge. More on this later.

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

It is kept as simple as possible by extending `pnp_kb.external_knowledge_base` which provides all the technical functionality for communication etc. The user only has to implement two methods: `query` and `update`. You can see an example implementation above that simply stores any value that comes in in `update` as a member variable and then retrieves and returns it in the `query` method. The methods are called via the functionality of the RPN Action Servers described above and should be relatively self-explanatory.

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

Let's have a look at this YAML file in a bit more detail. The first statement about `instances` is depricated and not used anymore. It is there for legacy reasons and should just be draged along for now. The next part defines which external KB to use:

```yaml
external_knowledge_base:
    module: "rpn_planner_kb.rpn_planner_knowledge_base"
    class:  "RPKnowledgeBase"
```

This links to the KB described above and should be self-explanatory.

The next part describes the actions that can be used in this domain. These are not the actual actions that are described above but Petri-Net internal action definitions. These can be used in the Petri-Net and will then call either a ROS action server or a RPN action server. If you would like to know more about atomic actions, please have a look at the documentation linked at the top of this file.
The main thing to notice here is they define the three key words `ros_action`, `rpn_action`, and `kb_action` that can be used to define what type of action you want to create.

```yaml
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
```

The last part called `actions` defines all the actual actions that can be performed by the net here we can find an action for each of the action servers we talked about above.

```yaml
    test_server:
        <<: *ros_action
        params:
          - "value"
        preconditions:
          and:
              - Exists: [LocalQuery: "value"]
```

This defines our action that is just based on the vanilla ROS SimpleActionServer described first. The server is called `test_server`. This name has to be the same as the name of the ROS node. And indeed above, we defined `rospy.init_node("test_server")`. This is how action servers are identified. By their unique name.
We tell the Petri-Net that this is a ROS action server using `<<: *ros_action`. Next we define the parameters that this server expects in it's goal. According to our `Simpletest.action`, we only have `value`. So we only define this:

```yaml
        params:
          - "value"
```

In reality, the params can be omitted completly and it would still work. It does, however, increase readability. Next follow optionl preconditions and effects. These are used to determine if the action can be executed (preconditions) and if it has to be executed/was successfully executed (effect). If the preconditions are not met, the net will fail as it cannot execute the action. If the effects are already true before the action has been started, the action will be skiped. If the effects are not true after the action has finished, the action will be executed again. These are standard behaviours that can be customised but this is a bit beyond the scope of this simple tutorial. The action here only defines preconditions:

```yaml
        preconditions:
          and:
              - Exists: [LocalQuery: "value"]
```

Preconditions and effects, support all standard logical operations, i.e. and, or, not, and a range of comparisons and checks such as the `Exists` used here. So before the action will be started, the Petri-Net will query the local KB and check if there is an entry called "value". This makes sure that we can fill the goal of our `SimpleTest.action` defined above.

In additiona to our vaniall ROS action, we also have our RPN action that we defined above:

```yaml
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
```

This is called `rpn_test_server` which must again be the same as the name of the ROS node it is supposed to represent. It is defined as an `<<: *rpn_action` but otherwise works exactly the same as above. The main difference is that it also defines an effect to show how this works.

```yaml
        effects:
            and:
                Exists: [LocalQuery: "result"]
```

Effects work the same and support the same operations as preconditions. The main difference is that effects are checked before the start of an action and after the action finishes. So in this case if the field `result` already exists in our local KB before the action is started, then we can assume that the effect has already happened and, therefore, we do not need to run the action at all. If it does not exists yet, then the action is executed. If after the action finishes, the field `result` does still not exist in the local KB, the action is restarted in the hope that running it another time will work better. Again, this might not be the smartest contingency plan but these can be mnaually defined if necessary. More on this in a later tutorial.

For our example, we know that the field `result` will exist after the action has been executed because our `SimpleTest.action` has this field and our implementation of the server returns a result. Btw. just because `SimpleTest.action` has a field called `result` does not mean that we will always have this entry in the local KB. If the server fails, the result will not be returned and therefore, the entry will not be made. Hence, restarting the server might fix this.

Finally, we have our KB action:

```yaml
    result_to_value:
        <<: *kb_action
        params:
            - "operation"
```

From the name, we can alreaady imagine that it will take what ever is in `result` and save it to `value`. How it does that is defined in the plan (see below). We could define preconditions and effects for this as well but have chosen not to for the sake of space. It only has one parameter called `operation` which we will see below.

**Plans**

Now that we have defined all the actions we can use, we have to put them in some form of sequence. This plan could be produced by a proper planner but for now we will just hand craft it. The plan can be found [here](rpn_recipe_planner/etc/plans/example.yaml) and looks like this:

```yaml
example_plan:
    server:
        module: "actionlib"
        class:  "ActionServer"
    action:
        module: "rpn_recipe_planner_msgs.msg"
        class: "RosServerAction"
    initial_knowledge:
        value: 5
    plan:
        - rpn_test_server: {}
        - while:
            condition: {Comparison: ["ne", [LocalQuery: "value", 10]]}
            actions:
                - test_server: {}
                - result_to_value:
                    operation:
                        LocalUpdate: ["value", {LocalQuery: "result"}, ""]

```

The name of this plan is `example_plan` and will become important later on because this plan will be turned into a ROS action server for execution. The actual type of this server and the goal it takes are defined here:

```yaml
    server:
        module: "actionlib"
        class:  "ActionServer"
    action:
        module: "rpn_recipe_planner_msgs.msg"
        class: "RosServerAction"
```

The server type is stright forward. The action file used looks like this:

```yaml
string id
string params
---
---
```

The goal requires some form of `id` and `params`. The id is simply a unique string (use uuid4 for example) and the `params` is the json representation of a `dict`. This `dict` is used to provide the local KB with inital knowledge. So you can pass in values that you want to give your servers for execution. Sadly, for now, you have to use this goal type. I am currently looking into accepting generic goals but this is ongoing work.

So once the plan has been parsed and turned into a Petri-Net, the framework will automaticaly create a ROS Action Server with a goal of type `RosServerGoal` called `example_plan`. When this server is started, the Petri-Net is executed.

The statement

```yaml
    initial_knowledge:
        value: 5
```

is yet another way to provide the local KB with inital data in addition to the `param` of the action goal mentioned above. This is to hardcode common knowledge. Here we say that this net should start with a `value=5`.

The plan of the net requires some explanation as well. It showcases the genral structure and how to use loops:

```yaml
    plan:
        - rpn_test_server: {}
        - while:
            condition: {Comparison: ["ne", [LocalQuery: "value", 10]]}
            actions:
                - test_server: {}
                - result_to_value:
                    operation:
                        LocalUpdate: ["value", {LocalQuery: "result"}, ""]
```

All the actions in the list are executed sequentially. There is a way of executing them concurrently but this is not part of this tutorial. Just to give you a taste, it is as easy as just defining a list of actions called `concurrent_actions`. All the actions in that list will then be executed concurrently. That's the beauty of Petri-Nets. Back to our example though, the first statement: `rpn_test_server: {}` simple means "start the action called `rpn_test_server` and don't give it any special parameters". The `{}` could be used to hard code special parameters. So if we wanted to ignore the local KB we could do something like this: `rpn_test_server: {value: 1}`. That would mean that our action uses `1` as the `value` instead of `5`. Since we don't define anything though it just uses the local KB so `value=5`. After this action has finished, the while loop is executed:

```yaml
        - while:
            condition: {Comparison: ["ne", [LocalQuery: "value", 10]]}
            actions:
                - test_server: {}
                - result_to_value:
                    operation:
                        LocalUpdate: ["value", {LocalQuery: "result"}, ""]
```

A while loop defines a condition that has to be true in oder for the loop to start and is checked every iteration. Like a proper while loop. It also defines a sequence of actions which are executed in order. Our condition is `condition: {Comparison: ["ne", [LocalQuery: "value", 10]]}` which translates to: "Check the local KB for `value` and if the result is not equal (`ne`) to 10, then return `true`" (comparisons are taken directly from the [python operator package](https://docs.python.org/2.7/library/operator.html)). As long as this condition is true, we execute our two actions inside. The first action is `test_server: {}` which takes in a `value` adds 1 and returns the `result`. So on the firt iteration it gets `value=5` and returns `result=6` which is automatically put into the local KB. The next action is our KB action:

```yaml
                - result_to_value:
                    operation:
                        LocalUpdate: ["value", {LocalQuery: "result"}, ""]
```

This action performes a local update: `LocalUpdate: ["value", {LocalQuery: "result"}, ""]`. This translates to: "Query the local KB for `result` and save the return value in the local KB as `value`". So it basically saves `result` to `value` -> `result=value`. Hence the next loop iteration `value=6` and I am sure you can see where this is going. So all we do here is basically counting from 5 to 10. I know, not very impressive when you think about it but this is just a place holder for whatever operation you would like to perform.

**Worlds**

There is one final file missing, the `worlds.yaml` file found [here](rpn_recipe_planner/etc/worlds.yaml). This file simply tells the programme where to find the domain file and the plans:

```yaml
example:
    domain: domains/domain.yaml
    recipes:
        - plans/example.yaml
```

Our world here is called `example` and it has exactly one plan or recipe as we call it here. Each world can have one domain file which defines all the possible actions in the world and several recipes or plans that can all be loaded at the same time. Each of these recipes will be turned into a separate ROS action server for execution. Here we only have one which is our plane discussed above.

### Petri-Net Execution

Now that we have defined everything we need all we do is run things. First we need to start the petri-net execution framework:

```
roslaunch rpn_recipe_planner recipe_planner.launch world:=example
```

The `world` parameter tells the framework which plans to load and should be an entry in the `worlds.yaml` file. Once we have done so, checking the rostopics tells us that the system has created our `example_plan` action server for us:

```
/example_plan/cancel
/example_plan/feedback
/example_plan/goal
/example_plan/result
/example_plan/status
```

Now all we need to do is start our two action servers that we created in the beginning:

```
rosrun rpn_recipe_planner simple_test.py
rosrun rpn_recipe_planner test.py
```

Now everything is ready to run and we can just start the action server of our plan directly using:

```
$ rostopic pub /example_plan/goal rpn_recipe_planner_msgs/RosServerActionGoal "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
goal_id:
  stamp:
    secs: 0
    nsecs: 0
  id: ''
goal:
  id: 'my_test_net'
  params: ''" -1
```

The `id` here is set to `my_test_net`. This is just a name and can be whatever you like apart from the empty string as this name is used to create a host of topics and services for internal communication. The name has to be unique. A video of the execution can be found here: https://youtu.be/MouAW1H-CaQ.

The execution of the net is almost instantaneously the only thing that takes time is the very first action server as it has several sleep statements in it.


## Using a controller

The Petri-Net can be used with an external controller that starts tasks and keeps track of them. This is interesting when thinking about a system that would be able to start tasks. This could be a dialogue system or an external scheduler. The Petri-Net repository contains an abstract controller that you can base your conterller on. There is also a basic example controller that we will have a look at here to show how this could work.

### The controller

The `AbstractController` which can be found [here](rpn_controller/src/rpn_controller/abstract_controller.py) contains the functionality required to create your own controller. Documentation can be found following the link in the top of this document. The main functionality of the controller is to manage the currently available and running Petri-Nets and to provide functionality to send and recieve information from/to a specific net. For taking care of the running Petri-Nets, the controller provides information on the available nets. To this end, the abstract controller works in conjunction with a specialised action server which will be descrined below. The other major functionality of the server is to exchange data. For this the thing to do is to override the abstract `query_callback` and `update_callback` methods. These methods are called if a component wants to update the controller or requires information that the controller can provide.

Let's have a look at the example controller which can be found [here](rpn_controller/scripts/example_controller.py):

```python
import rospy
from rpn_controller.abstract_controller import AbstractController
from rpn_controller_msgs.srv import ControllerQuery, ControllerQueryResponse
from rpn_controller_msgs.srv import ControllerUpdate, ControllerUpdateResponse
from uuid import uuid4


class ExampleController(AbstractController):
    def __init__(self, name):
        super(ExampleController, self).__init__(name)

    def spin(self):
        print "The controller has started"
        while not rospy.is_shutdown():
            if self.get_num_registered_servers() > 0:
                print "The available actions are:", self.get_registered_server_names()
                server = raw_input("Start server> ")
                self.get_server_client(server).wait_for_server()
                goal = self.get_server_goal(server)
                goal.id = str(uuid4())
                self.get_server_client(server).send_goal(goal)
            rospy.sleep(3.)

    def query_callback(self, req):
        print "QUERY", req
        # Do something fancy here to generate the data such as asking the user about their opinion etc.
        result = "my awesome resut"
        return ControllerQueryResponse(result)

    def update_callback(self, req):
        print "UPDATE", req
        return ControllerUpdateResponse()

```

This example shows a very simple implementation of such a controller. The query and update methods simply print the service requests sent and the query just always returns the same hard coded string. These methods are very specific to the actual use case of your system so the their functionality is left to the user to implement. As mentioned above, the servers register them selves with the controller so the controller knows which servers are available. The currently registered severs can be accessed via their names. To retieve the names of all the registered severs you can use the `get_registered_server_names()` method which returnes a list of names. With `get_num_registered_servers()` you can get the length of this list. 

**Starting a server**

The main functionality of a controller is of course controlling the servers. The way in wich the servers are controlled here is using the ROS action server methods. The `get_server_client(server)` returns a ROS action client where the `server` argument is the name of the server form the list obtained via the `get_registered_server_names()`. This action client supports all the standard functionality of a ROS action client.

In order to create the goal to send to the server, you can use the `get_server_goal(server)` which returns an object of the type of the goal required by the server in question. Again `server` is the name of the server you would like to get the goal for and should be the same as the one you use to call `get_server_client(server)`. If you require only the type of the goal but not the actual goal object, you can use the `get_server_goal_type(server)` method.

Hence, the easiest way of starting a server is:

```python
                self.get_server_client(server).wait_for_server()
                goal = self.get_server_goal(server)
                goal.id = str(uuid4())
                self.get_server_client(server).send_goal(goal)
```

### The controller plugin server

In oder to provide all the functionality the controller provides, we need to use a specific type of ROS action server that automatically registers itself with the controller and provides functionality to call the query and update methods using ROS services. This type of server can be found [here](rpn_controller/src/rpn_controller_servers/abstract_controller_plugin_server.py) and extends the ROS action server. The simple action server is not supported yet. 

```python
...
import utils as ut
from rpn_recipe_planner_msgs.msg import RosServerAction
from rpn_controller_msgs.srv import ControllerQuery, ControllerQueryRequest
from rpn_controller_msgs.srv import ControllerUpdate, ControllerUpdateRequest


class AbstractControllerPluginServer(actionlib.ActionServer, object):
    __metaclass__ = ABCMeta

    def __init__(self, ns, ActionSpec=RosServerAction, goal_cb=None, cancel_cb=actionlib.nop_cb, auto_start=True):
        if goal_cb is None:
            raise AttributeError("goal_cb has to be specified")
        self.ns = ns
        super(AbstractControllerPluginServer, self).__init__(
            ns=ns,
            ActionSpec=ActionSpec,
            goal_cb=goal_cb,
            cancel_cb=cancel_cb,
            auto_start=auto_start
        )

    @abstractproperty
    def controller_name(self):
        return

    def get_controller_name(self):
        if self.controller_name == "": return self.controller_name
        return self.controller_name if self.controller_name.startswith("/") else "/"+self.controller_name

    def start(self):
        super(AbstractControllerPluginServer, self).start()
        ut.register_client(self.ns, self.get_controller_name())
        rospy.on_shutdown(lambda: ut.unregister_client(self.ns, self.get_controller_name()))

    def __get_clean_ns(self):
        return self.ns[1:] if self.ns.startswith("/") else self.ns

    @property
    def query_service_name(self):
        return self.get_controller_name()+"/query" if self.controller_name != "" else ""

    @property
    def query_service_type(self):
        return ControllerQuery

    def generate_query_request(self, net_id, variable, meta_info):
        return ControllerQueryRequest(net_id, self.__get_clean_ns(), variable, meta_info)

    def query_controller(self, net_id, variable, meta_info={}):
        return ut.call_service(
            self.query_service_name,
            self.query_service_type,
            self.generate_query_request(net_id, variable, meta_info)
        )

    @property
    def update_service_name(self):
        return self.get_controller_name()+"/update" if self.controller_name != "" else ""

    @property
    def update_service_type(self):
        return ControllerUpdate

    def generate_update_request(self, net_id, variable, value, meta_info):
        a = self.__get_clean_ns()
        print self.ns, type(self.ns), a, type(a)
        return ControllerUpdateRequest(net_id, self.__get_clean_ns(), variable, value, meta_info)

    def update_controller(self, net_id, variable, value=None, meta_info={}):
        return ut.call_service(
            self.update_service_name,
            self.update_service_type,
            self.generate_update_request(net_id, variable, value, meta_info)
        )

```

This class is abstract and requires the user to override the `controller_name(self)` property. This should meerly return a strind with the ROS namespace of the controller. The two most important methods here are `query_controller(self, net_id, variable, meta_info={})` and `update_controller(self, net_id, variable, value=None, meta_info={})` which should be self-explenatory based on the explanation of knowledge bases above.

This server makes use of the methods in the `utils.py` file found [here](rpn_controller/src/rpn_controller_servers/utils.py).

**The basic server**

The simplest implementation of a server is the basic server [here]():

```python
import rospy
import actionlib
from rpn_recipe_planner_msgs.msg import RosServerAction
from rpn_controller_servers.abstract_controller_plugin_server import AbstractControllerPluginServer


class BasicControllerPluginServer(AbstractControllerPluginServer):
    def __init__(self, ns, controller_name="", ActionSpec=RosServerAction, goal_cb=None, cancel_cb=actionlib.nop_cb, auto_start=True):
        super(BasicControllerPluginServer, self).__init__(ns, ActionSpec, goal_cb, cancel_cb, auto_start)
        self.__cn = rospy.get_param("rpn_controller_name", controller_name)
        print "GOT CONTROLLER NAME AS:", self.__cn

    @property
    def controller_name(self):
        return self.__cn

```

All this implementation of the abstract server does is override the abstract property `controller_name` which will either return the name passed in to the constructor, read from the global ROS param `rpn_controller_name` or `""` if none of them has been specified. If `""` is returned the server will try to determine the name of the controller automatically. This is a convenience method that works well as long as all the ROS nodes run on the same PC. As soon as the nodes are distributed over a network, this method gets to slow for practical use. Hence, it is recommended to specify the controller name manually where possible.

This server can now be used like any other ROS action server so we will use it in the following example.

###The controller knowledge base

In order to communicate with the controller, we will a use a different knowledge base implementation. As we have seen above, the external knowledge base can be customised to fit the need of the users. Initially, we went with a very simple implementation that just stores values given a name. Now we want to be able to not just do that but to also communicate with the controller if we choose to do so. For this purpose, we created the `rpn_controller_knowledge_base.py` wich can be found [here](rpn_recipe_planner/src/rpn_controller_kb/rpn_controller_knowledge_base.py) and looks like this:

```python
from pnp_kb.external_knowledge_base import ExternalKnowledgeBase
import utils as ut
from rpn_recipe_planner_msgs.srv import RPQuery, RPQueryRequest
from rpn_recipe_planner_msgs.srv import RPUpdate, RPUpdateRequest
import json


class RPKnowledgeBase(ExternalKnowledgeBase):
    def __init__(self, net_id):
        super(RPKnowledgeBase, self).__init__(net_id)

    def query(self, variable, meta_info=None):
        try:
            a = getattr(self, variable)
            print "+++ QUERY +++", variable, a
            return a
        except (AttributeError, TypeError):
            if meta_info is None:
                meta_info = {}
            else:
                try:
                    meta_info = json.loads(meta_info)
                except:
                    pass

            return self.__controller_query(variable, meta_info)

    def __controller_query(self, variable, meta_info={}):
        print "+++ CONTROLLER QUERY +++", variable, meta_info
        if not isinstance(variable, (str, unicode)):
            try:
                variable = json.dumps(variable)
            except (AttributeError, TypeError) as e:
                print e
        meta_info = meta_info if isinstance(meta_info, (str, unicode)) else json.dumps(meta_info)

        r = ut.call_service(
            "/"+self.net_id.replace('-','_')+"/query",
            RPQuery,
            RPQueryRequest(
                variable=variable,
                meta_info=meta_info
            )
        )
        print "+++ CONTROLLER REPLY:", r
        try:
            return json.loads(r.result)
        except ValueError:
            return r.result

    def update(self, variable, value, meta_info={}):
        if variable == "CONTROLLER":
            print "+++ CONTROLLER UPDATE +++", variable, value, meta_info
            value = value if isinstance(value, (str, unicode)) else json.dumps(value)
            meta_info = meta_info if isinstance(meta_info, (str, unicode)) else json.dumps(meta_info)
            print type(value), type(meta_info)
            r = ut.call_service(
                "/"+self.net_id.replace('-','_')+"/update",
                RPUpdate,
                RPUpdateRequest(
                    value=value,
                    meta_info=meta_info
                )
            )
        else:
            setattr(self, variable, value)
            print "+++ UPDATE +++", variable, value

```

This implementation also extends the `ExternalKnowledgeBase` and provides the same functionality regarding storing and retrieval. In the query method, it tries to get and return the requested variable value and only if it can't will call the controller using: `__controller_query(self, variable, meta_info={})`. The magic block of code to achieve this is:

```python
        r = ut.call_service(
            "/"+self.net_id.replace('-','_')+"/query",
            RPQuery,
            RPQueryRequest(
                variable=variable,
                meta_info=meta_info
            )
        )
```

Let's have a look at that in a bit more detail. It uses a method from the `utils.py` file [here](rpn_recipe_planner/src/rpn_controller_kb/utils.py) called `call_service(srv_name, srv_type, req)` which take care of creating the service proxy, waiting for the service to become active and then calls it. If there is a exception, it will retry the service call after one second. The most interesting statement in the piece of code above is the generation of the service name. As you can see from this line: `"/"+self.net_id.replace('-','_')+"/query"` the service name is not fixed but depends on the `net_id` which is the unique identifyer given to any net on start up. This allows for the parallel execution of the same net several times because the controller can unambiuously identify the instance of the net and the action inside the net that tries to communicate with it.

Just imagine the following scenario: I start the net and query the controller. This query relies on user input and therefore effectively pauses the execution of the net until the user has provided the requrired data. In the meantime, another process started the same net again. Now this net is running and reaches the same point where it is waiting for user input. Since both nets might have been started with differing initial knowledge, the question to the user might be different despite it being asked by the same action. Now if the user answers any one of the questions the controller needs to know which instance of the net/action to send the answer to. Since we use the net id as the service name, each action has a dedicated service to receive the reply on so there will be no confusion.

The udate request works a little differently. Here we explicitly trigger communication with the controller. This is done via the variable name. If the variable is called `CONTROLLER` (`if variable == "CONTROLLER":`), the knowledge base immediately forwards the update to the controller. Otherwise, the value given is stored in the knowledge base directly as before.

All of this is done automatically by the knowledge base and the plugin server described above. So there is no need for the programmer to be involved in this. It just works in the background. How all of this actually is explained below.


###Controller usage example

We will build on the example used above and extend it to use our basic controlller.

**The domain file**

We will updata the domain file used above to include a new knowledge base action that will commuictate with the controller. The new domain file called `controller_domain.yaml` can be found [here](rpn_recipe_planner/etc/domains/controller_domain.yaml) and looks like this:

```python
instances: &instances
    - 
external_knowledge_base:
    module: "rpn_controller_kb.rpn_controller_knowledge_base"
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
    test_action:
        <<: *kb_action
        params:
            - "operation"
```

The only two differences to the domain file above for the simple example is that we are using our new knowledge base:

```python
external_knowledge_base:
    module: "rpn_controller_kb.rpn_controller_knowledge_base"
    class:  "RPKnowledgeBase"
```

And that we added a new action:

```python
    test_action:
        <<: *kb_action
        params:
            - "operation"
```

**The plan file**

The plan file `controller_example.yaml` can be found [here](rpn_recipe_planner/etc/plans/controller_example.yaml) and looks like this:

```python
example_plan:
    server:
        module: "rpn_controller_servers.basic_controller_plugin_server"
        class:  "BasicControllerPluginServer"
    action:
        module: "rpn_recipe_planner_msgs.msg"
        class: "RosServerAction"
    initial_knowledge:
        value: 5
    plan:
        - rpn_test_server: {}
        - while:
            condition: {Comparison: ["ne", [LocalQuery: "value", 10]]}
            actions:
                - test_server: {}
                - result_to_value:
                    operation:
                        LocalUpdate: ["value", {LocalQuery: "result"}, ""]
        - test_action:
            operation:
                RemoteUpdate: ["CONTROLLER", {LocalQuery: "value"}, ""]
        - test_action:
            operation:
                RemoteQuery: ["CONTROLLER", ""]

```

The only two differences between this new file and the one initially used are that we are now using our new basic controller plugin described above:

```python
    server:
        module: "rpn_controller_servers.basic_controller_plugin_server"
        class:  "BasicControllerPluginServer"
```

This means that the plan will not be turned into a ROS action server but it will become a `BasicControllerPluginServer`. Hence this will automaticallt register itself with the controller and be therefore maintained by it.

We also added two instances of the `test_action` that we difined earlier:

```python
        - test_action:
            operation:
                RemoteUpdate: ["CONTROLLER", {LocalQuery: "value"}, ""]
        - test_action:
            operation:
                RemoteQuery: ["CONTROLLER", ""]
```

The first one will update the controller with the value of `value` after the loop has finished and the second one will query something form the controller. In both occasions the `meta_info` field has been omitted but could be used to transmit additional information to the controller. In the case of the query, the variable name also has been set to `""` as it doesn't really matter here. This should of course be changed for any propper implementation.

**The worlds file**

The final thing we have to change is the [`worlds.yaml`](rpn_recipe_planner/etc/worlds.yaml) file to include our new domain and plan file:

```python
example:
    domain: domains/domain.yaml
    recipes:
        - plans/example.yaml
controller:
    domain: domains/controller_domain.yaml
    recipes:
        - plans/controller_example.yaml
```

Now if we start our recipe planner with `world:=controller` it will load the new files.

###Note

The plugin server shown here is only used for the plan file. You do not need to use it for the actions. For an action to comminicate with the controller it can simply use the query and update methods provided by the [`RPNActionServer`](ros_petri_net/src/rpn_action_servers/rpn_action_server.py). The controller plugin server should only be used for things that are directly started by the controller. Actions are started by the net instead.

###Running thr controller example

**You have to run the controller first.** There is currently no functionalty to retry the registration of a server more than once. This has been disabled as it led to problems along the way.

**Starting the controller**

This a simple as running any other ROS node:

```
rosrun rpn_controller example_controller.py
```

**Starting the recipe planner**

Same as berfore but using a different world:

```
roslaunch rpn_recipe_planner recipe_planner.launch world:=controller
```

Once this has started you should see that it registers with the controller  via the print outs on the terminal.

**Starting the actions**

Same as before:

```
rosrun rpn_recipe_planner simple_test.py
rosrun rpn_recipe_planner test.py
```

In the input prompt of the controller write:

```
Start server> example_plan
```

which is the name of our net. Once the net has finished, you should see the output of the controller update and query printed to the terminal.

A video showing how this all works can be found here: https://youtu.be/hcvYERSpIp8