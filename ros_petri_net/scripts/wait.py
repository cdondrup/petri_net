#!/usr/bin/env python
# -*- coding: utf-8 -*-


import rospy
from actionlib_msgs.msg import GoalStatus
from pnp_msgs.msg import waitAction, waitActionResult, waitFeedback
from threading import Thread
from actionlib import ActionServer


class WaitServer(object):
    def __init__(self, name):
        rospy.loginfo("Starting '%s'." % name)
        self._as = ActionServer(name, waitAction, goal_cb=self.goal_cb, cancel_cb=self.cancel_cb, auto_start=False)
        self._as.start()
        rospy.loginfo("Done.")
                                                    
    def goal_cb(self, gh):
        t = Thread(target=self.execute,
        args=(gh,))
        t.start()
                                                                                    
                                                                                        def
                                                                                        cancel_cb(self,
                                                                                                  gh):
                                                                                                    gh.set_canceled(waitActionResult())
                                                                                                            
                                                                                                                def
                                                                                                                execute(self,
                                                                                                                        gh):
                                                                                                                            gh.set_accepted()        
                                                                                                                                    goal
                                                                                                                                    =
                                                                                                                                    gh.get_goal()
                                                                                                                                            print
                                                                                                                                            "Waiting
                                                                                                                                            for
                                                                                                                                            %s
                                                                                                                                            seconds"
                                                                                                                                            %
                                                                                                                                            goal.time
                                                                                                                                                    end
                                                                                                                                                    =
                                                                                                                                                    rospy.Time.now().to_sec()
                                                                                                                                                    +
                                                                                                                                                    goal.time
                                                                                                                                                            while
                                                                                                                                                            rospy.Time.now().to_sec()
                                                                                                                                                            <
                                                                                                                                                            end
                                                                                                                                                            and
                                                                                                                                                            not
                                                                                                                                                            rospy.is_shutdown()
                                                                                                                                                            \
                                                                                                                                                                                and
                                                                                                                                                            not
                                                                                                                                                            gh.status_tracker.status.status
                                                                                                                                                            ==
                                                                                                                                                            GoalStatus.PREEMPTED:
                                                                                                                                                                            rospy.sleep(0.1)
                                                                                                                                                                                        gh.publish_feedback(waitFeedback(remaining=end-rospy.Time.now().to_sec()))
                                                                                                                                                                                                if
                                                                                                                                                                                                not
                                                                                                                                                                                                gh.status_tracker.status.status
                                                                                                                                                                                                ==
                                                                                                                                                                                                GoalStatus.PREEMPTED:
                                                                                                                                                                                                                gh.set_succeeded(waitActionResult())

                                                                                                                                                                                                                if
                                                                                                                                                                                                                __name__
                                                                                                                                                                                                                ==
                                                                                                                                                                                                                "__main__":
                                                                                                                                                                                                                        rospy.init_node("wait")
                                                                                                                                                                                                                            w
                                                                                                                                                                                                                            =
                                                                                                                                                                                                                            WaitServer(rospy.get_name())
                                                                                                                                                                                                                                rospy.spin()

