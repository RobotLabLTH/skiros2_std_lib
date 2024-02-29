from skiros2_common.core.abstract_skill import State
from skiros2_common.core.primitive import PrimitiveBase
from skiros2_common.core.params import ParamTypes
import skiros2_common.tools.logger as log
import rclpy
from rclpy import action, task
from rclpy.time import Duration
from actionlib_msgs.msg import GoalStatus
import queue
from threading import Lock
from typing import Optional
from action_msgs.msg import GoalStatus
from skiros2_std_skills.utils import AtomicVar

class PrimitiveActionClient(PrimitiveBase):
    """
    @brief Base class for skills based on a action server.

    See test_action_skill for a practical example

    build_client_onstart defines if the client will be built every time the skill it is called or not
    (save up to 0.3 seconds). Default is True

    feedback_timeout_sec defines a timeout in seconds for not reciving action 
    feedback in a timely manner. Could be used to detect if the action server 
    has gone down or experiences issues. Default is no timeout.

    wait_for_server_timeout defines the amount of time to wait for the action server to be alive.
    Defaults to 1 second.
    """
    build_client_onstart = True
    feedback_timeout_sec = None # set to second if you want reliability
    wait_for_server_timeout = 1.0

    def __init__(self):
        super().__init__()
        self.client = None

    def onPreempt(self):
        """
        @brief Cancel all goals
        """
        
        log.info("cancel requested")
        if self._goal_handle is not None:
            future = self._goal_handle.cancel_goal_async()
            future.add_done_callback(self._cancel_callback)

            log.info("cancel requested completed.")
            return self.success("Cancel requested.")
        else:
            return self.fail("Goal has not been accepted or rejected, cannot cancel.")

    def onStart(self):
        self._feedback_msg = AtomicVar()
        self._goal_msg = AtomicVar()
        self._result_msg = AtomicVar()
        self._fail_status = AtomicVar()

        if self.build_client_onstart or self.client is None:
            self.client = self.buildClient()
        
        if self.client is None:
            return self.startError("Action client returned by buildClient() is None.", -105)

        if not self.client.wait_for_server(self.wait_for_server_timeout):
            return self.startError("Action server is not available: {}".format(self.client), -101)
                
        self._goal_handle = None

        goal = self.buildGoal()
        if goal is None:
            return self.startError("Action goal is None", -106)

        self._goal_future = self.client.send_goal_async(goal, feedback_callback = self._feedback_callback)
        self._goal_future.add_done_callback(self._goal_request_callback)
        
        self._get_result_future = None
        self._last_feedback_timestamp = self.node.get_clock().now()

        return True

    #def restart(self, goal, text="Restarting action."):
    #    self.client.remove_future(self._cli)
    #    self._goal_future = self.client.send_goal_async(goal, feedback_callback=self._feedback_callback)
    #    return self.step(text)
    
    def execute(self):
        res = self._goal_msg.get_and_reset()
        if res is not None:
            if res == False:
                return self.fail("Rejected.", -102)

        fb = self._feedback_msg.get_and_reset()
        if fb is not None:
            self._last_feedback_timestamp = self.node.get_clock().now()
            return self.onFeedback(fb)
        
        status = self._fail_status.get_and_reset()
        if status is not None:
            return self.fail("Failed with status: %d" % status, -103)
        
        res = self._result_msg.get_and_reset()
        if res is not None:
            return self.onDone(res) # TODO: Remove None
        
        if self.feedback_timeout_sec is not None:
            time_since_start_sec = (self.node.get_clock().now()-self._last_feedback_timestamp).nanoseconds/1.0e9 # type: Duration
            log.info("Time since start %f, %d" % (time_since_start_sec, self.feedback_timeout_sec))
            if time_since_start_sec > self.feedback_timeout_sec:
                self.client.destroy()
                self.client = None
                return self.fail("Action feedback did not arrive within timeout of %d sec" % self.feedback_timeout_sec, -104)

        return State.Running
    
    def _cancel_callback(self, future):
        cancel_response = future.result()
        if len(cancel_response.goals_canceling) > 0:
            log.info('Goal successfully canceled')
        else:
            log.info('Goal failed to cancel')

    def _goal_request_callback(self, future: task.Future):
        goal_handle = future.result()
        self._goal_handle = goal_handle
        if goal_handle.accepted:
            self._goal_msg.set(True)
        else:
            self._goal_msg.set(False)

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self._result_callback)

    def _result_callback(self, future):
        result = future.result().result
        status = future.result().status # type: GoalStatus
        
        if status == GoalStatus.STATUS_SUCCEEDED:
            self._result_msg.set(result)
        else:
            self._fail_status.set(status)

    def _feedback_callback(self, msg):
        self._feedback_msg.set(msg.feedback)

    def onInit(self):
        """
        @brief Optional to override. Called once when loading the primitive. If return False, the primitive is not loaded
        @return True if loaded correctly. False on error
        """
        if not self.build_client_onstart:
            self.client = self.buildClient()
        return True

    def onEnd(self):
        """
        @brief Optional to override. Routine called just after skill execution end
        """
        return True

    def buildClient(self)->action.ActionClient:
        """
        @brief To override. Called when starting the skill
        @return an action client (e.g. actionlib.SimpleActionClient)
        """
        pass

    def buildGoal(self):
        """
        @brief To override. Called when starting the skill
        @return an action msg initialized
        """
        pass

    def onFeedback(self, msg):
        """
        @brief To override. Called every time a new feedback msg is received.
        @return Can return self.success, self.fail or self.step
        """
        #Do something with feedback msg
        return self.step("")

    def onDone(self, msg):
        """
        @brief To override. Called when goal is achieved.
        @return self.success or self.fail
        """
        #Do something with result msg
        return self.success("Finished. Result: {}".format(msg))
