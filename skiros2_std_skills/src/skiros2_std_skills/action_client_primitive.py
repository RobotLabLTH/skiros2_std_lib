from skiros2_common.core.primitive import PrimitiveBase
import skiros2_common.tools.logger as log
import rospy
import Queue


class PrimitiveActionClient(PrimitiveBase):
    """
    @brief Base class for skills based on a action server.

    See test_action_skill for a practical example

    build_client_onstart defines if the client will be built every time the skill it is called or not
    (save up to 0.3 seconds). Default is True
    """
    build_client_onstart = True

    def onPreempt(self):
        """
        @brief Cancel all goals
        """
        self.client.cancel_all_goals()
        return self.success("Stopped")

    def onStart(self):
        self.q = Queue.Queue(1)
        self._done = None
        if self.build_client_onstart:
            self.client = self.buildClient()
        if not self.client.wait_for_server(rospy.Duration(0.5)):
            log.error("[{}]".format(self._label), "Action server {} is not available.".format(self.client.action_client.ns))
            return False
        self.client.send_goal(self.buildGoal(), done_cb=self._doneCb, feedback_cb=self._feedbackCb)
        return True

    def restart(self, goal, text="Restarting action."):
        self._done = None
        self.client.send_goal(goal, done_cb=self._doneCb, feedback_cb=self._feedbackCb)
        return self.step(text)

    def execute(self):
        if not self.q.empty():
            msg = self.q.get(False)
            if self._done is not None:
                return self.onDone(self._done, msg)
            else:
                return self.onFeedback(msg)
        return self.step("")

    def _doneCb(self, status, msg):
        self.q.put(msg)
        self._done = status

    def _feedbackCb(self, msg):
        if self.q.empty():
            self.q.put(msg)

    def onInit(self):
        """
        @brief Optional to override. Called once when loading the primitive. If return False, the primitive is not loaded
        @return True if loaded correctly. False on error
        """
        if not self.build_client_onstart:
            self.client = self.buildClient()
        return True

    def onReset(self):
        """
        @brief Optional to override. Re-initialize the primitive
        """
        pass

    def onEnd(self):
        """
        @brief Optional to override. Routine called just after skill execution end
        """
        pass

    def buildClient(self):
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
        # Do something with feedback msg
        return self.step("")

    def onDone(self, status, msg):
        """
        @brief To override. Called when goal is achieved.
        @return self.success or self.fail
        """
        # Do something with result msg
        return self.success("Finished. State: {} Result: {}".format(status, msg))
