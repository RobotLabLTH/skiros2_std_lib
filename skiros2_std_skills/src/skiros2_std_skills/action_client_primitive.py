from skiros2_common.core.primitive import PrimitiveBase
from skiros2_common.core.params import ParamTypes
import skiros2_common.tools.logger as log
import rospy
from std_srvs.srv import Empty, EmptyRequest
try:
    import Queue as queue
except ImportError:
    import queue

class PrimitiveActionClient(PrimitiveBase):
    """
    @brief Base class for skills based on a action server.

    See test_action_skill for a practical example

    build_client_onstart defines if the client will be built every time the skill it is called or not
    (save up to 0.3 seconds). Default is True
    """
    build_client_onstart = True

    def modifyDescription(self, skill):
        skill.addParam("Reset", bool, ParamTypes.Required, "If True, reset the server, if config service is available.")

    def onPreempt(self):
        """
        @brief Cancel all goals
        """
        self.client.cancel_all_goals()
        return self.success("Stopped")

    def onStart(self):
        self.fb = queue.Queue(1)
        self.res = queue.Queue(1)
        if self.build_client_onstart:
            self.client = self.buildClient()
        if not self.client.wait_for_server(rospy.Duration(0.5)):
            return self.startError("Action server {} is not available.".format(self.client.action_client.ns), -101)
        if self.params["Reset"].value:
            self.params["Reset"].value = False
            try:
                srv = rospy.ServiceProxy('/{}/config'.format(self.client.action_client.ns), Empty)
                srv(EmptyRequest())
            except rospy.ServiceException as e:
                log.warn("[PrimitiveActionClient]", "Server reset failed. {}".format(e))
        self.client.send_goal(self.buildGoal(), done_cb= self._doneCb, feedback_cb = self._feedbackCb)
        return True

    def restart(self, goal, text="Restarting action."):
        self.client.send_goal(goal, done_cb= self._doneCb, feedback_cb = self._feedbackCb)
        return self.step(text)

    def execute(self):
        if not self.fb.empty():
            msg = self.fb.get(False)
            return self.onFeedback(msg)
        elif not self.res.empty():
            msg, status = self.res.get(False)
            return self.onDone(status, msg)
        return self.step("")

    def _doneCb(self, status, msg):
        self.res.put((msg, status))

    def _feedbackCb(self, msg):
        if self.fb.empty():
            self.fb.put(msg)

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
        return True

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
        #Do something with feedback msg
        return self.step("")

    def onDone(self, status, msg):
        """
        @brief To override. Called when goal is achieved.
        @return self.success or self.fail
        """
        #Do something with result msg
        return self.success("Finished. State: {} Result: {}".format(status, msg))
