from skiros2_common.core.abstract_skill import State
from skiros2_common.core.primitive import PrimitiveBase
from skiros2_common.core.params import ParamTypes
import skiros2_common.tools.logger as log
import rclpy
from rclpy import action, task, client, Future
from rclpy.time import Duration
from actionlib_msgs.msg import GoalStatus
import queue
from threading import Lock
from typing import Optional
from action_msgs.msg import GoalStatus
from skiros2_std_skills.utils import AtomicVar

class PrimitiveServiceClient(PrimitiveBase):
    """
    @brief Base class for skills based on a service client.

    See test_service_skill for a practical example
    """

    build_client_onstart = False
    call_timeout_sec = None

    def onPreempt(self):
        """
        @brief Cancel service call
        """
        log.debug("Cancel requested")
        if self._future is not None:
            self._future.cancel()
            self._future = None
            self.client.destroy()
            self.client = None
            log.debug("Service call canceled")
            return self.success("Service call canceled.")
        else:
            return self.fail("Goal has not been accepted or rejected, cannot cancel.")

    def onStart(self):
        self._response = AtomicVar()

        # 1. Build client
        if self.build_client_onstart or self.client is None:
            self.client = self.buildClient()

        if self.client is None:
            return self.startError("Action client returned by buildClient() is None.", -101)

        # 2. Wait for server
        if not self.client.wait_for_service(0.5):
            return self.startError("Service server is not available: {}".format(self.client), -101)

        # 3. Build message and call service
        service_request = self.buildRequest()
        if service_request is None:
            return self.startError("Service request is None", -106)

        self._future = self.client.call_async(service_request)
        self._future.add_done_callback(self._response_callback)
        self._last_call_timestamp = self.node.get_clock().now()

        return True
    
    def _response_callback(self, future: Future):
        self._response.set(future.result())

    def execute(self):
        res = self._response.get_and_reset()
        if res is not None:
            return self.onDone(res)
        
        time_since_start_sec = (self.node.get_clock().now()-self._last_call_timestamp).nanoseconds/1.0e9
        if self.call_timeout_sec is not None:
            if time_since_start_sec > self.call_timeout_sec:
                self.client.destroy()
                self.client = None
                return self.fail("Service call did not finish within timeout of %d sec" % self.call_timeout_sec, -104)

        return self.step("Service called, been waiting for %.1f sec" % time_since_start_sec)

    def onInit(self):
        """
        @brief Optional to override. Called once when loading the primitive. If return False, the primitive is not loaded
        @return True if loaded correctly. False on error
        """
        self.client = None
        if not self.build_client_onstart:
            self.client = self.buildClient()

        return True

    def onEnd(self):
        """
        @brief Optional to override. Routine called just after skill execution end
        """
        return True

    def buildClient(self)->client.Client:
        """
        @brief To override. Called when starting the skill
        @return an service client created by self.node.create_client(...)
        """
        pass

    def buildRequest(self):
        """
        @brief To override. Called when starting the skill
        @return an action msg initialized
        """
        pass

    def onDone(self, response):
        """
        @brief To override. Called when response has arrived.
        @return self.success or self.fail
        """
        #Do something with result msg
        return self.success("Finished. Result: {}".format(response))
