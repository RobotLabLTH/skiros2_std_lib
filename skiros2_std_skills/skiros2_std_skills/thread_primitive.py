from typing import final
import threading
from skiros2_common.core.primitive import PrimitiveBase
from skiros2_common.core.abstract_skill import State


class PrimitiveThreadBase(PrimitiveBase):
    """
    @brief      Base class for primitive skill that executes a long running
                process

                A primitive skill must not block the ticking of the BT. This
                class is intended for executing a long running process that
                cannot be reasonably split up into smaller pieces and instead
                starts a thread which executes the long running process.

                In particular this class allows the SkiROS gui to provide
                feedback and allows other skills to run at the same time as
                this skill is executing.
    """

    def preStart(self):
        """
        @brief      Any preparation is performed here

                    Runs before the thread is started and should do any
                    (small) preparations that need to be done before starting
                    the skill. The result of this function is then passed
                    through and returned by onStart.

        @return     True if the thread should start, False if not.
        """
        return True

    @final
    def onStart(self):
        """
        @brief      Starts the main thread of the skill
        """
        # Reset relevant flags
        self.status = 'Running.'
        self.caught_exception = False
        self.skill_succeeded = False
        self._skill_state = State.Running

        # Perform any precomputations in this function
        skill_can_start = self.preStart()

        # Start new thread if preStart was successful
        if skill_can_start:
            self.thread = threading.Thread(target=self._run)
            self.thread.excepthook = self._excepthook
            self.thread.start()

        return skill_can_start

    def _excepthook(self, args):
        self.caught_exception = True
        self.exception_msg = args

    @final
    def execute(self):
        if self.thread.is_alive():
            # While the skill is not complete return the running message
            return self.step(self.status)
        else:
            self.thread.join()

            if not self.caught_exception:
                # Return a complete message
                return self._skill_state

            # Send any exception to the skiros gui.
            return self.fail(self.exception_msg, -1)

    @final
    def _run(self):
        self._skill_state = self.run()
        if self._skill_state == State.Running:
            raise RuntimeError('The run function should not return a non terminal state.')

    def run(self):
        """
        @brief      Performs the long running execution which must happen
                    in a separate thread

                    This function must not return a non-terminal state, i.e.
                    `self.step('...')` cannot be returned.

                    There is no mechanism for preemption, it is responsibility
                    of whoever implements this function to make sure that the
                    thread stops when a preemption signal is sent.

        @return     A SkiROS state, i.e. `self.success('...')` or
                    `self.fail('...', code)`.
        """
        raise NotImplementedError
