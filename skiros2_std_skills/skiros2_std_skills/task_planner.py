from action_msgs.msg import GoalStatus
from skiros2_skill.core.skill import SkillDescription, SkillBase, SerialStar, ParallelFf, State, SkillWrapper
from skiros2_common.core.world_element import Element
from skiros2_common.core.params import ParamTypes
from skiros2_task.ros.task_manager_interface import TaskManagerInterface
from skiros2_skill.ros.utils import deserialize_skill
import skiros2_common.tools.logger as log

class TaskPlan(SkillDescription):
    def createDescription(self):
        #=======Params=========
        self.addParam("Goal", str, ParamTypes.Required)

class task_plan(SkillBase):
    """
    @brief      This special skill is a client for the task planner action
                server implemented in skiros2_task.

                After a plan is received the skill expands, passing from being a
                primitive to be a compound skill.
    """
    def createDescription(self):
        self.setDescription(TaskPlan(), self.__class__.__name__)
        self._expand_on_start = True
        self._skill_to_expand = None

    def expand(self, skill):
        self._skill_to_expand = skill

    def onPreempt(self):
        self._tm.preempt()
        return self.fail("Preempted.", -100)

    def onStart(self):
        self._action_status = None
        self._action_msg = None
        self._tm = TaskManagerInterface(self.node)
        return self._tm.plan(self.params["Goal"].value, self._done_planning)

    def _done_planning(self, status, result):
        log.info(f"Planning returned. Status: '{status}'")
        self._action_status = result.progress_code
        self._action_msg = result.progress_message

    def _add_children(self, skill, children):
        string = ""
        for i in children:
            s = self.skill(i.type, i.name)
            s.specifyParamsDefault(i.ph)
            skill.addChild(s)
            string += "{}[{}](".format(s.label, s.params.printState())
            string += self._add_children(skill.last(), i.children)
            string += ")\n"
        return string

    def execute(self):
        if self._action_status is None:
            return self.step("Planning...")
        elif self._action_status==0 or self._action_status==1:
            return self.fail(self._action_msg, -101)
        elif self._action_status==2:
            return self.success(self._action_msg)
        elif self._skill_to_expand:
            task = deserialize_skill(self._action_msg)
            self._skill_to_expand.setProcessor(SerialStar())
            task_string = self._add_children(self._skill_to_expand, task.children)
            self._skill_to_expand = None
            return self.step("{}".format(task_string))
        else:
            super(task_plan, self).execute()
