from skiros2_skill.core.skill import SkillDescription, SkillBase, Sequential, ParallelFf, State, SkillWrapper
from skiros2_common.core.world_element import Element
from skiros2_common.core.params import ParamTypes
from skiros2_task.ros.task_manager_interface import TaskManagerInterface
from skiros2_skill.ros.utils import deserialize_skill
import skiros2_common.tools.logger as log

class TaskPlan(SkillDescription):
    """
    """
    def createDescription(self):
        #=======Params=========
        self.addParam("Goal", str, ParamTypes.Required)
        #self.addParam("TaskPlan", "", ParamTypes.Required)

class task_plan(SkillBase):
    def createDescription(self):
        self.setDescription(TaskPlan(), self.__class__.__name__)
        self._expand_on_start = True
        self._action_status = None
        self._action_msg = None
        self._skill_to_expand = None

    def expand(self, skill):
        self._skill_to_expand = skill

    def onPreempt(self):
        self._tm.preempt()
        return self.fail("Preempted.", -100)

    def onStart(self):
        self._tm = TaskManagerInterface()
        return self._tm.plan(self.params["Goal"].value, self._done_planning)

    def _done_planning(self, status, msg):
        self._action_status = msg.progress_code
        self._action_msg = msg.progress_message

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
            self._skill_to_expand.setProcessor(Sequential())
            task_string = self._add_children(self._skill_to_expand, task.children)
            self._skill_to_expand = None
            return self.step("{}".format(task_string))
        else:
            super(SkillBase, self).execute()

class DynamicTree(SkillDescription):
    """
    """
    def createDescription(self):
        #=======Params=========
        self.addParam("TaskPlan", str, ParamTypes.Required)

class dynamic_tree(SkillBase):
    def createDescription(self):
        self.setDescription(DynamicTree(), self.__class__.__name__)
        self._expand_on_start = True

    def expand(self, skill):
        task = deserialize_skill(self.params["TaskPlan"].value)
        self._add_children(skill, task)

    def _add_children(self, skill, children):
        string = "{}(".format(skill.label)
        for i in children:
            skill.addChild(self.skill(i.type, i.name))
            skill.last().specifyParamsDefault(i.ph)
            string += self._add_children(skill.last(), i.children)
        return string + ") "

    def execute(self):
        if not self.children:
            return self.success("No skills to execute.")
        else:
            super(SkillBase, self).execute()