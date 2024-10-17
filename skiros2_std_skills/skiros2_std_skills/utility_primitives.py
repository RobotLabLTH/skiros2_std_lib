from skiros2_skill.core.skill import SkillDescription
from skiros2_common.core.primitive import PrimitiveBase
from skiros2_common.core.world_element import Element
from skiros2_common.core.params import ParamTypes
from skiros2_std_skills.thread_primitive import PrimitiveThreadBase


#################################################################################
# Wait
#################################################################################


class Wait(SkillDescription):
    """
    @brief Returns Running for a specific amount of time
    """

    def createDescription(self):
        # =======Params=========
        self.addParam("Duration", 0.0, ParamTypes.Required)


class wait(PrimitiveBase):
    def createDescription(self):
        self.setDescription(Wait(), self.__class__.__name__)

    def onPreempt(self):
        return self.success("Done")

    def onStart(self):
        self.last = self.node.get_clock().now()
        return True

    def execute(self):
        duration = self.node.get_clock().now() - self.last
        if duration.to_sec() > self.params["Duration"].value:
            return self.success("Done")
        return self.step("Waiting {}".format(self.params["Duration"].value))


#################################################################################
# Set relation
#################################################################################

class WmSetRelation(SkillDescription):
    """
    @brief Set a relation on the world model
    """

    def createDescription(self):
        # =======Params=========
        self.addParam("Src", Element("sumo:Object"), ParamTypes.Required)
        self.addParam("Relation", str, ParamTypes.Required)
        self.addParam("Dst", Element("sumo:Object"), ParamTypes.Required)
        self.addParam("RelationState", True, ParamTypes.Required)
        self.addParam("OldDstToRemove", Element("sumo:Object"), ParamTypes.Optional)
        self.addParam("OldSrcToRemove", Element("sumo:Object"), ParamTypes.Optional)


class wm_set_relation(PrimitiveThreadBase):
    def createDescription(self):
        self.setDescription(WmSetRelation(), self.__class__.__name__)

    def _set_relation(self, src, relation, dst):
        src.setRelation("-1", relation, dst.id)
        dst.setRelation(src.id, relation, "-1")
        self.action_strs.append(f"Set {src.id}-{relation}-{dst.id}.")

    def _remove_relation(self, src, relation, dst):
        rel = dst.getRelation(src.id, relation, "-1")
        if rel is not None:
            dst.removeRelation(rel)
        rel = src.getRelation("-1", relation, dst.id)
        if rel is not None:
            src.removeRelation(rel)
        self.action_strs.append(f"Removed {src.id}-{relation}-{dst.id}.")

    def run(self):
        src = self.params["Src"].value
        relation = self.params["Relation"].value
        dst = self.params["Dst"].value
        old_dst = self.params["OldDstToRemove"].value
        old_src = self.params["OldSrcToRemove"].value
        self.action_strs = []
        if src == dst:
            return self.fail("Can not relate {} with itself.".format(src), -1)
        if self.params["RelationState"].value:
            self._set_relation(src, relation, dst)
            if old_dst.id and old_dst.id != dst.id:
                self._remove_relation(src, relation, old_dst)
            elif old_src.id:
                self._remove_relation(old_src, relation, dst)
        else:
            self._remove_relation(src, relation, dst)

        self.params["Src"].value = src
        self.params["Dst"].value = dst
        self._wmi.update_element(src)
        self._wmi.update_element(dst)
        return self.success(" ".join(self.action_strs))

#################################################################################
# Set relation
#################################################################################


class WmSetProperties(SkillDescription):
    """
    @brief Set some properties on an element
    """

    def createDescription(self):
        # =======Params=========
        self.addParam("Src", Element("sumo:Object"), ParamTypes.Required)
        self.addParam("Properties", dict, ParamTypes.Required)


class wm_set_properties(PrimitiveBase):
    def createDescription(self):
        self.setDescription(WmSetProperties(), self.__class__.__name__)

    def execute(self):
        src = self.params["Src"].value
        props = self.params["Properties"].value
        for k, v in props.items():
            src.setProperty(k, v)
        self.params["Src"].value = src
        self._wmi.update_element_properties(src)
        return self.success("Set properties to {}. {}".format(src.id, props))

#################################################################################
# WmMoveObject
#################################################################################


class WmMoveObject(SkillDescription):
    """
    @brief Move an Object from StartLocation to TargetLocation in the world model
    """

    def createDescription(self):
        # =======Params=========
        self.addParam("StartLocation", Element("sumo:Object"), ParamTypes.Inferred)
        self.addParam("TargetLocation", Element("sumo:Object"), ParamTypes.Optional)
        self.addParam("Object", Element("sumo:Object"), ParamTypes.Required)
        self.addParam("Relation", "skiros:contain", ParamTypes.Required)
        # =======PreConditions=========
        self.addPreCondition(self.getRelationCond("StartContainObj", "skiros:spatiallyRelated", "StartLocation", "Object", True))
        # =======PostConditions=========
        #self.addPostCondition(self.getRelationCond("TargetContainObj", "skiros:spatiallyRelated", "TargetLocation", "Object", True))


class wm_move_object(PrimitiveBase):
    """
    Instant primitive

    Set Target-Contain-Object on the world model
    """

    def createDescription(self):
        self.setDescription(WmMoveObject(), self.__class__.__name__)

    def onEnd(self):
        self.params["StartLocation"].unset()
        return True

    def execute(self):
        start = self.params["StartLocation"].value
        target = self.params["TargetLocation"].value if self.params["TargetLocation"].value.id else start
        obj = self.params["Object"].value
        rel = obj.getRelation(pred=self._wmi.get_sub_properties("skiros:spatiallyRelated"), obj="-1")

        start.setProperty("skiros:ContainerState", "Empty")
        target.setProperty("skiros:ContainerState", "Full")
        rel["src"] = target.id
        rel["type"] = self.params["Relation"].value
        self._wmi.update_element_properties(start)
        self._wmi.update_element_properties(target)
        self.params["Object"].value = obj
        return self.success("{} moved from {} to {}.".format(obj.id, start.id, target.id))


class wm_move_and_transform_object(PrimitiveBase):
    """
    Instant primitive

    Set Target-Contain-Object on the world model and expresses the transformation in the frame of the target location
    """

    def createDescription(self):
        self.setDescription(WmMoveObject(), self.__class__.__name__)

    def onEnd(self):
        self.params["StartLocation"].unset()
        return True

    def transform_to_frame(self, element, target_frame):
        if not element.hasProperty("skiros:FrameId", not_none=True):
            raise Exception("Missing pose info for target.")
        reasoner = element._getReasoner("AauSpatialReasoner")
        reasoner.get_transform(element.getProperty(
            "skiros:FrameId").value, target_frame)
        reasoner.transform(element, target_frame)
        return element

    def execute(self):
        start = self.params["StartLocation"].value
        target = self.params["TargetLocation"].value if self.params["TargetLocation"].value.id else start
        obj = self.params["Object"].value
        rel = obj.getRelation(pred=self._wmi.get_sub_properties("skiros:spatiallyRelated"), obj="-1")

        start.setProperty("skiros:ContainerState", "Empty")
        target.setProperty("skiros:ContainerState", "Full")
        rel["src"] = target.id
        rel["type"] = self.params["Relation"].value

        if not target.hasProperty("skiros:FrameId", not_none=True):
            return self.fail("{} has no property 'skiros:FrameId'. Can not transform".format(obj.id), -1)
        new_parent_frame = target.getProperty("skiros:FrameId").value
        if not self.transform_to_frame(obj, new_parent_frame):
            return self.fail("Could not transform {} into parent frame {}.".format(obj.id, new_parent_frame), -2)
        obj.setProperty("skiros:BaseFrameId", new_parent_frame)

        self._wmi.update_element_properties(start)
        self._wmi.update_element_properties(target)
        self._wmi.update_element_properties(obj)
        self.params["Object"].value = obj
        return self.success("{} moved from {} to {}.".format(obj.id, start.id, target.id))


#################################################################################
# Counter
#################################################################################


class Counter(SkillDescription):
    """
    @brief      Returns Success after a number of tick
    """

    def createDescription(self):
        self.addParam("CountTarget", int, ParamTypes.Required)


class counter(PrimitiveBase):
    def createDescription(self):
        self.setDescription(Counter(), self.__class__.__name__)

    def _print_count(self):
        return "{}/{}".format(self._counter, self.params["CountTarget"].value)

    def onPreempt(self):
        return self.step(self._print_count())

    def onStart(self):
        self._counter = 0
        return True

    def execute(self):
        if self._counter < self.params["CountTarget"].value:
            self._counter += 1
            return self.step(self._print_count())
        else:
            return self.success(self._print_count())
