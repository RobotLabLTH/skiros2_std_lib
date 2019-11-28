from skiros2_skill.core.skill import SkillDescription
from skiros2_common.core.primitive import PrimitiveBase
from skiros2_common.core.world_element import Element
from skiros2_common.core.params import ParamTypes
import rospy

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
        self.last = rospy.Time.now()
        return True

    def execute(self):
        duration = rospy.Time.now() - self.last
        if duration.to_sec() > self.params["Duration"].value:
            return self.success("Done")
        return self.step("")


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


class wm_set_relation(PrimitiveBase):
    def createDescription(self):
        self.setDescription(WmSetRelation(), self.__class__.__name__)

    def execute(self):
        src = self.params["Src"].value
        relation = self.params["Relation"].value
        dst = self.params["Dst"].value
        if self.params["RelationState"].value:
            src.setRelation("-1", relation, dst.id)
            dst.setRelation(src.id, relation, "-1")
        else:
            rel = dst.getRelation(src.id, relation, "-1")
            if rel is not None:
                dst.removeRelation(rel)
            rel = src.getRelation("-1", relation, dst.id)
            if rel is not None:
                src.removeRelation(rel)
        self.params["Src"].value = src
        self.params["Dst"].value = dst
        return self.success("{} {}-{}-{}".format("Set" if self.params["RelationState"].value else "Unset", src.id, relation, dst.id))

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
        for k, v in props.iteritems():
            src.setProperty(k, v)
        self.params["Src"].value = src
        return self.success("Setted properties to {}. {}".format(src.id, props))

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
        objectt = self.params["Object"].value
        rel = objectt.getRelation(pred=self._wmi.get_sub_properties("skiros:spatiallyRelated"), obj="-1")

        start.setProperty("skiros:ContainerState", "Empty")
        target.setProperty("skiros:ContainerState", "Full")
        rel["src"] = target.id
        rel["type"] = self.params["Relation"].value
        self._wmi.update_element_properties(start)
        self._wmi.update_element_properties(target)
        self.params["Object"].value = objectt
        return self.success("{} moved from {} to {}.".format(objectt.id, start.id, target.id))

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
            return self.step(self._print_count())
        else:
            return self.success(self._print_count())
