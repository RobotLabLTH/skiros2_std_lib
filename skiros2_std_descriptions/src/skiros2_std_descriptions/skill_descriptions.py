from skiros2_skill.core.skill import SkillDescription
from skiros2_common.core.primitive import PrimitiveBase
from skiros2_common.core.world_element import Element
from skiros2_common.core.params import ParamTypes
import rospy



class DescriptionTemplate(object):
    required = {}
    inferred = {}

class ArmDescriptionTemplate(DescriptionTemplate):
    required = {
        "Arm": Element("rparts:ArmDevice")
    }

class GripperDescriptionTemplate(DescriptionTemplate):
    required = {
        "Gripper": Element("rparts:GripperEffector")
    }

class DepthCameraDescriptionTemplate(DescriptionTemplate):
    required = {
        "Camera": Element("skiros:DepthCamera")
    }
    inferred = {
        "ViewFrame": Element("skiros:TransformationPose")
    }


# class requires:
#     def __init__(self, description):
#         self.description = description

#     def __call__(self, clazz):

#         if issubclass(clazz, DescriptionTemplate):



#         fun = clazz.createDescription
#         def createDescription(cls):
#             fun(cls)
#             for k,v in self.description.required.items():
#                 cls.addParam(k, v, ParamTypes.Required)
#             for k,v in self.description.inferred.items():
#                 cls.addParam(k, v, ParamTypes.Inferred)

#         clazz.createDescription = createDescription
#         return clazz

# class infers:
#     def __init__(self, description):
#         self.description = description

#     def __call__(self, clazz):
#         fun = clazz.createDescription
#         def createDescription(cls):
#             fun(cls)
#             for k,v in self.description.required.items():
#                 cls.addParam(k, v, ParamTypes.Inferred)
#             for k,v in self.description.inferred.items():
#                 cls.addParam(k, v, ParamTypes.Inferred)

#         clazz.createDescription = createDescription
#         return clazz



# #################################################################################
# # Decorators
# #################################################################################

# class use_description:
#     def __init__(self, description):
#         self.description = description

#     def __call__(self, clazz):
#         clazz.createDescription = lambda cls: cls.setDescription(self.description(), cls.__class__.__name__)
#         return clazz


# class requires:
#     def __init__(self, description):
#         self.description = description

#     def __call__(self, clazz):
#         clazz.createDescription = lambda cls: cls.setDescription(self.description(), cls.__class__.__name__)
#         return clazz






# #################################################################################
# # ArmDescription
# #################################################################################
# class ArmDescription(SkillDescription):
#     def createDescription(self):
#         self.addParam("Arm", Element("rparts:ArmDevice"), ParamTypes.Required)

# #################################################################################
# # ManipulatorDescription
# #################################################################################
# class ManipulatorDescription(SkillDescription):
#     def createDescription(self):
#         self.addParam("Gripper", Element("rparts:GripperEffector"), ParamTypes.Required)
#         self.addParam("Arm",     Element("rparts:ArmDevice"),       ParamTypes.Inferred)

#         self.addPreCondition(self.getRelationCond("AhasG", "skiros:hasA", "Arm", "Gripper", True))

# #################################################################################
# # ActiveVisionDescription
# #################################################################################
# class ActiveVisionDescription(SkillDescription):
#     def createDescription(self):
#         self.addParam("Camera",    Element("skiros:DepthCamera"),        ParamTypes.Required)
#         self.addParam("ViewFrame", Element("skiros:TransformationPose"), ParamTypes.Inferred)
#         self.addParam("Gripper",   Element("rparts:GripperEffector"),    ParamTypes.Inferred)
#         self.addParam("Arm",       Element("rparts:ArmDevice"),          ParamTypes.Inferred)

#         self.addPreCondition(self.getRelationCond("AhasG", "skiros:hasA",    "Arm",     "Gripper",   True))
#         self.addPreCondition(self.getRelationCond("GHasC", "skiros:contain", "Gripper", "Camera",    True))
#         self.addPreCondition(self.getRelationCond("CHasV", "skiros:contain", "Camera",  "ViewFrame", True))






# #################################################################################
# # Wait
# #################################################################################

# class Wait(SkillDescription):
#     """
#     @brief Returns Running for a specific amount of time
#     """
#     def createDescription(self):
#         #=======Params=========
#         self.addParam("Duration", 0.0, ParamTypes.Required)

# class wait(PrimitiveBase):
#     def createDescription(self):
#         self.setDescription(Wait(), self.__class__.__name__)

#     def onPreempt(self):
#         return self.success("Done")

#     def onStart(self):
#         self.last = rospy.Time.now()
#         return True

#     def execute(self):
#         duration = rospy.Time.now() - self.last
#         if duration.to_sec() > self.params["Duration"].value:
#             return self.success("Done")
#         return self.step("")


# #################################################################################
# # Set relation
# #################################################################################

# class WmSetRelation(SkillDescription):
#     """
#     @brief Set a relation on the world model
#     """
#     def createDescription(self):
#         #=======Params=========
#         self.addParam("Src", Element("sumo:Object"), ParamTypes.Required)
#         self.addParam("Relation", str, ParamTypes.Required)
#         self.addParam("Dst", Element("sumo:Object"), ParamTypes.Required)
#         self.addParam("RelationState", True, ParamTypes.Required)

# class wm_set_relation(PrimitiveBase):
#     def createDescription(self):
#         self.setDescription(WmSetRelation(), self.__class__.__name__)

#     def execute(self):
#         src = self.params["Src"].value
#         relation = self.params["Relation"].value
#         dst = self.params["Dst"].value
#         if self.params["RelationState"].value:
#             src.setRelation("-1", relation, dst.id)
#             dst.setRelation(src.id, relation, "-1")
#         else:
#             rel = dst.getRelation(src.id, relation, "-1")
#             if rel is not None:
#                 dst.removeRelation(rel)
#             rel = src.getRelation("-1", relation, dst.id)
#             if rel is not None:
#                 src.removeRelation(rel)
#         self.params["Src"].value = src
#         self.params["Dst"].value = dst
#         return self.success("{} {}-{}-{}".format("Set" if self.params["RelationState"].value else "Unset", src.id, relation, dst.id))

# #################################################################################
# # Set relation
# #################################################################################

# class WmSetProperties(SkillDescription):
#     """
#     @brief Set some properties on an element
#     """
#     def createDescription(self):
#         #=======Params=========
#         self.addParam("Src", Element("sumo:Object"), ParamTypes.Required)
#         self.addParam("Properties", dict, ParamTypes.Required)

# class wm_set_properties(PrimitiveBase):
#     def createDescription(self):
#         self.setDescription(WmSetProperties(), self.__class__.__name__)

#     def execute(self):
#         src = self.params["Src"].value
#         props = self.params["Properties"].value
#         for k, v in props.iteritems():
#             src.setProperty(k, v)
#         self.params["Src"].value = src
#         return self.success("Setted properties to {}. {}".format(src.id, props))

# #################################################################################
# # WmMoveObject
# #################################################################################

# class WmMoveObject(SkillDescription):
#     """
#     @brief Move an Object from StartLocation to TargetLocation in the world model
#     """
#     def createDescription(self):
#         #=======Params=========
#         self.addParam("StartLocation", Element("sumo:Object"), ParamTypes.Required)
#         self.addParam("TargetLocation", Element("sumo:Object"), ParamTypes.Optional)
#         self.addParam("Object", Element("sumo:Object"), ParamTypes.Required)
#         self.addParam("Relation", "skiros:contain", ParamTypes.Required)
#         #=======PreConditions=========
#         self.addPreCondition(self.getRelationCond("StartContainObj", "skiros:spatiallyRelated", "StartLocation", "Object", True))
#         #=======PostConditions=========
#         #self.addPostCondition(self.getRelationCond("TargetContainObj", "skiros:spatiallyRelated", "TargetLocation", "Object", True))

# class wm_move_object(PrimitiveBase):
#     """
#     Instant primitive

#     Set Target-Contain-Object on the world model
#     """
#     def createDescription(self):
#         self.setDescription(WmMoveObject(), self.__class__.__name__)

#     def onEnd(self):
#         self.params["StartLocation"].unset()
#         return True

#     def execute(self):
#         start = self.params["StartLocation"].value
#         target = self.params["TargetLocation"].value if self.params["TargetLocation"].value.id else start
#         objectt = self.params["Object"].value
#         rel = objectt.getRelation(pred=self._wmi.get_sub_properties("skiros:spatiallyRelated"), obj="-1")

#         start.setProperty("skiros:ContainerState", "Empty")
#         target.setProperty("skiros:ContainerState", "Full")
#         rel["src"] = target.id
#         rel["type"] = self.params["Relation"].value
#         self._wmi.update_element_properties(start)
#         self._wmi.update_element_properties(target)
#         self.params["Object"].value = objectt
#         return self.success("{} moved from {} to {}.".format(objectt.id, start.id, target.id))
