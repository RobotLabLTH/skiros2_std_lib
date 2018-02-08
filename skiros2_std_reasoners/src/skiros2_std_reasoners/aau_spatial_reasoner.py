#################################################################################
# Software License Agreement (BSD License)
#
# Copyright (c) 2016, Francesco Rovida
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# * Redistributions of source code must retain the above copyright
#   notice, this list of conditions and the following disclaimer.
# * Redistributions in binary form must reproduce the above copyright
#   notice, this list of conditions and the following disclaimer in the
#   documentation and/or other materials provided with the distribution.
# * Neither the name of the copyright holder nor the
#   names of its contributors may be used to endorse or promote products
#   derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
# ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#################################################################################

from skiros2_common.core.discrete_reasoner import DiscreteReasoner
from skiros2_common.core.world_element import Element
import skiros2_common.tools.logger as log
import tf2_ros as tf
import tf2_geometry_msgs.tf2_geometry_msgs as tfmsg
import tf.transformations as tf_conv
from geometry_msgs.msg import TransformStamped
import rospy
import numpy

class AauSpatialReasoner(DiscreteReasoner):
    """
    @brief Reasoner that keeps care of transformations and spatial relations

    Keeps in self._tf_list a list of transforms to publish

    In self._linked_list keeps a list of objects that are linked to a tf already present in the tf system.

    Their pose is regularly updated (when changes)


    >>> sr = AauSpatialReasoner()
    >>> e = Element()
    >>> e2 = Element()
    >>> sr.onAddProperties(e)
    >>> sr.setData(e,  [1.0, 1.0 ,1.0], ":Position")
    >>> sr.setData(e,  [1.0, 1.0 ,1.0], ":OrientationEuler")
    >>> sr.setData(e,  [1.0, 1.0 ,1.0], ":Size")
    >>> sr.onAddProperties(e2)
    >>> sr.computeRelations(e, e2)
    []
    >>> sr.setData(e2,  [3.0, 2.0 ,-1.0], ":Position")
    >>> sr.setData(e2,  [1.0, 1.0 ,1.0], ":OrientationEuler")
    >>> sr.setData(e2,  [1.0, 2.0 ,3.0], ":Size")
    >>> numpy.array(sr.getData(e, ":OrientationEuler"))
    array([ 1.,  1.,  1.])
    >>> sr.computeRelations(e, e2)
    [':pX', ':oY', ':dZ']
    >>> sr.setData(e2,  [4.0, 5.0 , -2.0], ":Position")
    >>> sr.computeRelations(e, e2)
    [':pX', ':pY', ':fZ']
    """
    def parse(self, element, action):
        """
        Called by the world model every time an objects is modified

        Internally, does 2 things: parse to object and it to the tflist
        """
        if element._id=="skiros:Scene-0" or not element.hasProperty("skiros:DiscreteReasoner", value="AauSpatialReasoner"):
            return True
        if action=="add" or action=="update":
            c_rel = element.getRelation(pred=self._spatial_rels, obj="-1")
            if not c_rel:
                log.warn(self.__class__.__name__, "Adding relation to {}".format(element))
                element.addRelation("skiros:Scene-0", "skiros:contain", "-1")
            self._updateTfList(element)
        elif action=="remove":
            if self._linked_list.has_key(element._id):
                del self._linked_list[element._id]
            if self._tf_list.has_key(element._id):
                del self._tf_list[element._id]
        return True

    def _reset(self):
        self._tf_list = {}
        self._linked_list = {}
        self._missing_tf = {}
        root = self._wmi.getElement("skiros:Scene-0")
        self._spatial_rels = self._wmi.getSubProperties("skiros:spatiallyRelated")
        if root.hasProperty("skiros:FrameId"):
            self._base_frame = root.getProperty("skiros:FrameId").value
        else:
            self._base_frame = "map"
            log.info(self.__class__.__name__, "Adding FrameId {} to scene.".format(self._base_frame))
            root.setProperty("skiros:FrameId", self._base_frame)
            self._wmi.updateElement(root, self.__class__.__name__)
        for _, e in self._wmi.getRecursive(root.id, "skiros:spatiallyRelated").iteritems():
            if not e.id=="skiros:Scene-0":
                self._updateTfList(e)

    def _getParentFrame(self, e):
        c_rel = e.getRelation(pred=self._spatial_rels, obj="-1")
        if not c_rel:
            raise Exception("Element {} has not parent. Debug: {}".format(e.printState(), e.printState(True)))
        parent = self._wmi.getElement(c_rel['src'])
        while c_rel and not parent.hasProperty('skiros:FrameId'):
            return self._getParentFrame(parent)
        return str(parent.getProperty("skiros:FrameId").value)

    def _getTransform(self, base_frm, target_frm):
        try:
            tf = self._tlb.lookup_transform(base_frm, target_frm, rospy.Time(0), rospy.Duration(0.0))
            return ((tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z),
                    (tf.transform.rotation.x, tf.transform.rotation.y, tf.transform.rotation.z, tf.transform.rotation.w))
        except:
            missing = base_frm+target_frm
            if not self._missing_tf.has_key(missing):
                log.warn(self.__class__.__name__, "No tf found between {} {}. Linked tf will not be updated.".format(base_frm, target_frm))
                self._missing_tf[missing] = None
            return (None, None)

    def _updateLinkedObjects(self):
        to_update = list()
        for k, _ in list(self._linked_list.iteritems()):
            e = self._wmi.getElement(k)
            base_frm = e.getProperty("skiros:BaseFrameId").value
            linked_frm = e.getProperty("skiros:LinkedToFrameId").value
            new_p, new_o = self._getTransform(base_frm, linked_frm)
            old_p, old_o = e.getData(":Pose")
            if new_p and new_o:
                if old_p[0]==None or old_o[0]==None:
                    e.setData(":Pose", (new_p, new_o))
                    to_update.append(e)
                    continue
                treshold = 0.001
                #print "{} {}".format(self._vector_distance(new_p, old_p), self._vector_distance(new_o, old_o))
                #TODO: vector distance for quaternions doesn't work, need angleShortestPath func
                if self._vector_distance(new_p, old_p)>treshold:
                    e.setData(":Pose", (new_p, new_o))
                    to_update.append(e)
        for e in to_update:
            self._wmi.updateElement(e, self.__class__.__name__)

    def _publishTfList(self):
        for _, e in list(self._tf_list.iteritems()):
            self._tb.sendTransform(e.getData(":TransformMsg"))

    def _vector_distance(self, v1, v2):
        diff = numpy.array(v1)-numpy.array(v2)
        return numpy.linalg.norm(diff)

    def _quaternion_normalize(self, q):
        if numpy.count_nonzero(q)==0:
            q[0] = 1.0
        norm=numpy.linalg.norm(q)
        if norm==0:
           return q
        return q/norm

    def _updateTfList(self, element):
        """
        Add an element to the list of published tfs
        """
        if not element.hasProperty("skiros:PublishTf", value=True):
            return
        if element.hasProperty("skiros:FrameId", value=""):
            element.setProperty("skiros:FrameId", element._id)
        parent_frame = self._getParentFrame(element)
        base_frm = element.getProperty("skiros:BaseFrameId").value
        if base_frm=="":
            element.setProperty("skiros:BaseFrameId", parent_frame)
        elif base_frm != parent_frame:
            log.warn(self.__class__.__name__, "{} transformed from base {} to base {}".format(element, base_frm, parent_frame))
            element.setData(":PoseStampedMsg", self._tlb.transform(element.getData(":PoseStampedMsg"), parent_frame))
        if not self._tf_list.has_key(element._id):
            log.info("[AauSpatialReasoner] Publishing {} parent: {}".format(element, parent_frame))
        if element.hasProperty("skiros:LinkedToFrameId") and not element.hasProperty("skiros:LinkedToFrameId", ""):
            self._linked_list[element._id] = None
        if element.hasData(":Pose"):
            element.setData(":Orientation", self._quaternion_normalize(element.getData(":Orientation")))
            self._tf_list[element._id] = element

    def run(self):
        """ Run the reasoner daemon on the world model """
        self._tlb = tf.Buffer()
        self._tb = tf.TransformBroadcaster()
        self._tl = tf.TransformListener(self._tlb)
        self._reset()
        rate = rospy.Rate(25)
        while not rospy.is_shutdown():
            rate.sleep()
            self._updateLinkedObjects()
            self._publishTfList()

    def onAddProperties(self, element):
        """ Add default reasoner properties to the element """
        if not element.hasProperty("skiros:FrameId"):
            element.setProperty("skiros:FrameId", "")
        if not element.hasProperty("skiros:BaseFrameId"):
            element.setProperty("skiros:BaseFrameId", "")
        if not element.hasProperty("skiros:PublishTf"):
            element.setProperty("skiros:PublishTf", True)
        if not element.hasProperty("skiros:PositionX"):
            element.setProperty("skiros:PositionX", float)
        if not element.hasProperty("skiros:PositionY"):
            element.setProperty("skiros:PositionY", float)
        if not element.hasProperty("skiros:PositionZ"):
            element.setProperty("skiros:PositionZ", float)
        if not element.hasProperty("skiros:OrientationX"):
            element.setProperty("skiros:OrientationX", float)
        if not element.hasProperty("skiros:OrientationY"):
            element.setProperty("skiros:OrientationY", float)
        if not element.hasProperty("skiros:OrientationZ"):
            element.setProperty("skiros:OrientationZ", float)
        if not element.hasProperty("skiros:OrientationW"):
            element.setProperty("skiros:OrientationW", float)
        if not element.hasProperty("skiros:SizeX"):
            element.setProperty("skiros:SizeX", float)
        if not element.hasProperty("skiros:SizeY"):
            element.setProperty("skiros:SizeY", float)
        if not element.hasProperty("skiros:SizeZ"):
            element.setProperty("skiros:SizeZ", float)

    def onRemoveProperties(self, element):
        """ Remove default reasoner properties to the element """
        element.removeProperty("skiros:FrameId")
        element.removeProperty("skiros:BaseFrameId")
        element.removeProperty("skiros:PublishTf")
        element.removeProperty("skiros:PositionX")
        element.removeProperty("skiros:PositionY")
        element.removeProperty("skiros:PositionZ")
        element.removeProperty("skiros:OrientationX")
        element.removeProperty("skiros:OrientationY")
        element.removeProperty("skiros:OrientationZ")
        element.removeProperty("skiros:OrientationW")
        element.removeProperty("skiros:SizeX")
        element.removeProperty("skiros:SizeY")
        element.removeProperty("skiros:SizeZ")

    def hasData(self, element, get_code):
        """ Return true if the data is available in the element """
        if get_code==":Pose" or get_code==":PoseStampedMsg":
            return element.hasData(":Position") and element.hasData(":Orientation")
        elif get_code==":Position":
            return (element.getProperty("skiros:PositionX").value!=None and
                    element.getProperty("skiros:PositionY").value!=None and
                    element.getProperty("skiros:PositionZ").value!=None)
        elif get_code==":Orientation":
            return (element.getProperty("skiros:OrientationX").value!=None and
                    element.getProperty("skiros:OrientationY").value!=None and
                    element.getProperty("skiros:OrientationZ").value!=None and
                    element.getProperty("skiros:OrientationW").value!=None)
        elif get_code==":Size":
            return (element.getProperty("skiros:SizeX").value!=None and
                    element.getProperty("skiros:SizeY").value!=None and
                    element.getProperty("skiros:SizeZ").value!=None)
        else:
            log.error("[AauSpatialReasoner] Code {} not recognized".format(get_code))
            return False

    def getData(self, element, get_code):
        """
        Return data from the element in the format indicated in get_code


        """
        if get_code==":Pose":
            return (element.getData(":Position"), element.getData(":Orientation"))
        elif get_code==":TransformMsg":
            msg = TransformStamped()
            msg.header.frame_id = element.getProperty("skiros:BaseFrameId").value
            msg.header.stamp = rospy.Time.now()
            msg.child_frame_id = element.getProperty("skiros:FrameId").value
            msg.transform.translation.x = element.getProperty("skiros:PositionX").value
            msg.transform.translation.y = element.getProperty("skiros:PositionY").value
            msg.transform.translation.z = element.getProperty("skiros:PositionZ").value
            msg.transform.rotation.x = element.getProperty("skiros:OrientationX").value
            msg.transform.rotation.y = element.getProperty("skiros:OrientationY").value
            msg.transform.rotation.z = element.getProperty("skiros:OrientationZ").value
            msg.transform.rotation.w = element.getProperty("skiros:OrientationW").value
            return msg
        elif get_code==":PoseStampedMsg":
            msg = tfmsg.PoseStamped()
            msg.header.frame_id = element.getProperty("skiros:BaseFrameId").value
            msg.pose.position.x = element.getProperty("skiros:PositionX").value
            msg.pose.position.y = element.getProperty("skiros:PositionY").value
            msg.pose.position.z = element.getProperty("skiros:PositionZ").value
            msg.pose.orientation.x = element.getProperty("skiros:OrientationX").value
            msg.pose.orientation.y = element.getProperty("skiros:OrientationY").value
            msg.pose.orientation.z = element.getProperty("skiros:OrientationZ").value
            msg.pose.orientation.w = element.getProperty("skiros:OrientationW").value
            return msg
        elif get_code==":Position":
            return [element.getProperty("skiros:PositionX").value,
                    element.getProperty("skiros:PositionY").value,
                    element.getProperty("skiros:PositionZ").value]
        elif get_code==":Orientation":
            return [element.getProperty("skiros:OrientationX").value,
                    element.getProperty("skiros:OrientationY").value,
                    element.getProperty("skiros:OrientationZ").value,
                    element.getProperty("skiros:OrientationW").value]
        elif get_code==":OrientationEuler":
            return list(tf_conv.euler_from_quaternion([element.getProperty("skiros:OrientationX").value,
                    element.getProperty("skiros:OrientationY").value,
                    element.getProperty("skiros:OrientationZ").value,
                    element.getProperty("skiros:OrientationW").value]))
        elif get_code==":Size":
            return [element.getProperty("skiros:SizeX").value,
                    element.getProperty("skiros:SizeY").value,
                    element.getProperty("skiros:SizeZ").value]
        else:
            log.error("[AauSpatialReasoner] Code {} not recognized".format(get_code))
            return None

    def setData(self, element, data, set_code):
        """
        Convert user data to reasoner data and store it into given element
        """
        if set_code==":Pose":
            return (element.setData(":Position", data[0]), element.setData(":Orientation", data[1]))
        elif set_code==":TransformMsg":
            element.getProperty("skiros:BaseFrameId").value = str(data.header.frame_id)
            element.getProperty("skiros:PositionX").value = data.transform.translation.x
            element.getProperty("skiros:PositionY").value = data.transform.translation.y
            element.getProperty("skiros:PositionZ").value = data.transform.translation.z
            element.getProperty("skiros:OrientationX").value = data.transform.rotation.x
            element.getProperty("skiros:OrientationY").value = data.transform.rotation.y
            element.getProperty("skiros:OrientationZ").value = data.transform.rotation.z
            element.getProperty("skiros:OrientationW").value = data.transform.rotation.w
        elif set_code==":PoseStampedMsg":
            element.getProperty("skiros:BaseFrameId").value = str(data.header.frame_id)
            element.getProperty("skiros:PositionX").value = data.pose.position.x
            element.getProperty("skiros:PositionY").value = data.pose.position.y
            element.getProperty("skiros:PositionZ").value = data.pose.position.z
            element.getProperty("skiros:OrientationX").value = data.pose.orientation.x
            element.getProperty("skiros:OrientationY").value = data.pose.orientation.y
            element.getProperty("skiros:OrientationZ").value = data.pose.orientation.z
            element.getProperty("skiros:OrientationW").value = data.pose.orientation.w
        elif set_code==":Position":
            element.getProperty("skiros:PositionX").value = data[0]
            element.getProperty("skiros:PositionY").value = data[1]
            element.getProperty("skiros:PositionZ").value = data[2]
        elif set_code==":Orientation":
            element.getProperty("skiros:OrientationX").value = data[0]
            element.getProperty("skiros:OrientationY").value = data[1]
            element.getProperty("skiros:OrientationZ").value = data[2]
            element.getProperty("skiros:OrientationW").value = data[3]
        elif set_code==":OrientationEuler":
            q = tf_conv.quaternion_from_euler(*data)
            element.getProperty("skiros:OrientationX").value = q[0]
            element.getProperty("skiros:OrientationY").value = q[1]
            element.getProperty("skiros:OrientationZ").value = q[2]
            element.getProperty("skiros:OrientationW").value = q[3]
        elif set_code==":Size":
            element.getProperty("skiros:SizeX").value = data[0]
            element.getProperty("skiros:SizeY").value = data[1]
            element.getProperty("skiros:SizeZ").value = data[2]
        else:
            log.error("[AauSpatialReasoner] Code {} not recognized".format(set_code))
            return None

    def getAssociatedRelations(self):
        return [':pX', ':piX', ':mX', ':miX', ':oX', ':oiX', ':sX', ':siX', ':dX', ':diX', ':fX', ':fiX', ':eqX',
                ':pY', ':piY', ':mY', ':miY', ':oY', ':oiY', ':sY', ':siY', ':dY', ':diY', ':fY', ':fiY', ':eqY',
                ':pZ', ':piZ', ':mZ', ':miZ', ':oZ', ':oiZ', ':sZ', ':siZ', ':dZ', ':diZ', ':fZ', ':fiZ', ':eqZ']

    def getAssociatedProperties(self):
        return [':Size', ':Pose', ':Position', ':Orientation', ':OrientationEuler', ':PoseStampedMsg', ':TransformMsg']

    def _isclose(self, a, b, rel_tol=1e-06, abs_tol=0.001):
        """
        Implementation of equality check between floats

        Absolute tolerance set to 0.001 (millimiters)
        """
        return abs(a-b) <= max(rel_tol * max(abs(a), abs(b)), abs_tol)

    def _getAIRelations(self, a1, a2, b1, b2, axis):
        """
        @brief Extract Allen Intervals (1 out of the 13 qualitative relations joined with metrics)

        >>> sr = AauSpatialReasoner()
        >>> sr._getAIRelations(0, 2, 2, 4, 'X')
        [':mX']
        >>> sr._getAIRelations(0, 2, -2, 0, 'X')
        [':miX']
        >>> sr._getAIRelations(0, 2, 0, 2, 'X')
        [':eqX']
        >>> sr._getAIRelations(0, 2, 3, 6, 'X')
        [':pX', 1]
        >>> sr._getAIRelations(3, 6, 0, 2, 'X')
        [':piX', 1]
        >>> sr._getAIRelations(0, 3, 1, 5, 'X')
        [':oX', [1, 2, 2]]
        >>> sr._getAIRelations(1, 5, 0, 3, 'X')
        [':oiX', [1, 2, 2]]
        >>> sr._getAIRelations(0, 2, 0, 6, 'X')
        [':sX', 4]
        >>> sr._getAIRelations(0, 6, 0, 2, 'X')
        [':siX', 4]
        >>> sr._getAIRelations(3, 5, 2, 6, 'X')
        [':dX', [1, 1]]
        >>> sr._getAIRelations(2, 6, 3, 5, 'X')
        [':diX', [1, 1]]
        >>> sr._getAIRelations(3, 6, 0, 6, 'X')
        [':fX', 3]
        >>> sr._getAIRelations(0, 6, 3, 6, 'X')
        [':fiX', 3]
        >>> sr._getAIRelations(0, 0, 3, 3, 'X')
        [':pX', 3]
        >>> sr._getAIRelations(3, 3, 3, 3, 'X')
        [':mX']
        >>> sr._getAIRelations(4, 4, 3, 3, 'X')
        [':piX', 1]
        >>> sr._getAIRelations(0.01, 0.01, 0, 1, 'X')
        [':dX', [0.01, 0.99]]
        """
#        m = min([a1, a2, b1, b2])
#        if m<0:
#            a1 -= m
#            a2 -= m
#            b1 -= m
#            b2 -= m
        #print "{}: {} {} {} {}".format(axis, a1, a2, b1, b2)
        if a2<=b2:
            if a2<=b1:
                if self._isclose(a2, b1):
                    return [':m'+axis]
                else:
                    return [':p'+axis, b1-a2]
            else:
                if a1<=b1:
                    if self._isclose(a1, b1):
                        if self._isclose(a2, b2):
                            return [':eq'+axis]
                        else:
                            return [':s'+axis, b2-a2]
                    else:
                        if self._isclose(a2, b2):
                            return [':fi'+axis, b1-a1]
                        else:
                            return [':o'+axis, [b1-a1, a2-b1, b2-a2]]
                else:
                    if self._isclose(a2, b2):
                        return [':f'+axis, a1-b1]
                    else:
                        return [':d'+axis, [a1-b1, b2-a2]]
        else:
            if a1<=b2:
                if a1<=b1:
                    if self._isclose(a1, b1):
                        return [':si'+axis, a2-b2]
                    else:
                        return [':di'+axis, [b1-a1, a2-b2]]
                else:
                    if self._isclose(a1, b2):
                        return [':mi'+axis]
                    else:
                        return [':oi'+axis, [a1-b1, b2-a1, a2-b2]]
            else:
                return [':pi'+axis, a1-b2]


    def computeRelations(self, sub, obj):
        to_ret = []
        #TODO: transform pose
        #Get corners a1,a2,b1,b2
        sp = numpy.array([0, 0, 0])
        ss = numpy.array(self.getData(sub, ":Size"))
        if ss[0]==None:
            ss = numpy.array([0, 0, 0])
        a1 = sp-ss
        a2 = sp+ss
        op = numpy.array(self.getData(obj, ":Position"))
        os = numpy.array(self.getData(obj, ":Size"))
        if not op[0]:
            return to_ret
        if os[0]==None:
            os = numpy.array([0, 0, 0])
        b1 = op-os
        b2 = op+os
        #Calculates allen intervals for the 3 axes
        to_ret.append(self._getAIRelations(a1[0], a2[0], b1[0], b2[0], 'X')[0])
        to_ret.append(self._getAIRelations(a1[1], a2[1], b1[1], b2[1], 'Y')[0])
        to_ret.append(self._getAIRelations(a1[2], a2[2], b1[2], b2[2], 'Z')[0])
        #print to_ret
        return to_ret
