# coding:utf-8
import pymel.core as pm
from .api_lib import sculpt_api


def get_shape(polygon):
    for shape in polygon.getShapes():
        if not shape.io.get():
            return shape


def assert_geometry(geometry=None, shape_type="mesh"):
    u"""
    :param geometry: 几何体
    :param shape_type: 形节点类型
    :return:
    判断物体是否为集合体
    """
    if geometry is None:
        selected = pm.selected(o=1)
        if len(selected) == 0:
            return pm.warning("please select a " + shape_type)
        geometry = selected[0]
    if geometry.type() == shape_type:
        return geometry.getParent()
    if geometry.type() != "transform":
        return pm.warning("please select a " + shape_type)
    shape = get_shape(geometry)
    if not shape:
        return pm.warning("please select a " + shape_type)
    if shape.type() != shape_type:
        return pm.warning("please select a " + shape_type)
    return geometry


def get_skin_cluster(polygon=None):
    u"""
    :param polygon: 多边形
    :return: 蒙皮节点
    """
    if polygon is None:
        polygon = assert_geometry(shape_type="mesh")
    if polygon is None:
        return
    for history in polygon.history(type="skinCluster"):
        return history
    pm.warning("\ncan not find skinCluster")


class LSculptJob(object):

    def __init__(self):
        self.attr_name = None

    def __repr__(self):
        return self.__class__.__name__

    def __call__(self):
        sculpt_api.part_sdr_solve()
        pm.scriptJob(attributeChange=[self.attr_name, self], runOnce=True)

    def add_job(self):
        self.del_job()
        skin_polygon = assert_geometry()
        if skin_polygon is None:
            return
        skin_cluster = get_skin_cluster(skin_polygon)
        if skin_cluster is None:
            return
        influences = skin_cluster.getInfluence()
        unlock_joints = [joint for joint in influences if not joint.liw.get()]
        unlock_joint_ids = [influences.index(joint) for joint in unlock_joints]
        if len(unlock_joint_ids) == 0:
            return pm.warning("please unlock some joints")
        group = pm.group(em=1, n="LushSplitSculptGroup")
        sculpt_polygon = skin_polygon.duplicate()[0]
        for shape in sculpt_polygon.getShapes():
            if shape.io.get():
                pm.delete(shape)
        sculpt_polygon.setParent(group)
        sculpt_polygon.rename("LushSplitSculpt_"+skin_polygon.name().split(":")[-1].split("|")[-1])
        for shape in sculpt_polygon.getShapes():
            shape.overrideEnabled.set(True)
            shape.overrideColor.set(13)
        panels = pm.getPanel(all=True)
        for panel in panels:
            if pm.modelPanel(panel, ex=1):
                pm.modelEditor(panel, e=1, wireframeOnShaded=True)
        skin_polygon.v.set(0)
        pm.select(cl=1)
        sculpt_api.part_sdr_init(skin_polygon.name(), skin_cluster.name(), sculpt_polygon.name(), unlock_joint_ids)
        self.attr_name = sculpt_polygon.getShape().outMesh.name()
        pm.scriptJob(attributeChange=[self.attr_name, self], runOnce=True)

    def del_job(self):
        for job in pm.scriptJob(listJobs=True):
            if repr(self) in job:
                pm.scriptJob(kill=int(job.split(":")[0]))
        if not pm.objExists("|LushSplitSculptGroup"):
            return
        group = pm.PyNode("|LushSplitSculptGroup")
        for child in group.getChildren():
            name = child.name().split(":")[-1].split("|")[-1]
            if not name.startswith("LushSplitSculpt_"):
                continue
            polygon_name = name[len("LushSplitSculpt_"):]
            for polygon in pm.ls(polygon_name, type="transform"):
                polygon = assert_geometry(polygon)
                if polygon is None:
                    continue
                polygon.v.set(True)
        pm.delete(group)


def auto_apply():
    if not pm.objExists("|LushSplitSculptGroup"):
        LSculptJob().add_job()
    else:
        LSculptJob().del_job()
