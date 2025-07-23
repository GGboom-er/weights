# coding:utf-8
import json
import stat
import pymel.core as pm
from .api_lib import weight_tool_api
import os


def convert_p(p):
    unit = {"mm": 0.1, "cm": 1.0, "m": 100.0, "in": 2.45, "ft": 7.62, "yd": 91.44}.get(pm.currentUnit(q=1, l=1))
    return [p[i]*unit for i in range(3)]


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


def get_paint_joint(sk):
    if sk is None:
        return
    paint_joints = [joint for joint in sk.paintTrans.inputs() if joint.type() == "joint"]
    if len(paint_joints) != 1:
        return pm.mel.warning("\nyou need a paint joint")
    return paint_joints[0]


def get_unlock_joint(sk):
    if sk is None:
        return
    unlock_joints = [joint for joint in sk.getInfluence()
                     if joint.type() == "joint" and not joint.liw.get()]
    if len(unlock_joints) != 1:
        return pm.mel.warning("\nyou need a unlock joint")
    unlock_joint = unlock_joints[0]
    return unlock_joint


def clear_orig(polygon):
    for shape in polygon.getShapes():
        if shape.io.get():
            if not shape.outputs(type="groupParts"):
                pm.delete(shape)


def copy_weights():
    selected = pm.selected(o=1)
    if len(selected) < 2:
        return pm.warning(u"please select two object")
    if len(selected) > 2:
        src = selected[0]
        for dst in selected[1:]:
            pm.select(src, dst)
            copy_weights()
        return
    src, dst = selected
    if src.type() == "transform" and not src.getShape():
        name_polygon = {children.name().split("|")[-1].split(":")[-1]: children
                        for children in src.getChildren(type="transform")}
        for children in dst.getChildren(type="transform"):
            name = children.split("|")[-1].split(":")[-1]
            if name in name_polygon:
                pm.select(name_polygon[name], children)
                copy_weights()
        return
    src, dst = assert_geometry(src, "mesh"), assert_geometry(dst, "mesh")
    clear_orig(dst)
    if not all([src, dst]):
        return pm.warning(u"please select two polygon")
    skin_clusters = src.history(type="skinCluster")
    if len(skin_clusters) == 0:
        return pm.warning(u"please select two skin polygon")
    if dst.history(type="skinCluster"):
        return pm.mel.CopySkinWeights()
    pm.skinCluster(skin_clusters[0].getInfluence(), dst, tsb=1)
    pm.select(src, dst)
    pm.mel.CopySkinWeights()


def copy_unlock_skin_polygon():
    polygon = assert_geometry(shape_type="mesh")
    sk = get_skin_cluster(polygon)
    influences = sk.getInfluence()
    joints = [joint for joint in influences if not joint.liw.get()]
    dup_polygon = pm.duplicate(polygon)[0]
    dup_polygon.rename("_".join([joint.name().split("|")[-1].split(":")[-1] for joint in joints[:3]]))
    pm.skinCluster(joints, dup_polygon, mi=1, tsb=1)
    pm.select(polygon, dup_polygon)
    pm.mel.CopySkinWeights()
    pm.select(dup_polygon)


def re_skin():
    polygons = filter(assert_geometry,  pm.ls(sl=1, et="transform"))
    skin_cluster_list = [sk for sk in map(get_skin_cluster, polygons) if sk]
    for sk in skin_cluster_list:
        for src, dst in sk.matrix.inputs(c=1,  p=1, type="joint"):
            index = src.logicalIndex()
            sk.bindPreMatrix[index].set(src.get().inverse())
    joints = pm.ls(sl=1, type="joint")

    for joint in joints:
        for attr in joint.worldMatrix[0].outputs(type="skinCluster", p=1):
            index = attr.logicalIndex()
            attr.node().bindPreMatrix[index].set(attr.get().inverse())


def create_vx():
    for joint in pm.ls(sl=1, type="joint"):
        loc = pm.spaceLocator(n=joint.name()+"LushBezierVX")
        loc.setMatrix(joint.getMatrix(ws=1), ws=1)


def default_init_var():
    polygon = assert_geometry(shape_type="mesh")
    sk = get_skin_cluster(polygon)
    if sk is None:
        return polygon, sk, None, None, None
    paint_joint = get_paint_joint(sk)
    joints = sk.getInfluence()
    unlock_joints = [joint for joint in joints if joint.type() == "joint" and not joint.liw.get()]
    return polygon, sk, joints, unlock_joints, paint_joint


def paint_eye():
    polygon, sk, joints, unlock_joints, paint_joint = default_init_var()
    if not unlock_joints:
        return
    joint_points = [convert_p(joint.getTranslation(space="world")) for joint in unlock_joints]
    indices = [joints.index(unlock_joint) for unlock_joint in unlock_joints]
    weight_tool_api.paint_eye(polygon.name(), sk.name(), indices, joint_points)


def get_json_kwargs(fun):
    path = os.path.abspath("%s/../data/kwargs/%s.json" % (__file__, fun.__name__)).replace("\\", "/")
    if os.path.isfile(path):
        with open(path, "r") as fp:
            return json.load(fp)
    return dict()


def cache_kwargs(fun):
    def new_fun(**kwargs):
        if len(kwargs.keys()) == 0:
            fun(**get_json_kwargs(fun))
        else:
            path = os.path.abspath("%s/../data/kwargs/%s.json" % (__file__, fun.__name__)).replace("\\", "/")
            dir_path = os.path.dirname(path)
            if not os.path.isdir(dir_path):
                os.makedirs(dir_path)
            if os.path.isfile(path):
                os.chmod(path, stat.S_IWRITE)
            with open(path, "w") as fp:
                json.dump(kwargs, fp)
            fun(**kwargs)

    new_fun.__name__ = fun.__name__
    return new_fun


@cache_kwargs
def limit_max_influence(max_influence=4):
    polygon = assert_geometry(shape_type="mesh")
    sk = get_skin_cluster(polygon)
    indexes = list(range(len(sk.getInfluence())))
    weight_tool_api.limit_max_influence(polygon.name(), sk.name(), indexes, max_influence)


@cache_kwargs
def lock_influence_smooth(smooth_count=1, smooth_step=0.2):
    polygon, sk, joints, unlock_joints, paint_joint = default_init_var()
    if sk is None:
        return
    vtx_ids = [vtx.index() for vtx in pm.ls(sl=1, fl=11) if isinstance(vtx, pm.general.MeshVertex)]
    indexes = list(range(len(sk.getInfluence())))
    weight_tool_api.lock_influence_smooth(polygon.name(), sk.name(), indexes, vtx_ids, smooth_step, smooth_count)


def copy_joint_weight():
    polygon, sk, joints, unlock_joints, paint_joint = default_init_var()
    if not unlock_joints:
        return
    indices = [joints.index(unlock_joint) for unlock_joint in unlock_joints]
    joint_names = [unlock_joint.name() for unlock_joint in unlock_joints]
    print("copy star")
    print(weight_tool_api.copy_joint_weight)
    print("copy end 50")
    weight_tool_api.copy_joint_weight(polygon.name(), sk.name(), indices, joint_names)
    print("copy end")


def paste_joint_weight():
    polygon, sk, joints, unlock_joints, paint_joint = default_init_var()
    if not unlock_joints:
        return
    indices = [joints.index(unlock_joint) for unlock_joint in unlock_joints]
    joint_names = [unlock_joint.name() for unlock_joint in unlock_joints]
    weight_tool_api.paste_joint_weight(polygon.name(), sk.name(), indices, joint_names)
