# coding:utf-8
import functools

import pymel.core as pm
from .api_lib import bezier_api


def convert_p(p):
    unit = {"mm": 0.1, "cm": 1.0, "m": 100.0, "in": 2.45, "ft": 7.62, "yd": 91.44}.get(pm.currentUnit(q=1, l=1))
    return [p[i]*unit for i in range(3)]


def convert_length(l):
    unit = {"mm": 0.1, "cm": 1.0, "m": 100.0, "in": 2.45, "ft": 7.62, "yd": 91.44}.get(pm.currentUnit(q=1, l=1))
    return l*unit


def convert_y(y):
    unit = {"mm": 0.1, "cm": 1.0, "m": 100.0, "in": 2.45, "ft": 7.62, "yd": 91.44}.get(pm.currentUnit(q=1, l=1))
    return y/unit


def ik_x(v, p):
    u"""
    :param v: 向量
    :param p: 点坐标
    :return: 点p在向量v轴上的坐标值。或理解为点p到向量v所在直线的最近点与远点的距离
    """
    return sum(v[i] * p[i] for i in range(3)) / sum(v[i] ** 2 for i in range(3))


def vp_to_vx(v, p):
    return [v[0], v[1], v[2], ik_x(v, p)]


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


def default_init_var():
    polygon = assert_geometry(shape_type="mesh")
    sk = get_skin_cluster(polygon)
    if sk is None:
        return polygon, sk, None, None, None
    paint_joint = get_paint_joint(sk)
    joints = sk.getInfluence()
    unlock_joints = [joint for joint in joints if joint.type() == "joint" and not joint.liw.get()]
    return polygon, sk, joints, unlock_joints, paint_joint


def ik_init():
    polygon, sk, joints, unlock_joints, paint_joint = default_init_var()
    if len(unlock_joints) == 0 or paint_joint is None:
        return
    vx = spine_vx(unlock_joints[0], paint_joint)
    bezier_api.ik_init(polygon.name(), sk.name(), vx)
    return True


def soft_init():
    polygon, sk, joints, unlock_joints, paint_joint = default_init_var()
    if sk is None:
        return
    pm.softSelect(sse=1)
    pm.softSelect(ssc="0,1,1,1,0,1")
    radius = pm.softSelect(q=1, ssd=1)
    pm.softSelect(ssd=radius * 2)
    up_cmd = "move -r -os -wd 0 %.6f 0 ;" % convert_y(1)
    dn_cmd = "move -r -os -wd 0 %.6f 0 ;" % -convert_y(1)
    bezier_api.soft_init(polygon.name(), sk.name(), up_cmd, dn_cmd)
    pm.softSelect(ssd=radius)
    sk.ptw.get(sk.ptw.get())
    pm.dgdirty(sk)
    pm.refresh()
    return True


def ik_solve(xs, ys, r):
    r = pm.softSelect(q=1, ssd=1) * r * 2
    bezier_api.solve("ik", xs, ys, r)


def soft_solve(xs, ys, r):
    ys = [1 - y for y in ys]
    bezier_api.solve("soft", xs, ys, r)


def locator_vx(fun):
    def _fun(joint1, joint2):
        vx_nodes = pm.ls(joint2.name()+"LushBezierVX", type="transform")
        if len(vx_nodes) == 1:
            vx_node = vx_nodes[0]
            p = convert_p(joint2.getTranslation(space="world"))
            v = vx_node.getMatrix(ws=1)[0][:3]
            return vp_to_vx(v, p)
        else:
            return fun(joint1, joint2)
    return _fun


@locator_vx
def spine_vx(joint1, joint2):
    p = convert_p(joint2.getTranslation(space="world"))
    v = [(i + j) / 2 for i, j in zip(joint1.getMatrix(ws=1)[0][:3], joint2.getMatrix(ws=1)[0][:3])]
    if (pm.datatypes.Point(p) * joint1.getMatrix(ws=1).inverse())[0] < 0:
        v = [-i for i in v]
    return vp_to_vx(v, p)


@locator_vx
def brow_vx(joint1, joint2):
    p1, p2 = joint1.getTranslation(space="world"), joint2.getTranslation(space="world")
    p2[1] = p1[1]
    p2[2] = p1[2]
    p = (p1 + p2) / 2
    v = (p2 - p1).normal()
    p = convert_p(p)
    return vp_to_vx(v, p)


@locator_vx
def belt_vx(joint1, joint2):
    p1, p2 = joint1.getTranslation(space="world"), joint2.getTranslation(space="world")
    p = (p1 + p2) / 2
    v = (p2 - p1).normal()
    p = convert_p(p)
    return vp_to_vx(v, p)


def split_init(typ):
    polygon, sk, joints, unlock_joints, paint_joint = default_init_var()
    if paint_joint is None or unlock_joints is None:
        return
    if typ == "brow":
        unlock_joints.sort(key=lambda x: x.getTranslation(space="world").x)
    else:
        unlock_joints.sort(key=lambda x: x.name())
        unlock_joints.sort(key=lambda x: len(x.fullPath()))
    vxs = [globals()[typ+"_vx"](unlock_joints[i], joint) for i, joint in enumerate(unlock_joints[1:])]
    vxs = sum(vxs, [])
    indices = [joints.index(unlock_joint) for unlock_joint in unlock_joints]
    bezier_api.split_init(polygon.name(), sk.name(), indices, vxs)
    return True


def split_solve(xs, ys, r):
    r = pm.softSelect(q=1, ssd=1) * r * 2
    bezier_api.solve("split", xs, ys, r)


brow_init = functools.partial(split_init, "brow")
spine_init = functools.partial(split_init, "spine")
belt_init = functools.partial(split_init, "belt")
brow_solve = split_solve
spine_solve = split_solve
belt_solve = split_solve


def points_init():
    polygon, sk, joints, unlock_joints, paint_joint = default_init_var()
    if unlock_joints is None:
        return
    indices = [joints.index(unlock_joint) for unlock_joint in unlock_joints]
    joint_points = [convert_p(joint.getTranslation(space="world")) for joint in unlock_joints]
    print(polygon.name(), sk.name())
    bezier_api.points_init(polygon.name(), sk.name(), indices, joint_points)
    return True


def points_solve(xs, ys, r):
    r = pm.softSelect(q=1, ssd=1) * r
    ys = [1 - y for y in ys]
    bezier_api.solve("points", xs, ys, r)


def get_joint_chains(unlock_joints):
    joint_length = len(unlock_joints)
    un_use_joints = sorted(unlock_joints, key=lambda x: len(x.fullPath()))
    joint_chains = []
    for i in range(joint_length):
        joint_chain = [un_use_joints.pop(0)]
        joint_chain += [joint for joint in un_use_joints if joint.fullPath().startswith(joint_chain[0].fullPath())]
        un_use_joints = [joint for joint in un_use_joints if joint not in joint_chain]
        joint_chains.append(joint_chain)
        if len(un_use_joints) == 0:
            break
    return joint_chains


def chains_init():
    polygon, sk, joints, unlock_joints, paint_joint = default_init_var()
    if paint_joint is None or unlock_joints is None:
        return
    joint_chains = get_joint_chains(unlock_joints)
    indices = [joints.index(unlock_joint) for unlock_joint in unlock_joints]
    top_indices = [joints.index(joint_chain[0]) for joint_chain in joint_chains]
    point_data = [[convert_p(joint.getTranslation(space="world"))
                   for joint in joint_chain] for joint_chain in joint_chains]
    chin_lengths = [len(joint_chain) for joint_chain in joint_chains]
    joint_points = sum(point_data, [])
    bezier_api.chains_init(polygon.name(), sk.name(), indices, joint_points, chin_lengths, top_indices)
    return True


def lines_init():
    polygon, sk, joints, unlock_joints, paint_joint = default_init_var()
    if paint_joint is None or unlock_joints is None:
        return
    joint_chains = get_joint_chains(unlock_joints)
    joint_points = []
    indices = []
    for joint_chain in joint_chains:
        for i, joint in enumerate(joint_chain[1:]):
            indices.append(joints.index(joint_chain[i]))
            p1 = convert_p(joint_chain[i].getTranslation(space="world"))
            p2 = joint.getTranslation(space="world")
            joint_points.append(convert_p((p1+p2)/2))
    bezier_api.points_init(polygon.name(), sk.name(), indices, joint_points)
    return True


chains_solve = points_solve
lines_solve = points_solve


def finger_init():
    polygon, sk, joints, unlock_joints, paint_joint = default_init_var()
    if paint_joint is None or unlock_joints is None:
        return
    joint_chains = get_joint_chains(unlock_joints)
    indices = []
    vxs = []
    for joint_chain in joint_chains:
        for joint in joint_chain:
            indices.append(joints.index(joint))
        for i, joint in enumerate(joint_chain[1:]):
            vxs.append(spine_vx(joint_chain[i], joint))
    vxs = sum(vxs, [])
    chin_lengths = [len(joint_chain) for joint_chain in joint_chains]
    bezier_api.finger_init(polygon.name(), sk.name(), indices, vxs, chin_lengths)
    return True


def finger_solve(xs, ys, r):
    r = pm.softSelect(q=1, ssd=1) * r * 2
    bezier_api.solve("finger", xs, ys, r)


def paint(typ, xs=(0, 0.33, 0.67, 1), ys=(0, 0, 1, 1), r=1):
    if globals()[typ+"_init"]():
        globals()[typ+"_solve"](xs, ys, r)


def init(typ):
    return globals()[typ+"_init"]()


def solve(typ, xs=(0, 0.33, 0.67, 1), ys=(0, 0, 1, 1), r=1):
    globals()[typ + "_solve"](xs, ys, r)

