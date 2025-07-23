# coding:utf-8
from maya.OpenMaya import *
from maya.OpenMayaAnim import *


def bezier_v(p, t):
    u"""
    :param p: [float, float, float, float] 贝塞尔曲线x/y轴控制点
    :param t: 曲线上位置t/param
    :return: x/y轴数值
    """
    return p[0]*(1-t)**3.0 + 3*p[1]*t*(1-t)**2 + 3*p[2]*t**2*(1-t) + p[3]*t**3


def bezier_t(p, v):
    u"""
    :param p: [float, float, float, float] 贝塞尔曲线x/y轴控制点
    :param v: x/y轴数值
    :return: 曲线上位置t/param
    """
    min_t = 0.0
    max_t = 1.0
    while True:
        t = (min_t+max_t)/2.0
        error_range = bezier_v(p, t) - v
        if error_range > 0.0001:
            max_t = t
        elif error_range < -0.0001:
            min_t = t
        else:
            return t


def get_weight(x, xs, ys):
    u"""
    :param x: x轴坐标
    :param xs: [float, float, float, float] 贝塞尔曲线x轴控制点
    :param ys: [float, float, float, float] 贝塞尔曲线y轴控制点
    :return: weight/y轴坐标
    """
    if x <= 0.0:
        return ys[0]
    elif x >= 1.0:
        return ys[3]
    t = bezier_t(xs, x)
    return bezier_v(ys, t)


def ik_x(v, p):
    u"""
    :param v: 向量
    :param p: 点坐标
    :return: 点p在向量v轴上的坐标值。或理解为点p到向量v所在直线的最近点与远点的距离
    """
    return sum(v[i] * p[i] for i in range(3)) / sum(v[i] ** 2 for i in range(3))


def distance(p1, p2):
    u"""
    :param p1: 点坐标
    :param p2: 点坐标
    :return: 模型点到选择点之间的距离
    """
    return sum([(p1[i]-p2[i])**2 for i in range(3)])**0.5


def sub_point(p1, p2):
    return [p1[i]-p2[i] for i in range(3)]


def dot(list1, list2):
    return sum([elem1*elem2 for elem1, elem2 in zip(list1, list2)])


def get_near_point(p1, p2, p3):

    if dot(sub_point(p1, p2), sub_point(p3, p2)) < 0:
        return p2
    if dot(sub_point(p1, p3), sub_point(p2, p3)) < 0:
        return p3
    v = sub_point(p3, p2)
    x = dot(v, sub_point(p1, p2)) / dot(v, v)
    p4 = [p2[i] + v[i]*x for i in range(3)]
    return p4


def chain_distance(vtx_point, joint_points, first_index, chain_length, debug=False):
    min_distance = distance(joint_points[first_index], vtx_point)
    for chain_id in range(chain_length-1):
        near_point = get_near_point(vtx_point, joint_points[first_index+chain_id], joint_points[first_index+chain_id+1])
        near_distance = distance(vtx_point, near_point)
        if near_distance < min_distance:
            min_distance = near_distance

    return min_distance


def get_dag_path_by_name(name):
    selection_list = MSelectionList()
    selection_list.add(name)
    dag_path = MDagPath()
    selection_list.getDagPath(0, dag_path)
    return dag_path


def get_fn_mesh_by_name(name):
    fn_mesh = MFnMesh(get_dag_path_by_name(name))
    return fn_mesh


def get_fn_skin_by_name(name):
    selection_list = MSelectionList()
    selection_list.add(name)
    depend_node = MObject()
    selection_list.getDependNode(0, depend_node)
    fn_skin = MFnSkinCluster(depend_node)
    return fn_skin


def get_component_by_ids(ids):
    fn_component = MFnSingleIndexedComponent()
    part_component_obj = fn_component.create(MFn.kMeshVertComponent)
    fn_component.addElements(ids)
    return part_component_obj


def get_dag_path_component_by_name(name):
    selection_list = MSelectionList()
    selection_list.add(name)
    dag_path = MDagPath()
    components = MObject()
    selection_list.getDagPath(0, dag_path, components)
    return dag_path, components


def set_paint_weights(skin_cluster_name, paint_weights):
    selection_list = MSelectionList()
    selection_list.add(skin_cluster_name)
    depend_node = MObject()
    selection_list.getDependNode(0, depend_node)
    fn_depend_node = MFnDependencyNode(depend_node)
    ptw_plug = fn_depend_node.findPlug("ptw")
    double_array_data = MFnDoubleArrayData()
    double_array_object = double_array_data.create(paint_weights)
    ptw_plug.setMObject(double_array_object)


def get_weights(polygon_name, skin_cluster_name, indices, weights):
    skin_polygon_path, skin_polygon_components = get_dag_path_component_by_name(polygon_name + ".vtx[*]")
    fn_skin = get_fn_skin_by_name(skin_cluster_name)
    fn_skin.getWeights(skin_polygon_path, skin_polygon_components, indices, weights)


def set_weights(polygon_name, skin_cluster_name, indices, weights):
    MGlobal.executeCommand("dgdirty %s;" % skin_cluster_name)
    skin_polygon_path, skin_polygon_components = get_dag_path_component_by_name(polygon_name + ".vtx[*]")
    fn_skin = get_fn_skin_by_name(skin_cluster_name)
    fn_skin.setWeights(skin_polygon_path, skin_polygon_components, indices, weights)


def get_max_weights(polygon_name, skin_cluster_name, indices, max_weights):
    weights = MDoubleArray()
    get_weights(polygon_name, skin_cluster_name, indices, weights)
    joint_length = len(indices)
    vtx_length = weights.length()/joint_length
    max_weights.clear()
    max_weights.setLength(vtx_length)
    for vtx_id in range(vtx_length):
        max_weights[vtx_id] = 0
        weight_id = vtx_id * joint_length
        for joint_id in range(joint_length):
            max_weights[vtx_id] += weights[weight_id+joint_id]


class WeightSolve(object):

    def __init__(self):
        self.polygon_name = ""
        self.skin_cluster_name = ""
        self.wxs = MDoubleArray()
        self.max_weights = MDoubleArray()
        self.indices = MIntArray()
        self.chain_lengths = MIntArray()

    def ik_init(self, polygon_name, skin_cluster_name, vx):
        self.skin_cluster_name = skin_cluster_name
        skin_fn_mesh = get_fn_mesh_by_name(polygon_name)
        points = MPointArray()
        skin_fn_mesh.getPoints(points)
        vtx_length = points.length()
        self.wxs.clear()
        self.wxs.setLength(vtx_length)
        for vtx_id in range(vtx_length):
            self.wxs[vtx_id] = ik_x(vx, points[vtx_id]) - vx[3]

    def ik_solve(self, xs, ys, r):
        vtx_length = self.wxs.length()
        paint_weights = MDoubleArray()
        paint_weights.setLength(vtx_length)
        for vtx_id in range(vtx_length):
            paint_weights[vtx_id] = get_weight(self.wxs[vtx_id] / r + 0.5, xs, ys)
        set_paint_weights(self.skin_cluster_name, paint_weights)

    def soft_init(self, polygon_name, skin_cluster_name, up_cmd, dn_cmd):
        self.skin_cluster_name = skin_cluster_name
        skin_fn_mesh = get_fn_mesh_by_name(polygon_name)
        old_points = MPointArray()
        new_points = MPointArray()
        skin_fn_mesh.getPoints(old_points)
        MGlobal.executeCommand(up_cmd)
        skin_fn_mesh.getPoints(new_points)
        MGlobal.executeCommand(dn_cmd)
        vtx_length = old_points.length()
        self.wxs.clear()
        self.wxs.setLength(vtx_length)
        for vtx_id in range(vtx_length):
            self.wxs[vtx_id] = 2 * (1 - (new_points[vtx_id][1] - old_points[vtx_id][1]))

    def soft_solve(self, xs, ys, r):
        vtx_length = self.wxs.length()
        paint_weights = MDoubleArray()
        paint_weights.setLength(vtx_length)
        for vtx_id in range(vtx_length):
            paint_weights[vtx_id] = get_weight(self.wxs[vtx_id] / r, xs, ys)
        set_paint_weights(self.skin_cluster_name, paint_weights)

    def solve(self, typ, xs, ys, r):
        if typ == "ik":
            self.ik_solve(xs, ys, r)
        elif typ == "soft":
            self.soft_solve(xs, ys, r)
        elif typ == "split":
            self.split_solve(xs, ys, r)
        elif typ == "points":
            self.points_solve(xs, ys, r)
        elif typ == "finger":
            self.finger_solve(xs, ys, r)

    def split_init(self, polygon_name, skin_cluster_name, indices, vxs):
        self.polygon_name = polygon_name
        self.skin_cluster_name = skin_cluster_name
        self.set_indexes(indices)
        get_max_weights(polygon_name, skin_cluster_name, self.indices, self.max_weights)
        points = MPointArray()
        skin_fn_mesh = get_fn_mesh_by_name(polygon_name)
        skin_fn_mesh.getPoints(points)
        self.wxs.clear()
        split_length = len(indices) - 1
        vtx_length = self.max_weights.length()
        self.wxs.setLength(split_length*vtx_length)
        for vtx_id in range(vtx_length):
            wxs_id = vtx_id * split_length
            for split_id in range(split_length):
                vx_id = split_id * 4
                v = [vxs[vx_id+0], vxs[vx_id+1], vxs[vx_id+2]]
                x = vxs[vx_id+3]
                self.wxs[wxs_id+split_id] = ik_x(v, points[vtx_id]) - x

    def set_indexes(self, indices):
        self.indices.clear()
        for i in indices:
            self.indices.append(i)

    def split_solve(self, xs, ys, r):
        vtx_length = self.max_weights.length()
        split_length = self.wxs.length()/vtx_length
        joint_length = split_length + 1
        weights = MDoubleArray(vtx_length*joint_length, 0)
        for vtx_id in range(vtx_length):
            weight_id = vtx_id * joint_length
            weights[weight_id] = 1.0
            wx_id = vtx_id*split_length
            for split_id in range(split_length):
                w1 = weight_id + split_id
                w2 = w1 + 1
                weights[w2] = get_weight(self.wxs[wx_id+split_id] / r + 0.5, xs, ys)
                if weights[w2] >= weights[w1]:
                    weights[w2] = weights[w1]
                weights[w1] = weights[w1] - weights[w2]
            for joint_id in range(joint_length):
                weights[weight_id+joint_id] = weights[weight_id+joint_id] * self.max_weights[vtx_id]
        set_weights(self.polygon_name, self.skin_cluster_name, self.indices, weights)

    def points_init(self, polygon_name, skin_cluster_name, indices, joint_points):
        self.polygon_name = polygon_name
        self.skin_cluster_name = skin_cluster_name
        self.set_indexes(indices)
        get_max_weights(polygon_name, skin_cluster_name, self.indices, self.max_weights)
        points = MPointArray()
        skin_fn_mesh = get_fn_mesh_by_name(polygon_name)
        skin_fn_mesh.getPoints(points)
        self.wxs.clear()
        vtx_length = self.max_weights.length()
        joint_length = len(indices)
        self.wxs.setLength(vtx_length*joint_length)
        for vtx_id in range(vtx_length):
            wxs_id = vtx_id * joint_length
            for joint_id in range(joint_length):
                self.wxs[wxs_id+joint_id] = distance(joint_points[joint_id], points[vtx_id])

    def points_solve(self, xs, ys, r):
        vtx_length = self.max_weights.length()
        joint_length = self.indices.length()
        weights = MDoubleArray(vtx_length*joint_length, 0)
        for vtx_id in range(vtx_length):
            weight_id = vtx_id * joint_length
            vtx_weight = 0
            for joint_id in range(joint_length):
                index = weight_id + joint_id
                weights[index] = get_weight(self.wxs[index] / r, xs, ys)
                vtx_weight += weights[index]
            if vtx_weight < 0.00001:
                vtx_weight = 1.0
                near_joint_id = 0
                near_distance = self.wxs[weight_id]
                for joint_id in range(joint_length):
                    index = weight_id + joint_id
                    weights[index] = 0.0
                    current_distance = self.wxs[index]
                    if current_distance < near_distance:
                        near_distance = current_distance
                        near_joint_id = joint_id
                weights[weight_id + near_joint_id] = 1.0
            for joint_id in range(joint_length):
                index = vtx_id * joint_length + joint_id
                weights[index] = weights[index] / vtx_weight * self.max_weights[vtx_id]
        set_weights(self.polygon_name, self.skin_cluster_name, self.indices, weights)

    def chains_init(self, polygon_name, skin_cluster_name, indices, joint_points, chain_lengths,
                    top_indices):
        self.polygon_name = polygon_name
        self.skin_cluster_name = skin_cluster_name
        self.set_indexes(indices)
        get_max_weights(polygon_name, skin_cluster_name, self.indices, self.max_weights)
        vtx_length = self.max_weights.length()
        joint_length = self.indices.length()
        weights = MDoubleArray(vtx_length*joint_length, 0)
        top_id = 0
        for joint_id in range(joint_length):
            if indices[joint_id] == top_indices[0]:
                top_id = joint_id
        for vtx_id in range(vtx_length):
            weights[vtx_id*joint_length+top_id] = 1.0
        set_weights(self.polygon_name, self.skin_cluster_name, self.indices, weights)
        points = MPointArray()
        skin_fn_mesh = get_fn_mesh_by_name(polygon_name)
        skin_fn_mesh.getPoints(points)
        top_length = len(top_indices)
        self.wxs.clear()
        self.wxs.setLength(vtx_length*top_length)
        for vtx_id in range(vtx_length):
            wxs_id = vtx_id * top_length
            first_index = 0
            for top_id in range(top_length):
                vtx_point = points[vtx_id]
                chain_length = chain_lengths[top_id]
                self.wxs[wxs_id + top_id] = 0
                self.wxs[wxs_id+top_id] = chain_distance(vtx_point, joint_points, first_index, chain_length)
                first_index += chain_length
            if vtx_id == 0:
                top_id = 0
                first_index = 0
                vtx_point = points[vtx_id]
                chain_length = chain_lengths[top_id]
                chain_distance(vtx_point, joint_points, first_index, chain_length, True)

        self.set_indexes(top_indices)

    def finger_init(self, polygon_name, skin_cluster_name, indices, vxs, chain_lengths):
        self.polygon_name = polygon_name
        self.skin_cluster_name = skin_cluster_name
        self.chain_lengths = chain_lengths
        self.set_indexes(indices)

        points = MPointArray()
        skin_fn_mesh = get_fn_mesh_by_name(polygon_name)
        skin_fn_mesh.getPoints(points)

        weights = MDoubleArray()
        get_weights(polygon_name, skin_cluster_name, self.indices, weights)

        joint_length = len(indices)
        vtx_length = points.length()
        top_length = len(chain_lengths)
        vx_length = len(vxs)/4

        self.max_weights.setLength(top_length*vtx_length)
        self.wxs.setLength(vx_length*vtx_length)

        for vtx_id in range(vtx_length):
            wx_id = vtx_id * vx_length
            for vx_id in range(vx_length):
                slice_vx_id = vx_id * 4
                v = [vxs[slice_vx_id + 0], vxs[slice_vx_id + 1], vxs[slice_vx_id + 2]]
                x = vxs[slice_vx_id + 3]
                self.wxs[wx_id + vx_id] = ik_x(v, points[vtx_id]) - x
            slice_chain_id = 0
            for top_id in range(top_length):
                chain_length = chain_lengths[top_id]
                weight_id = vtx_id * joint_length
                max_weight_id = vtx_id * top_length
                weight = 0
                for chain_id in range(chain_length):
                    weight += weights[weight_id+slice_chain_id+chain_id]
                self.max_weights[max_weight_id+top_id] = weight
                slice_chain_id += chain_length

    def finger_solve(self, xs, ys, r):
        top_length = len(self.chain_lengths)
        vtx_length = self.max_weights.length() / top_length
        joint_length = self.indices.length()
        weights = MDoubleArray(vtx_length*joint_length)
        wx_length = self.wxs.length()/vtx_length

        for vtx_id in range(vtx_length):
            weight_id = vtx_id * joint_length
            wx_id = vtx_id * wx_length
            for top_id in range(top_length):
                chain_length = self.chain_lengths[top_id]
                split_length = chain_length - 1
                weights[weight_id] = 1.0
                for split_id in range(split_length):
                    w1 = weight_id + split_id
                    w2 = w1 + 1
                    weights[w2] = get_weight(self.wxs[wx_id + split_id] / r + 0.5, xs, ys)
                    if weights[w2] >= weights[w1]:
                        weights[w2] = weights[w1]
                    weights[w1] = weights[w1] - weights[w2]
                max_weight = self.max_weights[vtx_id*top_length+top_id]

                for chain_id in range(chain_length):
                    weights[weight_id+chain_id] = weights[weight_id+chain_id] * max_weight
                weight_id += chain_length
                wx_id += split_length
        set_weights(self.polygon_name, self.skin_cluster_name, self.indices, weights)


ws = WeightSolve()
solve = ws.solve
ik_init = ws.ik_init
soft_init = ws.soft_init
split_init = ws.split_init
points_init = ws.points_init
chains_init = ws.chains_init
finger_init = ws.finger_init


