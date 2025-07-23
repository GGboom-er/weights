# coding:utf-8

from maya.OpenMaya import *
from maya.OpenMayaAnim import *
from maya import cmds


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


def get_max_weights(weights, joint_length, vtx_length):
    u"""
    :param weights: 权重
    :param joint_length: 骨骼长度
    :param vtx_length: 点长度
    :return: 每点最大权重值
    """
    return [sum([weights[joint_length*j+i] for i in range(joint_length)]) for j in range(vtx_length)]


def dot(list1, list2):
    return sum([elem1*elem2 for elem1, elem2 in zip(list1, list2)])


def matrix_dot_list(matrix, list2):
    return [dot(list1, list2) for list1 in matrix]


def sub_list(list1, list2):
    return [elem1-elem2 for elem1, elem2 in zip(list1, list2)]


def liner_regression(data_x, data_y):
    u"""
    函数：y = b1*x1+b2*x2+b3*x3 ..... + bn * xn
    :param data_x:  函数中一组x的值/maya中每个蒙皮模型的坐标
    :param data_y: 函数中一组y的值/maya中变形模型的坐标。
    :return: slopes斜率/maya中的权重值。
    """
    slopes_length = len(data_x[0])
    slopes = [1.0/slopes_length] * slopes_length
    for i in range(10):
        for slope_index in range(slopes_length):
            other_slope_sum = 1.0 - slopes[slope_index]
            if other_slope_sum < 0.0001:
                edit_scale_list = [-1.0/(slopes_length-1)] * slopes_length
            else:
                edit_scale_list = [-slope/other_slope_sum for slope in slopes]
            # edit_scale_list 所有骨骼权重修改比例的列表
            # edit_weight 骨骼权重修改量
            edit_scale_list[slope_index] = 1.0
            xs = matrix_dot_list(data_x, edit_scale_list)
            ys = sub_list(data_y, matrix_dot_list(data_x, slopes))
            dot_xs = dot(xs, xs)
            if dot_xs < 0.00001:
                continue
            edit_weight = dot(xs, ys) / dot(xs, xs)
            slopes = [slope+edit_weight*offset_weight for slope, offset_weight in zip(slopes, edit_scale_list)]
            slopes = [max(min(slope, 1.0), 0) for slope in slopes]
            sum_slopes = sum(slopes)
            slopes = [slope/sum_slopes for slope in slopes]
    return slopes


class PartSdr(object):

    def __init__(self):
        self.skin_polygon_name = ""
        self.skin_cluster_name = ""
        self.sculpt_polygon_name = ""
        self.part_joint_ids = MIntArray()
        self.part_vtx_ids = MIntArray()
        self.max_weights = MFloatArray()
        self.point_data = []

    def init(self, skin_polygon_name, skin_cluster_name, sculpt_polygon_name, unlock_joint_ids):
        self.skin_polygon_name = skin_polygon_name
        self.skin_cluster_name = skin_cluster_name
        self.sculpt_polygon_name = sculpt_polygon_name
        self.part_joint_ids.clear()
        for i in unlock_joint_ids:
            self.part_joint_ids.append(i)
        skin_fn_mesh = get_fn_mesh_by_name(self.skin_polygon_name)
        fn_skin = get_fn_skin_by_name(self.skin_cluster_name)
        unlock_weights = MDoubleArray()
        skin_polygon_path, skin_polygon_components = get_dag_path_component_by_name(self.skin_polygon_name+".vtx[*]")
        fn_skin.getWeights(skin_polygon_path, skin_polygon_components, self.part_joint_ids, unlock_weights)
        joint_length = self.part_joint_ids.length()
        max_weights = get_max_weights(unlock_weights, joint_length, skin_fn_mesh.numVertices())
        self.max_weights.clear()
        self.part_vtx_ids.clear()
        for i, w in enumerate(max_weights):
            if w > 0.005:
                self.part_vtx_ids.append(i)
                self.max_weights.append(w)
        part_component = get_component_by_ids(self.part_vtx_ids)
        old_weights = MDoubleArray()
        fn_skin.getWeights(skin_polygon_path, part_component, self.part_joint_ids, old_weights)
        part_vtx_length = self.part_vtx_ids.length()
        self.point_data = []
        for i in range(part_vtx_length):
            self.point_data.append([[], [], []])
        for rigid_joint_id in range(joint_length):
            rigid_weights = MDoubleArray()
            for vtx_id in range(part_vtx_length):
                for joint_id in range(joint_length):
                    if joint_id == rigid_joint_id:
                        rigid_weights.append(self.max_weights[vtx_id])
                    else:
                        rigid_weights.append(0)
            fn_skin.setWeights(skin_polygon_path, part_component, self.part_joint_ids, rigid_weights)
            part_points = MPointArray()
            skin_fn_mesh.getPoints(part_points)
            for i in range(part_vtx_length):
                self.point_data[i][0].append(part_points[self.part_vtx_ids[i]].x)
                self.point_data[i][1].append(part_points[self.part_vtx_ids[i]].y)
                self.point_data[i][2].append(part_points[self.part_vtx_ids[i]].z)
        fn_skin.setWeights(skin_polygon_path, part_component, self.part_joint_ids, old_weights)

    def solve(self):
        if not cmds.objExists(self.sculpt_polygon_name):
            return
        part_vtx_length = self.part_vtx_ids.length()
        part_points = MPointArray()
        sculpt_fn_mesh = get_fn_mesh_by_name(self.sculpt_polygon_name)
        sculpt_fn_mesh.getPoints(part_points)
        weights = MDoubleArray()
        for i in range(part_vtx_length):
            p = part_points[self.part_vtx_ids[i]]
            slopes = liner_regression(self.point_data[i], [p.x, p.y, p.z])
            for w in slopes:
                weights.append(self.max_weights[i]*w)
        skin_polygon_path = get_dag_path_by_name(self.skin_polygon_name)
        part_component = get_component_by_ids(self.part_vtx_ids)
        fn_skin = get_fn_skin_by_name(self.skin_cluster_name)
        fn_skin.setWeights(skin_polygon_path, part_component, self.part_joint_ids, weights)
        skin_fn_mesh = get_fn_mesh_by_name(self.skin_polygon_name)
        skin_points = MPointArray()
        skin_fn_mesh.getPoints(skin_points)
        sculpt_fn_mesh.setPoints(skin_points)


part_sdr = PartSdr()


def part_sdr_init(*args, **kwargs):
    part_sdr.init(*args, **kwargs)


def part_sdr_solve():
    part_sdr.solve()

