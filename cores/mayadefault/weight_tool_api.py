# coding:utf-8
from maya.OpenMaya import *
from maya.OpenMayaAnim import *


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


def get_dag_path_component_by_name(name):
    selection_list = MSelectionList()
    selection_list.add(name)
    dag_path = MDagPath()
    components = MObject()
    selection_list.getDagPath(0, dag_path, components)
    return dag_path, components


def get_weights(polygon_name, skin_cluster_name, indices, weights):
    skin_polygon_path, skin_polygon_components = get_dag_path_component_by_name(polygon_name + ".vtx[*]")
    fn_skin = get_fn_skin_by_name(skin_cluster_name)
    fn_skin.getWeights(skin_polygon_path, skin_polygon_components, indices, weights)


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


def set_weights(polygon_name, skin_cluster_name, indices, weights):
    MGlobal.executeCommand("dgdirty %s;" % skin_cluster_name)
    skin_polygon_path, skin_polygon_components = get_dag_path_component_by_name(polygon_name + ".vtx[*]")
    fn_skin = get_fn_skin_by_name(skin_cluster_name)
    fn_skin.setWeights(skin_polygon_path, skin_polygon_components, indices, weights)


def py_to_m_array(cls, _list):
    result = cls()
    for elem in _list:
        result.append(elem)
    return result


def paint_eye(polygon_name, skin_cluster_name, indices, joint_points):
    indices = py_to_m_array(MIntArray, indices)
    joint_points = py_to_m_array(MPointArray, [MPoint(*p) for p in joint_points])
    max_weights = MDoubleArray()
    get_max_weights(polygon_name, skin_cluster_name, indices, max_weights)

    vtx_length = max_weights.length()

    find_vtx_ids = set()
    fn_mesh = get_fn_mesh_by_name(polygon_name)

    joint_length = indices.length()
    vtx_id_data = [[] for _ in range(joint_length)]
    util = MScriptUtil()
    ptr = util.asIntPtr()
    for joint_id in range(joint_length):
        closest_point = MPoint()
        fn_mesh.getClosestPoint(joint_points[joint_id], closest_point, MSpace.kTransform, ptr)
        face_id = util.getInt(ptr)
        face_vtx_ids = MIntArray()
        fn_mesh.getPolygonVertices(face_id, face_vtx_ids)
        face_vtx_length = face_vtx_ids.length()
        closest_vtx_id = face_vtx_ids[0]
        face_vtx_point = MPoint()
        fn_mesh.getPoint(closest_vtx_id, face_vtx_point)
        min_distance = face_vtx_point.distanceTo(joint_points[joint_id])
        for vtx_arr_id in range(1, face_vtx_length, 1):
            face_vtx_id = face_vtx_ids[vtx_arr_id]
            fn_mesh.getPoint(face_vtx_id, face_vtx_point)
            next_distance = face_vtx_point.distanceTo(joint_points[joint_id])
            if next_distance < min_distance:
                min_distance = next_distance
                closest_vtx_id = face_vtx_id
        next_ids = []
        vtx_id_data[joint_id].append(next_ids)
        if closest_vtx_id in find_vtx_ids:
            continue
        next_ids.append(closest_vtx_id)
        find_vtx_ids.add(closest_vtx_id)
    mit_vtx = MItMeshVertex(get_dag_path_by_name(polygon_name))

    for _ in range(vtx_length):
        for joint_id in range(joint_length):
            next_ids = []
            vtx_id_data[joint_id].append(next_ids)
            pre_ids = vtx_id_data[joint_id][-2]
            if len(pre_ids) == 0:
                continue
            for pre_id in pre_ids:
                connect_vertices = MIntArray()
                mit_vtx.setIndex(pre_id, ptr)
                mit_vtx.getConnectedVertices(connect_vertices)
                connect_vtx_length = connect_vertices.length()
                for connect_vertices_id in range(connect_vtx_length):
                    connect_vtx_id = connect_vertices[connect_vertices_id]
                    if connect_vtx_id in find_vtx_ids:
                        continue
                    next_ids.append(connect_vtx_id)
                    find_vtx_ids.add(connect_vtx_id)
        if len(find_vtx_ids) == vtx_length:
            break

    weights = MDoubleArray(joint_length*vtx_length, 0)
    for joint_id in range(joint_length):
        for vtx_ids in vtx_id_data[joint_id]:
            for vtx_id in vtx_ids:
                weights[vtx_id*joint_length+joint_id] = max_weights[vtx_id]

    set_weights(polygon_name, skin_cluster_name, indices, weights)


def limit_max_influence(polygon_name, skin_cluster_name, indices, max_influence):
    indices = py_to_m_array(MIntArray, indices)
    fn_mesh = get_fn_mesh_by_name(polygon_name)
    vtx_length = fn_mesh.numVertices()
    joint_length = len(indices)
    weights = MDoubleArray()
    get_weights(polygon_name, skin_cluster_name, indices, weights)
    for vtx_id in range(vtx_length):
        weight_slice_id = vtx_id * joint_length
        iws = []
        for joint_id in range(joint_length):
            weight_id = weight_slice_id+joint_id
            iws.append(tuple([joint_id, weights[weight_id]]))
            weights[weight_id] = 0
        iws.sort(key=lambda x: x[1], reverse=True)
        max_weight = 0
        for iw_id in range(max_influence):
            max_weight += iws[iw_id][1]
        for iw_id in range(max_influence):
            weights[weight_slice_id+iws[iw_id][0]] = iws[iw_id][1]/max_weight
    set_weights(polygon_name, skin_cluster_name, indices, weights)


def get_ids_weights(polygon_name, skin_cluster_name, indices, vtx_ids, weights):
    fn_component = MFnSingleIndexedComponent()
    skin_polygon_components = fn_component.create(MFn.kMeshVertComponent)
    fn_component.addElements(vtx_ids)
    skin_polygon_path = get_dag_path_by_name(polygon_name)
    fn_skin = get_fn_skin_by_name(skin_cluster_name)
    fn_skin.getWeights(skin_polygon_path, skin_polygon_components, indices, weights)


def set_ids_weights(polygon_name, skin_cluster_name, indices, vtx_ids, weights):
    MGlobal.executeCommand("dgdirty %s;" % skin_cluster_name)
    fn_component = MFnSingleIndexedComponent()
    skin_polygon_components = fn_component.create(MFn.kMeshVertComponent)
    fn_component.addElements(vtx_ids)
    skin_polygon_path = get_dag_path_by_name(polygon_name)
    fn_skin = get_fn_skin_by_name(skin_cluster_name)
    fn_skin.setWeights(skin_polygon_path, skin_polygon_components, indices, weights)


def lock_influence_smooth(polygon_name, skin_cluster_name, indices, vtx_ids, smooth_step, smooth_count):
    indices = py_to_m_array(MIntArray, indices)
    vtx_ids = py_to_m_array(MIntArray, vtx_ids)
    joint_length = indices.length()
    vtx_length = vtx_ids.length()
    mit_vtx = MItMeshVertex(get_dag_path_by_name(polygon_name))
    util = MScriptUtil()
    ptr = util.asIntPtr()
    grow_ids = set()
    id_connect_ids = dict()
    for vtx_index in range(vtx_length):
        vtx_id = vtx_ids[vtx_index]
        grow_ids.add(vtx_id)
        connect_vertices = MIntArray()
        mit_vtx.setIndex(vtx_id, ptr)
        mit_vtx.getConnectedVertices(connect_vertices)
        connect_vtx_length = connect_vertices.length()
        for connect_vtx_id in range(connect_vtx_length):
            grow_ids.add(connect_vertices[connect_vtx_id])
        id_connect_ids[vtx_id] = connect_vertices
    id_map = dict()
    grow_vtx_ids = MIntArray()
    for i, grow_id in enumerate(sorted(grow_ids)):
        grow_vtx_ids.append(grow_id)
        id_map[grow_id] = i
    weights = MDoubleArray()
    get_ids_weights(polygon_name, skin_cluster_name, indices, grow_vtx_ids, weights)
    for _ in range(smooth_count):
        smooth_weights = MDoubleArray()
        smooth_weights.copy(weights)
        for vtx_index in range(vtx_length):
            vtx_id = vtx_ids[vtx_index]
            weight_id = id_map[vtx_id] * joint_length
            connect_ids = id_connect_ids[vtx_id]
            connect_vtx_length = connect_ids.length()
            for joint_id in range(joint_length):
                smooth_weights[weight_id + joint_id] = 0
                for connect_index in range(connect_vtx_length):
                    connect_id = connect_ids[connect_index]
                    smooth_weights[weight_id+joint_id] += weights[id_map[connect_id]*joint_length+joint_id]
                smooth_weights[weight_id+joint_id] /= connect_vtx_length
            sum_weight = 0
            for joint_id in range(joint_length):
                i = weight_id + joint_id
                smooth_weight = smooth_weights[i]
                smooth_weights[i] = weights[i]
                if smooth_weights[i] > 0.0001:
                    smooth_weights[i] = smooth_weights[i] + (smooth_weight - smooth_weights[i]) * smooth_step
                sum_weight += smooth_weights[weight_id+joint_id]
            for joint_id in range(joint_length):
                smooth_weights[weight_id+joint_id] /= sum_weight
        weights.copy(smooth_weights)
    set_ids_weights(polygon_name, skin_cluster_name, indices, grow_vtx_ids, weights)


cache_name_weights = dict()


def copy_joint_weight(polygon_name, skin_cluster_name, indices, joint_names):
    indices = py_to_m_array(MIntArray, indices)
    weights = MDoubleArray()
    get_weights(polygon_name, skin_cluster_name, indices, weights)
    joint_length = indices.length()
    vtx_length = weights.length()/joint_length
    cache_name_weights.clear()
    for joint_name in joint_names:
        cache_name_weights[joint_name] = [0] * vtx_length
    for vtx_id in range(vtx_length):
        weight_id = vtx_id*joint_length
        for joint_id in range(joint_length):
            cache_name_weights[joint_names[joint_id]][vtx_id] = weights[weight_id+joint_id]


def paste_joint_weight(polygon_name, skin_cluster_name, indices, joint_names):
    indices = py_to_m_array(MIntArray, indices)
    weights = MDoubleArray()
    get_weights(polygon_name, skin_cluster_name, indices, weights)
    joint_length = indices.length()
    vtx_length = weights.length()/joint_length
    edit_ids = []
    keep_ids = []
    for joint_id in range(joint_length):
        if joint_names[joint_id] in cache_name_weights:
            edit_ids.append(joint_id)
        else:
            keep_ids.append(joint_id)
    if len(edit_ids) == 0:
        return
    for vtx_id in range(vtx_length):
        weight_id = vtx_id * joint_length
        keep_weight_sum = 0.0
        edit_weight_sum = 0.0
        cache_weight_sum = 0.0
        for keep_id in keep_ids:
            keep_weight_sum += weights[weight_id+keep_id]
        for edit_id in edit_ids:
            edit_weight_sum += weights[weight_id+edit_id]
            cache_weight_sum += cache_name_weights[joint_names[edit_id]][vtx_id]
            weights[weight_id + edit_id] = cache_name_weights[joint_names[edit_id]][vtx_id]
        weight_sum = keep_weight_sum + edit_weight_sum
        new_edit_weight_sum = min([weight_sum, cache_weight_sum])
        new_keep_weight_sum = min(keep_weight_sum, weight_sum-new_edit_weight_sum)
        edit_scale = 0.0 if cache_weight_sum < 0.00001 else new_edit_weight_sum / cache_weight_sum
        keep_scale = 0.0 if keep_weight_sum < 0.00001 else new_keep_weight_sum / keep_weight_sum
        if len(keep_ids) == 0:
            edit_scale = 0.0 if cache_weight_sum < 0.00001 else weight_sum / cache_weight_sum
        for keep_id in keep_ids:
            weights[weight_id + keep_id] *= keep_scale
        for edit_id in edit_ids:
            weights[weight_id + edit_id] *= edit_scale
        new_sum = 0.0
        for joint_id in range(joint_length):
            new_sum += weights[weight_id + joint_id]
    set_weights(polygon_name, skin_cluster_name, indices, weights)


def str_to_dag_path(name, dag_path):
    selection_list = MSelectionList()
    selection_list.add(name)
    selection_list.getDagPath(0, dag_path)


def str_to_depend_node(name, depend_node):
    selection_list = MSelectionList()
    selection_list.add(name)
    selection_list.getDependNode(0, depend_node)


def get_mesh_points(polygon_name,  points):
    dag_path = MDagPath()
    str_to_dag_path(polygon_name, dag_path)
    fn_mesh = MFnMesh(dag_path)
    fn_mesh.getPoints(points, MSpace.kWorld)


def get_mesh_normals(polygon_name, normals):
    dag_path = MDagPath()
    str_to_dag_path(polygon_name, dag_path)
    fn_mesh = MFnMesh(dag_path)
    fn_mesh.getVertexNormals(True, normals)


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


def _normal_warp_skin(src_polygon_name, dst_polygon_name, src_skin_cluster_name, dst_skin_cluster_name, indices):
    indices = py_to_m_array(MIntArray, indices)
    joint_length = len(indices)

    src_dag_path = MDagPath()
    str_to_dag_path(src_polygon_name, src_dag_path)
    src_fn_mesh = MFnMesh(src_dag_path)
    src_face_length = src_fn_mesh.numPolygons()
    src_weights = MDoubleArray()
    get_weights(src_polygon_name, src_skin_cluster_name, indices, src_weights)

    dst_dag_path = MDagPath()
    str_to_dag_path(dst_polygon_name, dst_dag_path)
    dst_fn_mesh = MFnMesh(dst_dag_path)
    dst_vtx_length = dst_fn_mesh.numVertices()
    dst_weights = MDoubleArray()
    get_weights(dst_polygon_name, dst_skin_cluster_name, indices, dst_weights)

    for dst_vtx_id in range(dst_vtx_length):
        # if dst_vtx_id != 10:
        #     continue
        distance = 999999
        fv_ids = MIntArray()  # fv face vertices
        fv_ws = []
        for src_face_id in range(src_face_length):
            current_ids = MIntArray()
            src_fn_mesh.getPolygonVertices(src_face_id, current_ids)
            src_normal = MVector()
            src_fn_mesh.getPolygonNormal(src_face_id, src_normal)

            dst_point = MPoint()
            dst_fn_mesh.getPoint(dst_vtx_id, dst_point)
            dst_normal = MVector()
            dst_fn_mesh.getVertexNormal(dst_vtx_id, dst_normal)
            angle = src_normal.angle(dst_normal)
            if angle > 0.68:
                continue
            face_vtx_length = current_ids.length()
            data_x = [[0]*face_vtx_length, [0]*face_vtx_length, [0]*face_vtx_length]
            for face_vtx_id in range(face_vtx_length):
                src_point = MPoint()
                src_fn_mesh.getPoint(current_ids[face_vtx_id], src_point)
                data_x[0][face_vtx_id] = src_point.x
                data_x[1][face_vtx_id] = src_point.y
                data_x[2][face_vtx_id] = src_point.z

            dst_fn_mesh.getVertexNormal(dst_vtx_id, dst_normal)
            data_y = [dst_point.x, dst_point.y, dst_point.z]
            current_ws = liner_regression(data_x, data_y)

            src_point = MPoint()
            for face_vtx_id in range(face_vtx_length):
                src_point.x += current_ws[face_vtx_id] * data_x[0][face_vtx_id]
                src_point.y += current_ws[face_vtx_id] * data_x[1][face_vtx_id]
                src_point.z += current_ws[face_vtx_id] * data_x[2][face_vtx_id]

            current_distance = src_point.distanceTo(dst_point)
            if current_distance < distance:
                distance = current_distance
                fv_ws = current_ws
                fv_ids = current_ids
        fv_length = fv_ids.length()
        dst_slice_id = dst_vtx_id * joint_length
        for joint_id in range(joint_length):
            w = 0.0
            for fv_id in range(fv_length):
                src_weight_id = fv_ids[fv_id] * joint_length + joint_id
                w += fv_ws[fv_id]*src_weights[src_weight_id]
            dst_weights.set(w, dst_slice_id+joint_id)
    set_weights(dst_polygon_name, dst_skin_cluster_name, indices, dst_weights)


def normal_warp_skin(src_polygon_name, dst_polygon_name, skin_cluster_name):

    src_dag_path = MDagPath()
    str_to_dag_path(src_polygon_name, src_dag_path)
    src_fn_mesh = MFnMesh(src_dag_path)
    src_face_length = src_fn_mesh.numPolygons()
    joint_length = src_fn_mesh.numVertices()
    indices = py_to_m_array(MIntArray, range(joint_length))

    dst_dag_path = MDagPath()
    str_to_dag_path(dst_polygon_name, dst_dag_path)
    dst_fn_mesh = MFnMesh(dst_dag_path)
    dst_vtx_length = dst_fn_mesh.numVertices()
    dst_weights = MDoubleArray()
    get_weights(dst_polygon_name, skin_cluster_name, indices, dst_weights)

    for dst_vtx_id in range(dst_vtx_length):
        distance = 999999
        fv_ids = MIntArray()  # fv face vertices
        fv_ws = []
        for src_face_id in range(src_face_length):
            current_ids = MIntArray()
            src_fn_mesh.getPolygonVertices(src_face_id, current_ids)
            src_normal = MVector()
            src_fn_mesh.getPolygonNormal(src_face_id, src_normal)

            dst_point = MPoint()
            dst_fn_mesh.getPoint(dst_vtx_id, dst_point)
            dst_normal = MVector()
            dst_fn_mesh.getVertexNormal(dst_vtx_id, dst_normal)
            angle = src_normal.angle(dst_normal)
            if angle > 0.68:
                continue
            face_vtx_length = current_ids.length()
            data_x = [[0]*face_vtx_length, [0]*face_vtx_length, [0]*face_vtx_length]
            for face_vtx_id in range(face_vtx_length):
                src_point = MPoint()
                src_fn_mesh.getPoint(current_ids[face_vtx_id], src_point)
                data_x[0][face_vtx_id] = src_point.x
                data_x[1][face_vtx_id] = src_point.y
                data_x[2][face_vtx_id] = src_point.z

            dst_fn_mesh.getVertexNormal(dst_vtx_id, dst_normal)
            data_y = [dst_point.x, dst_point.y, dst_point.z]
            current_ws = liner_regression(data_x, data_y)

            src_point = MPoint()
            for face_vtx_id in range(face_vtx_length):
                src_point.x += current_ws[face_vtx_id] * data_x[0][face_vtx_id]
                src_point.y += current_ws[face_vtx_id] * data_x[1][face_vtx_id]
                src_point.z += current_ws[face_vtx_id] * data_x[2][face_vtx_id]

            current_distance = src_point.distanceTo(dst_point)
            if current_distance < distance:
                distance = current_distance
                fv_ws = current_ws
                fv_ids = current_ids
        fv_length = fv_ids.length()
        if fv_length == 0:
            continue
        dst_slice_id = dst_vtx_id * joint_length
        for joint_id in range(joint_length):
            dst_weights.set(0, dst_slice_id + joint_id)
        for fv_id in range(fv_length):
            dst_weights.set(fv_ws[fv_id], dst_slice_id + fv_ids[fv_id])
    set_weights(dst_polygon_name, skin_cluster_name, indices, dst_weights)

