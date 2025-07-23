
#include "FunCToPy.h"

#include "maya/MSelectionList.h"
#include "maya/MDagPath.h"
#include "maya/MString.h"
#include "maya/MDoubleArray.h"
#include "maya/MIntArray.h"
#include "maya/MFnMesh.h"
#include "maya/MFnSkinCluster.h"
#include "maya/MFnSingleIndexedComponent.h"
#include "maya/MPointArray.h"
#include "maya/MGlobal.h"
#include <tbb/tbb.h>



double dot(double* list1, double* list2, int length) {
    double result = 0;
    for (int i = 0; i < length; i++) {
        result += list1[i] * list2[i];
    }
    return result;
}

double* matrix_dot_list(double** matrix, double* list2, int matrix_length, int list_length) {
    double* result = new double[matrix_length];
    for (int i = 0; i < matrix_length; i++) {
        result[i] = dot(matrix[i], list2, list_length);
    }
    return result;
}

double* sub_list(double* list1, double* list2, int length) {
    double* result = new double[length];
    for (int i = 0; i < length; i++) {
        result[i] = list1[i] - list2[i];
    }
    return result;
}


double* liner_regression(double** data_x, double* data_y, int x_length, int y_length) {
    double* slopes = new double[x_length];
    for (int i = 0; i < x_length; i++) {
        slopes[i] = 1.0 / x_length;
    }
    for (int i = 0; i < 10; i++) {
        for (int slope_index = 0; slope_index < x_length; slope_index++) {
            double other_slope_sum = 1.0 - slopes[slope_index];
            double* edit_scale_list = new double[x_length];;
            if (other_slope_sum < 0.00001) {
                for (int j = 0; j < x_length; j++) {
                    edit_scale_list[j] = -1.0 / (x_length - 1.0);
                }
            }
            else {
                for (int j = 0; j < x_length; j++) {
                    edit_scale_list[j] = -1.0 * slopes[j] / other_slope_sum;
                }
            }
            edit_scale_list[slope_index] = 1.0;
            double* xs = matrix_dot_list(data_x, edit_scale_list, y_length, x_length);
            double* ys = sub_list(data_y, matrix_dot_list(data_x, slopes, y_length, x_length), y_length);
            double dot_xs = dot(xs, xs, y_length);
            if (dot_xs < 0.00001) {
                continue;
            }
            double edit_weight = dot(xs, ys, y_length) / dot_xs;
            for (int j = 0; j < x_length; j++) {
                slopes[j] = slopes[j] + edit_weight * edit_scale_list[j];
                if (slopes[j] > 1) {
                    slopes[j] = 1;
                }
                if (slopes[j] < 0) {
                    slopes[j] = 0;
                }
            }
            double sum_slopes = 0;
            for (int j = 0; j < x_length; j++) {
                sum_slopes += slopes[j];
            }
            for (int j = 0; j < x_length; j++) {
                slopes[j] = slopes[j] / sum_slopes;
            }
            delete[] xs;
            delete[] ys;
            delete[] edit_scale_list;
        }
    }
    return slopes;
}


class PartSdr {
public:
    MString skin_polygon_name;
    MString skin_cluster_name;
    MString sculpt_polygon_name;
    MIntArray part_joint_ids;
    MIntArray part_vtx_ids;
    MDoubleArray max_weights;
    double*** point_data;
    void init(MString _skin_polygon_name, MString _skin_cluster_name, MString _sculpt_polygon_name, MIntArray& unlock_joint_ids);
    void solve();
};


void PartSdr::init(MString _skin_polygon_name, MString _skin_cluster_name, MString _sculpt_polygon_name, MIntArray& unlock_joint_ids) {
    // init class var
    int old_vtx_length = part_vtx_ids.length();
    int joint_length = unlock_joint_ids.length();
    skin_polygon_name = _skin_polygon_name;
    skin_cluster_name = _skin_cluster_name;
    sculpt_polygon_name = _sculpt_polygon_name;
    MDagPath skin_polygon_path;
    MObject skin_depend_node;
    MDagPath sculpt_fn_mesh_path;
    MSelectionList selection_list;
    selection_list.add(skin_polygon_name);
    selection_list.add(skin_cluster_name);
    selection_list.add(sculpt_polygon_name);
    selection_list.getDagPath(0, skin_polygon_path);
    selection_list.getDependNode(1, skin_depend_node);
    selection_list.getDagPath(2, sculpt_fn_mesh_path);
    for (int i = 0; i < old_vtx_length; i++) {
        delete[] point_data[i][0];
        delete[] point_data[i][1];
        delete[] point_data[i][2];
        delete[] point_data[i];
    }
    delete[] point_data;
    part_joint_ids.clear();
    for (int i = 0; i < joint_length; i++) {
        part_joint_ids.append(unlock_joint_ids[i]);
    }
    // get all weights
    MFnSkinCluster fn_skin(skin_depend_node);
    MFnSingleIndexedComponent fn_component;
    MObject skin_polygon_components = fn_component.create(MFn::kMeshVertComponent);
    MIntArray all_vtx_ids;
    MFnMesh skin_fn_mesh(skin_polygon_path);
    int all_vtx_length = skin_fn_mesh.numVertices();
    for (int i = 0; i < all_vtx_length; i++) {
        all_vtx_ids.append(i);
    }
    MDoubleArray unlock_weights;
    fn_skin.getWeights(skin_polygon_path, skin_polygon_components, part_joint_ids, unlock_weights);
    // get max weights
    MDoubleArray all_max_weights;
    max_weights.clear();
    part_vtx_ids.clear();
    for (int i = 0; i < all_vtx_length; i++) {
        double max_weight = 0.0;
        for (int j = 0; j < joint_length; j++) {
            max_weight += unlock_weights[i * joint_length + j];
        }
        if (max_weight > 0.005) {
            max_weights.append(max_weight);
            part_vtx_ids.append(i);
        }
    }
    // get old weights
    MObject part_component = fn_component.create(MFn::kMeshVertComponent);
    fn_component.addElements(part_vtx_ids);
    MDoubleArray old_weights;
    fn_skin.getWeights(skin_polygon_path, part_component, part_joint_ids, old_weights);

    // get point data
    int part_vtx_length = part_vtx_ids.length();
    point_data = new double** [part_vtx_length];
    for (int i = 0; i < part_vtx_length; i++) {
        point_data[i] = new double* [3];
        point_data[i][0] = new double[joint_length];
        point_data[i][1] = new double[joint_length];
        point_data[i][2] = new double[joint_length];
    }
    for (int rigid_joint_id = 0; rigid_joint_id < joint_length; rigid_joint_id++) {
        MDoubleArray rigid_weights;
        for (int vtx_id = 0; vtx_id < part_vtx_length; vtx_id++) {
            for (int joint_id = 0; joint_id < joint_length; joint_id++) {
                if (joint_id == rigid_joint_id) {
                    rigid_weights.append(max_weights[vtx_id]);
                }
                else {
                    rigid_weights.append(0);
                }
            }
        }
        fn_skin.setWeights(skin_polygon_path, part_component, part_joint_ids, rigid_weights);
        MPointArray part_points;
        skin_fn_mesh.getPoints(part_points);
        for (int i = 0; i < part_vtx_length; i++) {
            point_data[i][0][rigid_joint_id] = part_points[part_vtx_ids[i]].x;
            point_data[i][1][rigid_joint_id] = part_points[part_vtx_ids[i]].y;
            point_data[i][2][rigid_joint_id] = part_points[part_vtx_ids[i]].z;
        }
    }
    fn_skin.setWeights(skin_polygon_path, part_component, part_joint_ids, old_weights);
};


void PartSdr::solve() {
    // init var
    int part_vtx_length = part_vtx_ids.length();
    int part_joint_length = part_joint_ids.length();
    MDagPath skin_polygon_path;
    MObject skin_depend_node;
    MDagPath sculpt_fn_mesh_path;
    MSelectionList selection_list;
    selection_list.add(skin_polygon_name);
    selection_list.add(skin_cluster_name);
    selection_list.add(sculpt_polygon_name);
    selection_list.getDagPath(0, skin_polygon_path);
    selection_list.getDependNode(1, skin_depend_node);
    selection_list.getDagPath(2, sculpt_fn_mesh_path);
    MPointArray part_points;
    MFnMesh sculpt_fn_mesh(sculpt_fn_mesh_path);
    sculpt_fn_mesh.getPoints(part_points);
    // edit vtx ids
    MPointArray skin_points;
    MFnMesh skin_fn_mesh(skin_polygon_path);
    skin_fn_mesh.getPoints(skin_points);
    MIntArray edit_vtx_ids;
    for (int i = 0; i < part_vtx_length; i++) {
        int index = part_vtx_ids[i];
        double offset = (skin_points[index] - part_points[index]).length();
        if (offset > 0.00001) {
            edit_vtx_ids.append(i);
        }
    }
    int edit_vtx_length = edit_vtx_ids.length();
    if (edit_vtx_length < 1) {
        return;
    }
    // compute weights
    MDoubleArray weights;
    weights.setLength(edit_vtx_length * part_joint_length);
    double*** _point_data = point_data;
    MDoubleArray _max_weights = max_weights;
    MIntArray _part_vtx_ids = part_vtx_ids;
    tbb::parallel_for(0, edit_vtx_length, 1, [&part_points, part_joint_length, &_point_data, &weights, &_max_weights, &_part_vtx_ids, &edit_vtx_ids](int i) {
        int index = edit_vtx_ids[i];
        MPoint p = part_points[_part_vtx_ids[index]];
        double* data_y = new double[3]{ p.x, p.y, p.z };
        double* slopes = liner_regression(_point_data[index], data_y, part_joint_length, 3);
        for (int joint_id = 0; joint_id < part_joint_length; joint_id++) {
            weights[i * part_joint_length + joint_id] = _max_weights[index] * slopes[joint_id];
        }
        delete[] slopes;
    }
    );

    // set weights
    MFnSingleIndexedComponent fn_component;
    MObject part_component = fn_component.create(MFn::kMeshVertComponent);
    MIntArray edit_weight_ids;
    for (int i = 0; i < edit_vtx_length; i++) {
        edit_weight_ids.append(part_vtx_ids[edit_vtx_ids[i]]);
    }
    fn_component.addElements(edit_weight_ids);
    MFnSkinCluster fn_skin(skin_depend_node);
    fn_skin.setWeights(skin_polygon_path, part_component, part_joint_ids, weights);
    // copy points
    skin_fn_mesh.getPoints(skin_points);
    sculpt_fn_mesh.setPoints(skin_points);
}

PartSdr part_sdr;

void part_sdr_init(MString _skin_polygon_name, MString _skin_cluster_name, MString _sculpt_polygon_name, MIntArray& unlock_joint_ids) {
    part_sdr.init(_skin_polygon_name, _skin_cluster_name, _sculpt_polygon_name, unlock_joint_ids);
}

void part_sdr_solve() {
    part_sdr.solve();
}

FunCToPy0(part_sdr_solve)
FunCToPy4(part_sdr_init, MString, MString, MString, MIntArray)
PyDefList(
    PyDef(part_sdr_solve),
    PyDef(part_sdr_init)
)
PyMod(sculpt_api)