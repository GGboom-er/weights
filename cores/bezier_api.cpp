#include "FunCToPy.h"

#include "maya/MSelectionList.h"
#include "maya/MDagPath.h"
#include "maya/MDoubleArray.h"
#include "maya/MIntArray.h"
#include "maya/MFnMesh.h"
#include "maya/MFnSkinCluster.h"
#include "maya/MFnSingleIndexedComponent.h"
#include "maya/MPointArray.h"
#include "maya/MGlobal.h"
#include "maya/MPlug.h"
#include "maya/MFnDoubleArrayData.h"
#include "maya/MVector.h"
#include "maya/MVectorArray.h"
#include "maya/MPoint.h"
#include <tbb/tbb.h>
#include <vector>

using namespace std;
#define ik_x(v, p) (v[0] * p[0] + v[1] * p[1] + v[2] * p[2]) / (v[0] * v[0] + v[1] * v[1] + v[2] * v[2])


//  base math compute
inline double bezier_v(double p[4], double t) {
    return p[0] * pow(1.0 - t, 3.0) + 3 * p[1] * t * pow(1 - t, 2) + 3 * p[2] * pow(t, 2) * (1 - t) + p[3] * pow(t, 3);
}

double bezier_t(double p[4], double v) {
    double min_t = 0.0;
    double max_t = 1.0;
    double t;
    double error_range;
    while (true) {
        t = (min_t + max_t) / 2;
        error_range = bezier_v(p, t) - v;
        if (error_range > 0.0001) {
            max_t = t;
        }
        else
        {
            if (error_range < -0.0001) {
                min_t = t;
            }
            else
            {
                return t;
            };
        };

    };
};

double get_weight(double x, double xs[4], double ys[4]) {
    if (x <= 0) {
        return ys[0];
    };
    if (x >= 1) {
        return ys[3];
    };
    double t = bezier_t(xs, x);
    return bezier_v(ys, t);
}


// base maya api fun

void set_paint_weights(MString skin_cluster_name,  MDoubleArray& paint_weights) {
    MSelectionList selection_list;
    selection_list.add(skin_cluster_name);
    MObject depend_node;
    selection_list.getDependNode(0, depend_node);
    MFnDependencyNode fn_depend_node(depend_node);
    MPlug ptw_plug = fn_depend_node.findPlug("ptw", false);
    MFnDoubleArrayData double_array_data;
    MObject double_array_object = double_array_data.create(paint_weights);
    ptw_plug.setMObject(double_array_object);
    MObject temp_obj = ptw_plug.asMObject();
    MFnDoubleArrayData temp_double_array_data(temp_obj);
    MDoubleArray temp_array = temp_double_array_data.array();
}

void get_mesh_points(MString polygon_name, MPointArray& points) {
    MSelectionList selection_list;
    selection_list.add(polygon_name);
    MDagPath dag_path;
    selection_list.getDagPath(0, dag_path);
    MFnMesh fn_mesh(dag_path);
    fn_mesh.getPoints(points);
}

void get_weights(MString polygon_name, MString skin_cluster_name, MIntArray& indices, MDoubleArray& weights) {
    MSelectionList selection_list;
    selection_list.add(polygon_name + ".vtx[*]");
    selection_list.add(skin_cluster_name);
    MDagPath path;
    MObject components;
    MObject skin_depend_node;
    selection_list.getDagPath(0, path, components);
    selection_list.getDependNode(1, skin_depend_node);
    MFnSkinCluster fn_skin(skin_depend_node);
    fn_skin.getWeights(path, components, indices, weights);
}

void set_weights(MString polygon_name, MString skin_cluster_name, MIntArray& indices, MDoubleArray& weights) {
    MSelectionList selection_list;
    selection_list.add(polygon_name + ".vtx[*]");
    selection_list.add(skin_cluster_name);
    MDagPath path;
    MObject components;
    MObject skin_depend_node;
    selection_list.getDagPath(0, path, components);
    selection_list.getDependNode(1, skin_depend_node);
    MString command = "dgdirty " + skin_cluster_name;
    MGlobal::executeCommand(command);
    MFnSkinCluster fn_skin(skin_depend_node);
    fn_skin.setWeights(path, components, indices, weights);
}

void get_max_weights(MString polygon_name, MString skin_cluster_name, MIntArray& indices, MDoubleArray& max_weights) {
    MDoubleArray weights;
    get_weights(polygon_name, skin_cluster_name, indices, weights);
    int joint_length = indices.length();
    int vtx_length = weights.length() / joint_length;
    max_weights.setLength(vtx_length);
    tbb::parallel_for(0, vtx_length, 1, [joint_length, &weights,  &max_weights](int vtx_id) {
        max_weights[vtx_id] = 0;
        int weight_id = vtx_id * joint_length;
        for (int joint_id = 0; joint_id < joint_length; joint_id++) {
            max_weights[vtx_id] += weights[weight_id + joint_id];
        }
    }
    );
}


inline double Dot(MVector p1, MVector p2) {
    return p1[0] * p2[0] + p1[1] * p2[1] + p1[2] * p1[2];
}

inline MVector get_near_point(MVector p1, MVector p2, MVector p3) {
    // 求线段p2-p3上距离p1最近的点

    MVector p1_p2 = p1 - p2;
    MVector p3_p2 = p3 - p2;
    if (Dot(p1_p2, p3_p2) <= 0) {
        return p2;
    }
    if (Dot(p1 - p3, p2 - p3) <= 0) {
        return p3;
    }
    double x = Dot(p3_p2, p1_p2) / Dot(p3_p2, p3_p2);
    MVector p4 = p2 + p3_p2 * x;
    return p4;
}


double chain_distance(MVector vtx_point, MVectorArray& joint_points, int first_index, int chain_length) {
    double min_distance = (joint_points[first_index] - vtx_point).length();
    for (int chain_id = 0; chain_id < (chain_length -1); chain_id++) {
        MVector near_point = get_near_point(vtx_point, joint_points[first_index + chain_id], joint_points[first_index + chain_id + 1]);
        double near_distance = (vtx_point - near_point).length();
        if (near_distance < min_distance) {
            min_distance = near_distance;
        }
    }
    return min_distance;
}

class WeightSolve {

public:
    MString polygon_name;
    MString skin_cluster_name;
    MIntArray indices;
    MDoubleArray max_weights;
    MDoubleArray wxs;
    MIntArray chain_lengths;
    void solve(MString typ, MDoubleArray& xs, MDoubleArray& ys, double r);
    void ik_init(MString polygon_name, MString _skin_cluster_name, MDoubleArray& vx);
    void ik_solve(double xs[4], double ys[4], double r);
    void soft_init(MString _polygon_name, MString _skin_cluster_name, MString up_cmd, MString dn_cmd);
    void soft_solve(double xs[4], double ys[4], double r);
    void arg_init(MString _polygon_name, MString _skin_cluster_name, MIntArray& _indices);
    void split_init(MString _polygon_name, MString _skin_cluster_name, MIntArray& _indices, MDoubleArray& vxs);
    void split_solve(double xs[4], double ys[4], double r);
    void points_init(MString _polygon_name, MString _skin_cluster_name, MIntArray& _indices, MPointArray& joint_points);
    void points_solve(double xs[4], double ys[4], double r);
    void chains_init(MString _polygon_name, MString _skin_cluster_name, MIntArray& _indices, MVectorArray& joint_points, MIntArray& chain_lengths, MIntArray& top_indices);
    void finger_init(MString _polygon_name, MString _skin_cluster_name, MIntArray& _indices, MDoubleArray& vxs, MIntArray& chain_lengths);
    void finger_solve(double xs[4], double ys[4], double r);
};


void WeightSolve::solve(MString typ, MDoubleArray& _xs, MDoubleArray& _ys, double r) {
    double xs[4] = {_xs[0], _xs[1], _xs[2], _xs[3]};
    double ys[4] = {_ys[0], _ys[1], _ys[2], _ys[3]};
    if (typ == "ik") {
        ik_solve(xs, ys, r);
    }
    if (typ == "soft") {
        soft_solve(xs, ys, r);
    }
    if (typ == "split") {
        split_solve(xs, ys, r);
    }
    if (typ == "points") {
        points_solve(xs, ys, r);
    }
    if (typ == "finger") {
        finger_solve(xs, ys, r);
    }
    
}


void WeightSolve::ik_init(MString polygon_name, MString _skin_cluster_name, MDoubleArray& vx) {
    skin_cluster_name = _skin_cluster_name;
    MPointArray points;
    get_mesh_points(polygon_name, points);
    int vtx_length = points.length();
    wxs.setLength(vtx_length);
    tbb::parallel_for(0, vtx_length, 1, [this, &points, &vx](int vtx_id) {
        wxs[vtx_id] = ik_x(vx, points[vtx_id]) - vx[3];
    }
    );
}



void WeightSolve::ik_solve(double xs[4], double ys[4], double r) {
    int vtx_length = wxs.length();
    MDoubleArray paint_weights(vtx_length, 0);
    tbb::parallel_for(0, vtx_length, 1, [this, &xs, &ys, r, &paint_weights](int vtx_id) {
        paint_weights[vtx_id] = get_weight(wxs[vtx_id] / r + 0.5, xs, ys);
    }
    );
    set_paint_weights(skin_cluster_name, paint_weights);
}

void WeightSolve::soft_init(MString polygon_name, MString _skin_cluster_name, MString up_cmd, MString dn_cmd) {
    skin_cluster_name = _skin_cluster_name;
    MPointArray old_points;
    MPointArray new_points;
    get_mesh_points(polygon_name, old_points);
    MGlobal::executeCommand(up_cmd);
    get_mesh_points(polygon_name, new_points);
    MGlobal::executeCommand(dn_cmd);
    int vtx_length = old_points.length();
    wxs.setLength(vtx_length);
    tbb::parallel_for(0, vtx_length, 1, [this, &old_points, &new_points](int vtx_id) {
        wxs[vtx_id] = 2 * (1 - (new_points[vtx_id][1] - old_points[vtx_id][1]));
    }
    );
}

void WeightSolve::soft_solve(double xs[4], double ys[4], double r){
    MDoubleArray paint_weights;
    int vtx_length = wxs.length();
    paint_weights.setLength(vtx_length);
    tbb::parallel_for(0, vtx_length, 1, [this, &xs, &ys, r, &paint_weights](int vtx_id) {
        paint_weights[vtx_id] = get_weight(wxs[vtx_id] / r, xs, ys);
    }
    );
    set_paint_weights(skin_cluster_name, paint_weights);
}

void WeightSolve::arg_init(MString _polygon_name, MString _skin_cluster_name, MIntArray& _indices) {
    polygon_name = _polygon_name;
    skin_cluster_name = _skin_cluster_name;
    indices.copy(_indices);
    get_max_weights(polygon_name, skin_cluster_name, indices, max_weights);
}

void WeightSolve::split_init(MString _polygon_name, MString _skin_cluster_name, MIntArray& _indices, MDoubleArray& vxs) {
    arg_init(_polygon_name, _skin_cluster_name, _indices);
    MPointArray points;
    get_mesh_points(polygon_name, points);
    int vtx_length = points.length();
    int split_length = vxs.length() / 4;
    wxs.setLength(split_length * vtx_length);
    tbb::parallel_for(0, vtx_length, 1, [this, split_length, vtx_length, &points, &vxs](int vtx_id) {
        int wxs_id = vtx_id * split_length;
        for (int split_id = 0; split_id < split_length; split_id++) {
            int vx_id = split_id * 4;
            MVector v = MVector(vxs[vx_id], vxs[vx_id + 1], vxs[vx_id + 2]);
            wxs[wxs_id + split_id] = ik_x(v, points[vtx_id]) - vxs[vx_id + 3];
        }
    }
    );
}

void WeightSolve::split_solve(double xs[4], double ys[4], double r) {
    MDoubleArray weights;
    int vtx_length = max_weights.length();
    weights.setLength(wxs.length() + vtx_length);
    tbb::parallel_for(0, vtx_length, 1, [this, &xs, &ys, r, &weights](int vtx_id) {
        int vtx_length = max_weights.length();
        int split_length = wxs.length() / vtx_length;
        int joint_length = split_length + 1;
        int weight_id = vtx_id * joint_length;
        weights[weight_id] = 1.0;
        int wx_id = vtx_id * split_length;
        for (int split_id = 0; split_id < split_length; split_id++) {
            int w1 = weight_id + split_id;
            int w2 = w1 + 1;
            weights[w2] = get_weight(wxs[wx_id + split_id] / r + 0.5, xs, ys);
            if (weights[w2] >= weights[w1]) {
                weights[w2] = weights[w1];
            }
            weights[w1] = weights[w1] - weights[w2];
        }
        for (int joint_id = 0; joint_id < joint_length; joint_id++) {
            weights[weight_id + joint_id] = weights[weight_id + joint_id] * max_weights[vtx_id];
        }
    }
    );
    set_weights(polygon_name, skin_cluster_name, indices, weights);
}


void WeightSolve::points_init(MString _polygon_name, MString _skin_cluster_name, MIntArray& _indices, MPointArray& joint_points) {
    arg_init(_polygon_name, _skin_cluster_name, _indices);
    MPointArray points;
    get_mesh_points(polygon_name, points);
    int vtx_length = points.length();
    int joint_length = joint_points.length();
    wxs.setLength(joint_length * vtx_length);
    tbb::parallel_for(0, vtx_length, 1, [this, joint_length, vtx_length, &points, joint_points](int vtx_id) {
        int wxs_id = vtx_id * joint_length;
        for (int joint_id = 0; joint_id < joint_length; joint_id++) {
            wxs[wxs_id + joint_id] = joint_points[joint_id].distanceTo(points[vtx_id]);
        }
    }
    );
}

void WeightSolve::points_solve(double xs[4], double ys[4], double r) {
    MDoubleArray weights;
    int vtx_length = max_weights.length();
    weights.setLength(wxs.length());
    tbb::parallel_for(0, vtx_length, 1, [this, &xs, &ys, r, &weights](int vtx_id) {
        int vtx_length = max_weights.length();
        int joint_length = wxs.length() / vtx_length;
        int weight_id = vtx_id * joint_length;
        double vtx_weight = 0.0;
        for (int joint_id = 0; joint_id < joint_length; joint_id++) {
            int index = weight_id + joint_id;
            weights[index] = get_weight(wxs[index] / r, xs, ys);
            vtx_weight += weights[index];
        }
        if (vtx_weight < 0.00001) {
            vtx_weight = 1.0;
            int near_joint_id = 0;
            double near_distance = wxs[weight_id];
            for (int joint_id = 0; joint_id < joint_length; joint_id++) {
                int index = weight_id + joint_id;
                weights[index] = 0.0;
                double current_distance = wxs[index];
                if (current_distance < near_distance) {
                    near_distance = current_distance;
                    near_joint_id = joint_id;
                }
            }
            weights[weight_id + near_joint_id] = 1.0;
        }
        for (int joint_id = 0; joint_id < joint_length; joint_id++) {
            int index = vtx_id * joint_length + joint_id;
            weights[index] = weights[index] / vtx_weight * max_weights[vtx_id];
        }
    }
    );
    set_weights(polygon_name, skin_cluster_name, indices, weights);
}

void WeightSolve::chains_init(MString _polygon_name, MString _skin_cluster_name, MIntArray& _indices, MVectorArray& joint_points, MIntArray& chain_lengths, MIntArray& top_indices) {
    arg_init(_polygon_name, _skin_cluster_name, _indices);
    MPointArray points;
    get_mesh_points(polygon_name, points);
    int vtx_length = max_weights.length();
    int joint_length = indices.length();
    MDoubleArray weights(vtx_length * joint_length, 0);
    int top_id = 0;
    for (int joint_id = 0; joint_id < joint_length; joint_id++) {
        if (indices[joint_id] == top_indices[0]) {
            top_id = joint_id;
            break;
        }
    }
    int top_length = top_indices.length();
    wxs.setLength(vtx_length * top_length);
    tbb::parallel_for(0, vtx_length, 1, [this, joint_length, top_length, top_id, &joint_points, &chain_lengths, &points, &weights](int vtx_id) {
        weights[vtx_id * joint_length + top_id] = 1.0;
        int wxs_id = vtx_id * top_length;
        int first_index = 0;
        for (int top_id = 0; top_id < top_length; top_id++) {
            int chain_length = chain_lengths[top_id];
            MVector vtx_point = MVector(points[vtx_id].x, points[vtx_id].y, points[vtx_id].z);
            wxs[wxs_id + top_id] = chain_distance(vtx_point, joint_points, first_index, chain_length);
            first_index += chain_length;
        }
    }
    );
    set_weights(polygon_name, skin_cluster_name, indices, weights);
    indices.copy(top_indices);
}


void WeightSolve::finger_init(MString _polygon_name, MString _skin_cluster_name, MIntArray& _indices, MDoubleArray& vxs, MIntArray& _chain_lengths) {
    polygon_name = _polygon_name;
    skin_cluster_name = _skin_cluster_name;
    indices.copy(_indices);
    chain_lengths.copy(_chain_lengths);

    MPointArray points;
    get_mesh_points(polygon_name, points);

    MDoubleArray weights;
    get_weights(polygon_name, skin_cluster_name, indices, weights);

    int joint_length = indices.length();
    int vtx_length = points.length();
    int top_length = chain_lengths.length();
    int vx_length = vxs.length() / 4;

    max_weights.setLength(top_length * vtx_length);
    wxs.setLength(vx_length * vtx_length);

    tbb::parallel_for(0, vtx_length, 1, [this, joint_length, top_length, vx_length, &vxs, &points, &weights](int vtx_id) {
        int wx_id = vtx_id * vx_length;
        for (int vx_id = 0; vx_id < vx_length; vx_id++) {
            int slice_vx_id = vx_id * 4;
            MVector v(vxs[slice_vx_id + 0], vxs[slice_vx_id + 1], vxs[slice_vx_id + 2]);
            double x = vxs[slice_vx_id + 3];
            wxs[wx_id + vx_id] = ik_x(v, points[vtx_id]) - x;
        }

        int slice_chain_id = 0;
        for (int top_id = 0; top_id < top_length; top_id++) {
            int chain_length = chain_lengths[top_id];
            int weight_id = vtx_id * joint_length;
            int max_weight_id = vtx_id * top_length;
            double weight = 0;
            for (int chain_id = 0; chain_id < chain_length; chain_id++) {
                weight += weights[weight_id + slice_chain_id + chain_id];
            }
            max_weights[max_weight_id + top_id] = weight;
            slice_chain_id += chain_length;
        }
    });

};


void WeightSolve::finger_solve(double xs[4], double ys[4], double r) {
    int top_length = chain_lengths.length();
    int vtx_length = max_weights.length() / top_length;
    int joint_length = indices.length();
    int wx_length = wxs.length() / vtx_length;
    MDoubleArray weights(vtx_length * joint_length);
    tbb::parallel_for(0, vtx_length, 1, [this, joint_length, wx_length, top_length, &weights, r, &xs, &ys](int vtx_id) {
        int weight_id = vtx_id * joint_length;
        int wx_id = vtx_id * wx_length;
        for (int top_id = 0; top_id < top_length; top_id++) {
            int chain_length = chain_lengths[top_id];
            int split_length = chain_length - 1;
            weights[weight_id] = 1.0;
            for (int split_id = 0; split_id < split_length; split_id++) {
                int w1 = weight_id + split_id;
                int w2 = w1 + 1;
                weights[w2] = get_weight(wxs[wx_id + split_id] / r + 0.5, xs, ys);
                if (weights[w2] >= weights[w1]) {
                    weights[w2] = weights[w1];
                }
                weights[w1] = weights[w1] - weights[w2];
            }
            double max_weight = max_weights[vtx_id * top_length + top_id];
            for (int chain_id = 0; chain_id < chain_length; chain_id++) {
                weights[weight_id + chain_id] = weights[weight_id + chain_id] * max_weight;
            }
            weight_id += chain_length;
            wx_id += split_length;
        }
    });
    set_weights(polygon_name, skin_cluster_name, indices, weights);
}
WeightSolve weightSolve;

void solve(MString typ, MDoubleArray& xs, MDoubleArray& ys, double r) {
    weightSolve.solve(typ, xs, ys, r);
}
void ik_init(MString polygon_name, MString _skin_cluster_name, MDoubleArray& vx) {
    weightSolve.ik_init(polygon_name, _skin_cluster_name, vx);
}
void soft_init(MString _polygon_name, MString _skin_cluster_name, MString up_cmd, MString dn_cmd) {
    weightSolve.soft_init(_polygon_name, _skin_cluster_name, up_cmd, dn_cmd);
}
void split_init(MString _polygon_name, MString _skin_cluster_name, MIntArray& _indices, MDoubleArray& vxs) {
    weightSolve.split_init(_polygon_name, _skin_cluster_name, _indices, vxs);
}
void points_init(MString _polygon_name, MString _skin_cluster_name, MIntArray& _indices, MPointArray& joint_points) {
    weightSolve.points_init(_polygon_name, _skin_cluster_name, _indices, joint_points);
}
void chains_init(MString _polygon_name, MString _skin_cluster_name, MIntArray& _indices, MVectorArray& joint_points, MIntArray& chain_lengths, MIntArray& top_indices) {
    weightSolve.chains_init(_polygon_name, _skin_cluster_name, _indices, joint_points, chain_lengths, top_indices);
}
void finger_init(MString _polygon_name, MString _skin_cluster_name, MIntArray& _indices, MDoubleArray& vxs, MIntArray& chain_lengths) {
    weightSolve.finger_init(_polygon_name, _skin_cluster_name, _indices, vxs, chain_lengths);
}

FunCToPy4(solve, MString, MDoubleArray, MDoubleArray, double)
FunCToPy3(ik_init, MString, MString, MDoubleArray)
FunCToPy4(soft_init, MString, MString, MString, MString)
FunCToPy4(split_init, MString, MString, MIntArray, MDoubleArray)
FunCToPy4(points_init, MString, MString, MIntArray, MPointArray)
FunCToPy6(chains_init, MString, MString, MIntArray, MVectorArray, MIntArray, MIntArray)
FunCToPy5(finger_init, MString, MString, MIntArray, MDoubleArray, MIntArray)

PyDefList(
    PyDef(solve),
    PyDef(ik_init),
    PyDef(soft_init),
    PyDef(split_init),
    PyDef(points_init),
    PyDef(chains_init),
    PyDef(finger_init)
)

PyMod(bezier_api)
