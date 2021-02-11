#pragma once

#include "point_cloud.h"
#include "neighbor_graph.h"

#include "lib_begin.h"

enum BilateralWeightType {
	BWT_GAUSS_ON_NORMALS,
	BWT_GAUSS_ON_PLANE_DISTANCE
};

/** the normal estimator class needs a reference to a point_cloud and a neighbor_graph and allows to
    compute [[bilaterally] weighted] least squares normals and to consistently orient the normals */
class CGV_API normal_estimator : public point_cloud_types
{
protected:
	point_cloud& pc;
	neighbor_graph& ng;
public:
	Crd normal_quality_exp;

	Crd localization_scale;
	Crd plane_distance_scale;
	Crd normal_sigma;

	Crd noise_to_sampling_ratio;
	bool use_orientation;
	BilateralWeightType bw_type;

	Crd compute_normal_quality(const Pnt& p1, const Nml& n1, const Pnt& p2, const Nml& n2, Crd l0) const;
public:
	/// construct from point cloud and neighbor graph
	normal_estimator(point_cloud& _pc, neighbor_graph& _ng);
	/// smooth normals with bilateral weights
	void smooth_normals();
	/// fill the given vector with the weights to the reference point and the neighbors
	/// ok, not normalized? 
	void compute_weights(Idx vi, std::vector<Crd>& weights, std::vector<Pnt>* points_ptr = 0) const;
	
	/// ok,@yzy basic normal compuitation 
	void compute_normals_ls();
	/// ok, used to compute weights 
	Crd estimate_scale(Idx vi) const;
	/// (re)compute normals from neighbor graph and distance weights
	/// ok, the simplest way to compute normals 
	void compute_weighted_normals(bool reorient);
	/// ok, just add an other weight 
	void compute_bilateral_weights(Idx vi, std::vector<Crd>& weights, std::vector<Pnt>* points_ptr = 0) const;
	/// ok, recompute normals from neighbor graph and distance and normal weights
	/// refine the normals 
	void compute_bilateral_weighted_normals(bool reorient);
	/// recompute normals from neighbor graph and distance and normal weights
	/// ok, an other weight refine approach 
	void compute_plane_bilateral_weighted_normals(bool reorient);
	/// try to compute global consistent normal orientation
	/// ok, can be found on paper 
	void orient_normals();
	/// orient normals towards given point
	/// ok, strightforward 
	void orient_normals(const Pnt& view_point);
	///
	void set_pc(point_cloud& _pc) {
		pc = _pc;
	}
	///
	void set_ng(neighbor_graph& _ng) {
		ng = _ng;
	}
};

#include <cgv/config/lib_end.h>
