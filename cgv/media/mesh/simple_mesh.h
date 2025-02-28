#pragma once

#include <vector>
#include <cgv/math/fvec.h>
#include <cgv/math/fmat.h>
#include <cgv/utils/file.h>
#include <cgv/media/illum/textured_surface_material.h>
#include <cgv/media/axis_aligned_box.h>
#include <cgv/media/colored_model.h>
#include <cgv/math/quaternion.h>
#include <cgv/math/geom.h>
#include <cgv/render/render_types.h>

#include "../lib_begin.h"

namespace cgv {
	namespace media {
		namespace mesh {
			
template <typename T>
class simple_mesh_obj_reader;

/** coordinate type independent base class of simple mesh data structure that handles indices and colors. */
class CGV_API simple_mesh_base : public colored_model
{
public:
	/// define index type
	typedef cgv::type::uint32_type idx_type;
	/// define index pair type
	typedef cgv::math::fvec<idx_type, 2> vec2i;
	/// define index triple type
	typedef cgv::math::fvec<idx_type, 3> vec3i;
	/// define material type
	typedef illum::textured_surface_material mat_type;
protected:
	std::vector<idx_type> position_indices;
	std::vector<idx_type> normal_indices;
	std::vector<idx_type> tex_coord_indices;
	std::vector<idx_type> faces;
	std::vector<idx_type> group_indices;
	std::vector<std::string> group_names;
	std::vector<idx_type> material_indices;
	std::vector<mat_type> materials;
public:
	/// position count
	virtual idx_type get_nr_positions() const = 0;
	/// create a new empty face to which new corners are added and return face index
	idx_type start_face();
	/// create a new corner from position, optional normal and optional tex coordinate indices and return corner index
	idx_type new_corner(idx_type position_index, idx_type normal_index = -1, idx_type tex_coord_index = -1);
	/// return position index of corner
	idx_type c2p(idx_type ci) const { return position_indices[ci]; }
	/// return normal index of corner
	idx_type c2n(idx_type ci) const { return normal_indices[ci]; }
	/// return texture index of corner
	idx_type c2t(idx_type ci) const { return tex_coord_indices[ci]; }
	/// return the number of faces
	idx_type get_nr_faces() const { return idx_type(faces.size()); }
	/// return the number of corners
	idx_type get_nr_corners() const { return idx_type(position_indices.size()); }
	/// return index of first corner of face with index fi
	idx_type begin_corner(idx_type fi) const { return faces[fi]; }
	/// return index of end corner (one after the last one) of face with index fi
	idx_type end_corner(idx_type fi) const { return fi + 1 == faces.size() ? idx_type(position_indices.size()) : faces[fi + 1]; }
	/// return number of edges/corners of face with index fi
	idx_type face_degree(idx_type fi) const { return end_corner(fi) - begin_corner(fi); }
	/// return number of materials in mesh
	size_t get_nr_materials() const { return materials.size(); }
	/// add a new material and return its index
	idx_type new_material() { materials.push_back(mat_type()); return idx_type(materials.size() - 1); }
	/// return const reference to i-th material
	const mat_type& get_material(size_t i) const { return materials[i]; }
	/// return reference to i-th material
	mat_type& ref_material(size_t i) { return materials[i]; }
	/// return material index of given face
	const idx_type& material_index(idx_type fi) const { return material_indices[fi]; }
	/// return reference to material index of given face
	idx_type& material_index(idx_type fi) { return material_indices[fi]; }
	/// return number of face groups
	size_t get_nr_groups() const { return group_names.size(); }
	/// return the name of the i-th face group
	const std::string& group_name(size_t i) const { return group_names[i]; }
	/// set a new group name
	std::string& group_name(size_t i) { return group_names[i]; }
	/// add a new group and return its index
	idx_type new_group(const std::string& name) { group_names.push_back(name); return idx_type(group_names.size() - 1); }
	/// return group index of given face
	const idx_type& group_index(idx_type fi) const { return group_indices[fi]; }
	/// return reference to group index of given face
	idx_type& group_index(idx_type fi) { return group_indices[fi]; }
	/// revert face orientation
	void revert_face_orientation();
	/// sort faces by group and material indices with two bucket sorts
	void sort_faces(std::vector<idx_type>& perm, bool by_group = true, bool by_material = true) const;
	/// merge the three indices into one index into a vector of unique index triples
	void merge_indices(std::vector<idx_type>& vertex_indices, std::vector<vec3i>& unique_triples, bool* include_tex_coords_ptr = 0, bool* include_normals_ptr = 0) const;
	/// extract element array buffers for triangulation
	void extract_triangle_element_buffer(const std::vector<idx_type>& vertex_indices, std::vector<idx_type>& triangle_element_buffer, 
		const std::vector<idx_type>* face_perm_ptr = 0, std::vector<vec3i>* material_group_start_ptr = 0) const;
	/// extract element array buffers for edges in wireframe
	void extract_wireframe_element_buffer(const std::vector<idx_type>& vertex_indices, std::vector<idx_type>& edge_element_buffer) const;
	/// compute a index vector storing the inv corners per corner and optionally index vectors with per position corner index, per corner next and or prev corner index (implementation assumes closed manifold connectivity)
	void compute_inv(std::vector<uint32_t>& inv, std::vector<uint32_t>* p2c_ptr = 0, std::vector<uint32_t>* next_ptr = 0, std::vector<uint32_t>* prev_ptr = 0);
	/// given the inv corners compute index vector per corner its edge index and optionally per edge its corner index and return edge count (implementation assumes closed manifold connectivity)
	uint32_t compute_c2e(const std::vector<uint32_t>& inv, std::vector<uint32_t>& c2e, std::vector<uint32_t>* e2c_ptr = 0);
	/// compute index vector with per corner its face index
	void compute_c2f(std::vector<uint32_t>& c2f);
};

/// the simple_mesh class is templated over the coordinate type that defaults to float
template <typename T = float>
class CGV_API simple_mesh : public simple_mesh_base
{
public:
	/// type of axis aligned 3d box
	typedef typename simple_mesh<T> mesh_type;
	/// type of axis aligned 3d box
	typedef typename cgv::media::axis_aligned_box<T, 3> box_type;
	/// type of 3d vector
	typedef typename cgv::math::fvec<T, 3> vec3;
	/// type of 2d vector
	typedef typename cgv::math::fvec<T, 2> vec2;
	/// linear transformation 
	typedef typename cgv::math::fmat<T, 3, 3> mat3;
	///
	typedef cgv::math::quaternion<T> quat;
	/// color type used in surface materials
	typedef typename illum::surface_material::color_type clr_type;
	/// textured surface materials are supported by mat_type
	typedef typename illum::textured_surface_material mat_type;
	/// 32bit index
	typedef cgv::type::uint32_type idx_type;
protected:
	friend class simple_mesh_obj_reader<T>;
	std::vector<vec3>  positions;
	std::vector<vec3>  normals;
	std::vector<vec2>  tex_coords;

	vec3 compute_normal(const vec3& p0, const vec3& p1, const vec3& p2);
public:
	/// construct from string corresponding to Conway notation (defaults to empty mesh)
	simple_mesh(const std::string& conway_notation = "");
	
	/// clear simple mesh
	void clear();

	/// add a new position and return position index
	idx_type new_position(const vec3& p) { positions.push_back(p); return idx_type(positions.size()-1); }
	/// access to positions
	idx_type get_nr_positions() const { return idx_type(positions.size()); }
	vec3& position(idx_type pi) { return positions[pi]; }
	const vec3& position(idx_type pi) const { return positions[pi]; }
	const std::vector<vec3>& get_positions() const { return positions; }

	/// add a new normal and return normal index
	idx_type new_normal(const vec3& n) { normals.push_back(n); return idx_type(normals.size() - 1); }
	/// access to normals
	bool has_normals() const { return get_nr_normals() > 0; }
	idx_type get_nr_normals() const { return idx_type(normals.size()); }
	vec3& normal(idx_type ni) { return normals[ni]; }
	const vec3& normal(idx_type ni) const { return normals[ni]; }

	/// add a new normal and return normal index
	idx_type new_tex_coord(const vec2& tc) { tex_coords.push_back(tc); return idx_type(tex_coords.size() - 1); }
	/// access to texture coordinates
	bool has_tex_coords() const { return get_nr_tex_coords() > 0; }
	idx_type get_nr_tex_coords() const { return idx_type(tex_coords.size()); }
	vec2& tex_coord(idx_type ti) { return tex_coords[ti]; }
	const vec2& tex_coord(idx_type ti) const { return tex_coords[ti]; }
	/// compute per face normals (ensure that per corner normal indices are set correspondingly)
	void compute_face_normals();

	void compute_ray_mesh_intersections(vec3 ray);

	void randomize_texcoordi();

	// p1 and p2 are corresp. points 
	void compute_texcoordi(float rx, float ry, float rz, vec3 camera_intrinsic)
	{
		tex_coords.clear();
		tex_coord_indices.clear();
		for (int i = 0; i < get_nr_positions(); i++) {

		vec3 cposi_1 = vec3(0, 0, 0);
		quat q_1(-0.9999385663968424, 0.010694446357222644, 0.002913979700329364, 3.1165314188749525e-05);

		vec3 cposi_2 = vec3(-5.25745261382357, -6.25186680269724, -0.00572032780798151);
		quat q_2(-0.998471406259843, 0.0012813314616936773, 0.004791197678897108, 0.055047738336842185);

		vec3 cposi_3 = vec3(-11.372755480459, -10.2836719679026, -0.000375695328008629);
		quat q_3(-0.9908034418640084, 0.006686899799069564, 0.0050330476422138885, 0.13504996628264998);


		// choose which to test 
		vec3 camposi = cposi_1;
		quat camq = q_1;

		// translation only: align leica with the cgv coordinates 
		quat rotz = quat(vec3(0, 0, 1), 25 * M_PI / 180);
		quat rotx = quat(vec3(1, 0, 0), -90 * M_PI / 180);
		quat r_align = rotx * rotz;
		r_align.rotate(camposi);
		//camposi += vec3(0, h, 0);
		camq = r_align * camq * r_align.inverse();

		// camera space alignment
		quat alinmentq_x = quat(vec3(1, 0, 0), rx * M_PI / 180);
		quat alinmentq_y = quat(vec3(0, 1, 0), (-115 + ry) * M_PI / 180);// 115 = 90 + 25 
		quat alinmentq_z = quat(vec3(0, 0, 1), rz * M_PI / 180);
		camq = alinmentq_x * alinmentq_y * alinmentq_z * camq;

		// compute camera-to-point direction
		vec3 pdir = positions.at(i) - camposi;
		pdir.y() = camera_intrinsic.y() * pdir.y() + camera_intrinsic.z(); // internal camera parameter
		pdir.normalize();
		camq.rotate(pdir);

		// reject some points with normal information 
		// control by face selection 
		if (dot(pdir, normals.at(i)) < 0.2) {
			tex_coords.push_back(vec2(-1, -1));
			continue;
		}

		// from pdir to uv  
		float u = atan2(pdir.x(), pdir.z()) / (2 * M_PI) + 0.5;
		float v = pdir.y() * 0.5 + 0.5;
		tex_coords.push_back(vec2(1 - u, v));
	}
	for (int i = 0; i < get_nr_corners(); i++) {
		tex_coord_indices.push_back(position_indices.at(i));
	}
}

	void coordi_correction_leica() {
		for (int i = 0; i < get_nr_positions(); i++) {
			positions.at(i) = positions.at(i) - vec3(0, 1, 0);
		}
	}

	void pick_face(vec3 p) {
		
		//for (int pi = 0; pi < positions.size(); pi++) {
		//	set_color(pi, rgb(1, 0, 0));
		//}
		
		std::vector<vec3> points_per_face;
		for (int fidx = 0; fidx < faces.size(); fidx++) {
			// each face 
			idx_type pidx_1 = position_indices.at(faces.at(fidx));
			idx_type pidx_2 = position_indices.at(faces.at(fidx) + 1);
			idx_type pidx_3 = position_indices.at(faces.at(fidx) + 2);

			vec3 p_1 = positions.at(pidx_1);
			vec3 p_2 = positions.at(pidx_2);
			vec3 p_3 = positions.at(pidx_3);

			//set_color(pidx_1, rgb(1, 1, 0));
			//set_color(pidx_2, rgb(1, 1, 0));
			//set_color(pidx_3, rgb(1, 1, 0));

			vec3 face_n = (normals.at(pidx_1) + normals.at(pidx_1) + normals.at(pidx_1)) / 3.0;

			if (dot(cross(p_2 - p_1, p - p_1), face_n) >= 0){
				if (dot(cross(p_3 - p_2, p - p_2), face_n) >= 0) {
					if (dot(cross(p_1 - p_3, p - p_3), face_n) >= 0) {
						// inside! change color! 
						set_color(pidx_1, rgb(1, 1, 0));
						set_color(pidx_2, rgb(1, 1, 0));
						set_color(pidx_3, rgb(1, 1, 0));
					}
				}
			}
		}
	}

	/// Conway ambo operator
	void ambo();
	/// Conway truncate operator
	void truncate(T lambda = 0.33333f);
	/// Conway snub operator
	void snub(T lambda = 0.33333f);
	/// Conway dual operator
	void dual();
	/// Conway gyro operator
	void gyro(T lambda = 0.3333f);
	/// Conway join operator
	void join();
	/// Conway ortho operator
	void ortho();
	/// construct new mesh according to Conway polyhedron notation: [a|t|s|d|g|j|o]*[T|C|O|D|I] which is evaluated from right to left and last capital letter is Platonic solid and lowercase letters are Conway operations
	void construct_conway_polyhedron(const std::string& conway_notation);

	/// compute the axis aligned bounding box
	box_type compute_box() const;
	/// compute vertex normals by averaging triangle normals
	void compute_vertex_normals();
	/// read simple mesh from file (currently only obj is supported)
	bool read(const std::string& file_name);
	/// write simple mesh to file (currently only obj is supported)
	bool write(const std::string& file_name) const;

	bool write_with_materials(const std::string& file_name);
	/// extract vertex attribute array, return size of color in bytes
	unsigned extract_vertex_attribute_buffer(
		const std::vector<idx_type>& vertex_indices,
		const std::vector<vec3i>& unique_triples,
		bool include_tex_coords, bool include_normals, 
		std::vector<T>& attrib_buffer, bool *include_colors_ptr = 0) const;
	/// apply transformation to mesh
	void transform(const mat3& linear_transformation, const vec3& translation);
	/// apply transformation to mesh with given inverse linear transformation
	void transform(const mat3& linear_transform, const vec3& translation, const mat3& inverse_linear_transform);
};

		}
	}
}

#include <cgv/config/lib_end.h>