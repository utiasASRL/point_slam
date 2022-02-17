//
//
//		0==========================0
//		|    Local feature test    |
//		0==========================0
//
//		version 1.0 : 
//			> 
//
//---------------------------------------------------
//
//		Cloud header
//
//----------------------------------------------------
//
//		Hugues THOMAS - 10/02/2017
//


# pragma once

#include <vector>
#include <unordered_map>
#include <map>
#include <algorithm>
#include <numeric>
#include <iostream>
#include <iomanip>
#include <cmath>
#include <stdio.h>
#include <string.h>
#include <chrono>
#include <random>


#include <time.h>

#include "../npm_ply/ply_types.h"
#include "../npm_ply/ply_file_in.h"
#include "../npm_ply/ply_file_out.h"

#include <Eigen/Eigenvalues>


//------------------------------------------------------------------------------------------------------------
// Point class
// ***********
//
//------------------------------------------------------------------------------------------------------------

class PointXYZ
{
public:

	// Elements
	// ********

	float x, y, z;


	// Methods
	// *******
	
	// Constructor
	PointXYZ() { x = 0; y = 0; z = 0; }
	PointXYZ(float x0, float y0, float z0) { x = x0; y = y0; z = z0; }
	
	// array type accessor
	float& operator[](std::size_t idx)       
	{
		if (idx == 2) return z;
		else if (idx == 1) return y;
		else  return x;
	}
    const float& operator[](std::size_t idx) const
	{
		if (idx == 2) return z;
		else if (idx == 1) return y;
		else return x;
	}

	// opperations
	float dot(const PointXYZ P) const
	{
		return x * P.x + y * P.y + z * P.z;
	}

	float sq_norm()
	{
		return x*x + y*y + z*z;
	}

	PointXYZ cross(const PointXYZ P) const
	{
		return PointXYZ(y*P.z - z*P.y, z*P.x - x*P.z, x*P.y - y*P.x);
	}	

	PointXYZ& operator+=(const PointXYZ& P)
	{
		x += P.x;
		y += P.y;
		z += P.z;
		return *this;
	}

	PointXYZ& operator-=(const PointXYZ& P)
	{
		x -= P.x;
		y -= P.y;
		z -= P.z;
		return *this;
	}

	PointXYZ& operator*=(const float& a)
	{
		x *= a;
		y *= a;
		z *= a;
		return *this;
	}
};


// Point Opperations
// *****************

inline PointXYZ operator + (const PointXYZ A, const PointXYZ B)
{
	return PointXYZ(A.x + B.x, A.y + B.y, A.z + B.z);
}

inline PointXYZ operator - (const PointXYZ A, const PointXYZ B)
{
	return PointXYZ(A.x - B.x, A.y - B.y, A.z - B.z);
}

inline PointXYZ operator * (const PointXYZ P, const float a)
{
	return PointXYZ(P.x * a, P.y * a, P.z * a);
}

inline PointXYZ operator * (const float a, const PointXYZ P)
{
	return PointXYZ(P.x * a, P.y * a, P.z * a);
}

inline std::ostream& operator << (std::ostream& os, const PointXYZ P)
{
	return os << "[" << P.x << ", " << P.y << ", " << P.z << "]";
}

inline bool operator == (const PointXYZ A, const PointXYZ B)
{
	return A.x == B.x && A.y == B.y && A.z == B.z;
}

inline PointXYZ floor(const PointXYZ P)
{
	return PointXYZ(std::floor(P.x), std::floor(P.y), std::floor(P.z));
}

PointXYZ max_point(const std::vector<PointXYZ>& points);
PointXYZ min_point(const std::vector<PointXYZ>& points);
PointXYZ max_point(const PointXYZ A, const PointXYZ B);
PointXYZ min_point(const PointXYZ A, const PointXYZ B);



//------------------------------------------------------------------------------------------------------------
// Point class 2D
// **************
//
//------------------------------------------------------------------------------------------------------------

class PointXY
{
public:

	// Elements
	// ********

	float x, y;


	// Methods
	// *******
	
	// Constructor
	PointXY() { x = 0; y = 0;}
	PointXY(float x0, float y0) { x = x0; y = y0;}
	PointXY(PointXYZ P) { x = P.x; y = P.y;}
	
	// array type accessor
	float operator [] (int i) const
	{
		if (i == 0) return x;
		else return y;
	}

	// opperations
	float dot(const PointXY P) const
	{
		return x * P.x + y * P.y;
	}

	float sq_norm() const
	{
		return x*x + y*y;
	}

	float cross(const PointXY P) const
	{
		return x*P.y - y*P.x;
	}	

	PointXY& operator+=(const PointXY& P)
	{
		x += P.x;
		y += P.y;
		return *this;
	}

	PointXY& operator-=(const PointXY& P)
	{
		x -= P.x;
		y -= P.y;
		return *this;
	}

	PointXY& operator*=(const float& a)
	{
		x *= a;
		y *= a;
		return *this;
	}
};


// Point Opperations
// *****************

inline PointXY operator + (const PointXY A, const PointXY B)
{
	return PointXY(A.x + B.x, A.y + B.y);
}

inline PointXY operator - (const PointXY A, const PointXY B)
{
	return PointXY(A.x - B.x, A.y - B.y);
}

inline PointXY operator * (const PointXY P, const float a)
{
	return PointXY(P.x * a, P.y * a);
}

inline PointXY operator * (const float a, const PointXY P)
{
	return PointXY(P.x * a, P.y * a);
}

inline std::ostream& operator << (std::ostream& os, const PointXY P)
{
	return os << "[" << P.x << ", " << P.y << "]";
}

inline bool operator == (const PointXY A, const PointXY B)
{
	return A.x == B.x && A.y == B.y;
}

inline PointXY floor(const PointXY P)
{
	return PointXY(std::floor(P.x), std::floor(P.y));
}



//------------------------------------------------------------------------------------------------------------
// Pointcloud class
// ****************
//
//------------------------------------------------------------------------------------------------------------

struct PointCloud
{

	std::vector<PointXYZ>  pts;

	// Must return the number of data points
	inline size_t kdtree_get_point_count() const { return pts.size(); }

	// Returns the dim'th component of the idx'th point in the class:
	// Since this is inlined and the "dim" argument is typically an immediate value, the
	//  "if/else's" are actually solved at compile time.
	inline float kdtree_get_pt(const size_t idx, const size_t dim) const
	{
		if (dim == 0) return pts[idx].x;
		else if (dim == 1) return pts[idx].y;
		else return pts[idx].z;
	}

	// Optional bounding-box computation: return false to default to a standard bbox computation loop.
	//   Return true if the BBOX was already computed by the class and returned in "bb" so it can be avoided to redo it again.
	//   Look at bb.size() to find out the expected dimensionality (e.g. 2 or 3 for point clouds)
	template <class BBOX>
	bool kdtree_get_bbox(BBOX& /* bb */) const { return false; }

};


// Utility function for pointclouds
void filter_pointcloud(std::vector<PointXYZ>& pts, std::vector<float>& scores, float filter_value);
void filter_floatvector(std::vector<float>& vec, float filter_value);

template <class T>
void filter_anyvector(std::vector<T>& vec, std::vector<float>& scores, float filter_value)
{
	// Remove every element whose score is < filter_value
	auto vec_address = vec.data();
	vec.erase(std::remove_if(vec.begin(), vec.end(),
		[&scores, vec_address, filter_value](const T& f) { return scores[(size_t)(&f - vec_address)] < filter_value; }),
		vec.end());
}


// PLY reading/saving functions
void save_cloud(std::string dataPath, std::vector<PointXYZ>& points, std::vector<PointXYZ>& normals, std::vector<float>& features);
void save_cloud(std::string dataPath, std::vector<PointXYZ>& points, std::vector<float>& features);
void save_cloud(std::string dataPath, std::vector<PointXYZ>& points, std::vector<PointXYZ>& normals);
void save_cloud(std::string dataPath, std::vector<PointXYZ>& points);

void load_cloud(std::string& dataPath,
	std::vector<PointXYZ>& points);

void load_cloud(std::string& dataPath,
	std::vector<PointXYZ>& points,
	std::vector<float>& float_scalar,
	std::string& float_scalar_name,
	std::vector<int>& int_scalar,
	std::string& int_scalar_name);
	
void load_cloud_normals(std::string& dataPath, 
	std::vector<PointXYZ>& points, 
	std::vector<PointXYZ>& normals, 
	std::vector<float>& float_scalar, 
	std::string& float_scalar_name,
	std::vector<int>& int_scalar,
	std::string& int_scalar_name);



//------------------------------------------------------------------------------------------------------------
// Plane3D class
// *************
//
//------------------------------------------------------------------------------------------------------------

class Plane3D
{
public:

	// Elements
	// ********

	// The plane is define by the equation a*x + b*y + c*z = d. The values (a, b, c) are stored in a PointXYZ called u.
	PointXYZ u;
	float d;


	// Methods
	// *******

	// Constructor
	Plane3D() { u.x = 1; u.y = 0; u.z = 0; d = 0; }
	Plane3D(const float a0, const float b0, const float c0, const float d0) { u.x = a0; u.y = b0; u.z = c0; d = d0; }
	Plane3D(const PointXYZ P0, const PointXYZ N0)
	{
		// Init with point and normal
		u = N0;
		d = N0.dot(P0);
	}
	Plane3D(const PointXYZ A, const PointXYZ B, const PointXYZ C)
	{
		// Init with three points
		u = (B - A).cross(C - A);
		d = u.dot(A);
	}

	// Method getting distance to one point
	float point_distance(const PointXYZ P)
	{
		return std::abs((u.dot(P) - d) / std::sqrt(u.sq_norm()));
	}

	// Method getting square distance to one point
	float point_sq_dist(const PointXYZ P)
	{
		float tmp = u.dot(P) - d;
		return tmp * tmp / u.sq_norm();
	}

	// Method for reversing the normal of the Plane
	void reverse()
	{
		u *= -1;
		d *= -1;
	}


	// Method getting distances to some points
	void point_distances_signed(std::vector<PointXYZ>& points, std::vector<float>& distances)
	{
		if (distances.size() != points.size())
			distances = std::vector<float>(points.size());
		size_t i = 0;
		float inv_norm_u = 1 / std::sqrt(u.sq_norm());
		for (auto& p : points)
		{
			distances[i] = (u.dot(p) - d) * inv_norm_u;
			i++;
		}
	}

	// Method getting distances to some points
	void point_distances(std::vector<PointXYZ>& points, std::vector<float>& distances)
	{
		if (distances.size() != points.size())
			distances = std::vector<float>(points.size());
		size_t i = 0;
		float inv_norm_u = 1 / std::sqrt(u.sq_norm());
		for (auto& p : points)
		{
			distances[i] = std::abs((u.dot(p) - d) * inv_norm_u);
			i++;
		}
	}
	
	int in_range(std::vector<PointXYZ>& points, float threshold)
	{
		int count = 0;
		float inv_norm_u = 1 / std::sqrt(u.sq_norm());
		for (auto& p : points)
		{
			if (std::abs((u.dot(p) - d) * inv_norm_u) < threshold)
				count++;
		}
		return count;
	}
};


//-------------------------------------------------------------------------------------------
//
// VoxKey
// ******
//
//	Here we define a struct that will be used as key in our hash map. It contains 3 integers.
//  Then we specialize the std::hash function for this class.
//
//-------------------------------------------------------------------------------------------

class VoxKey
{
public:
	int x;
	int y;
	int z;

	VoxKey()
	{
		x = 0;
		y = 0;
		z = 0;
	}
	VoxKey(int x0, int y0, int z0)
	{
		x = x0;
		y = y0;
		z = z0;
	}

	bool operator==(const VoxKey &other) const
	{
		return (x == other.x && y == other.y && z == other.z);
	}

	int& operator[](std::size_t idx)       
	{
		if (idx == 2) return z;
		else if (idx == 1) return y;
		else  return x;
	}
    const int& operator[](std::size_t idx) const
	{
		if (idx == 2) return z;
		else if (idx == 1) return y;
		else return x;
	}

	void update_min(const VoxKey &k0)
	{
		if (k0.x < x)
			x = k0.x;
		if (k0.y < y)
			y = k0.y;
		if (k0.z < z)
			z = k0.z;
	}

	void update_max(const VoxKey &k0)
	{
		if (k0.x > x)
			x = k0.x;
		if (k0.y > y)
			y = k0.y;
		if (k0.z > z)
			z = k0.z;
	}
};

inline VoxKey operator+(const VoxKey A, const VoxKey B)
{
	return VoxKey(A.x + B.x, A.y + B.y, A.z + B.z);
}

inline bool operator<(const VoxKey A, const VoxKey B)
{
	if (A.x == B.x)
	{
		if (A.y == B.y)
			return (A.z < B.z);
		else
			return (A.y < B.y);
	}
	else
		return (A.x < B.x);
}

inline VoxKey operator-(const VoxKey A, const VoxKey B)
{
	return VoxKey(A.x - B.x, A.y - B.y, A.z - B.z);
}

// Simple utility function to combine hashtables
template <typename T, typename... Rest>
void hash_combine(std::size_t &seed, const T &v, const Rest &...rest)
{
	seed ^= std::hash<T>{}(v) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
	(hash_combine(seed, rest), ...);
}

// Specialization of std:hash function
namespace std
{
	template <>
	struct hash<VoxKey>
	{
		std::size_t operator()(const VoxKey &k) const
		{
			std::size_t ret = 0;
			hash_combine(ret, k.x, k.y, k.z);
			return ret;
		}
	};
}


//-------------------------------------------------------------------------------------------
//
// PixKey
// ******
//
//	Same as VoxKey but in 2D
//
//-------------------------------------------------------------------------------------------

class PixKey
{
public:
	int x;
	int y;

	PixKey()
	{
		x = 0;
		y = 0;
	}
	PixKey(int x0, int y0)
	{
		x = x0;
		y = y0;
	}

	bool operator==(const PixKey &other) const
	{
		return (x == other.x && y == other.y);
	}
};

inline PixKey operator+(const PixKey A, const PixKey B)
{
	return PixKey(A.x + B.x, A.y + B.y);
}

inline PixKey operator-(const PixKey A, const PixKey B)
{
	return PixKey(A.x - B.x, A.y - B.y);
}

// Specialization of std:hash function
namespace std
{
	template <>
	struct hash<PixKey>
	{
		std::size_t operator()(const PixKey &k) const
		{
			std::size_t ret = 0;
			hash_combine(ret, k.x, k.y);
			return ret;
		}
	};
} // namespace std


void random_3_pick(int &A_i, int &B_i, int &C_i,
				   std::uniform_int_distribution<int> &distribution,
				   std::default_random_engine &generator);

bool is_triplet_bad(PointXYZ &A, PointXYZ &B, PointXYZ &C, PointXYZ &u);

Plane3D plane_ransac(std::vector<PointXYZ> &points,
					 float max_dist = 0.1,
					 int max_steps = 100);
					 
Plane3D frame_ground_ransac(std::vector<PointXYZ> &points,
							std::vector<PointXYZ> &normals,
							float vertical_thresh_deg = 20.0,
							float max_dist = 0.1);

bool rot_u_to_v(PointXYZ u, PointXYZ v, Eigen::Matrix3d &R);

