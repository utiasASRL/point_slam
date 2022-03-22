#pragma once

#include <set>
#include <cstdint>
#include <cmath>
#include <algorithm>
#include <unordered_set>
#include <limits>

#include "../grid_subsampling/grid_subsampling.h"
#include "../polar_processing/polar_processing.h"

using namespace std;

// KDTree type definition
typedef nanoflann::KDTreeSingleIndexAdaptorParams KDTree_Params;
typedef nanoflann::KDTreeSingleIndexAdaptor<nanoflann::L2_Simple_Adaptor<float, PointCloud>, PointCloud, 3> PointXYZ_KDTree;
typedef nanoflann::KDTreeSingleIndexDynamicAdaptor<nanoflann::L2_Simple_Adaptor<float, PointCloud>, PointCloud, 3> PointXYZ_Dynamic_KDTree;

Eigen::Matrix4d pose_interp(float t, Eigen::Matrix4d const &H1, Eigen::Matrix4d const &H2, int verbose);

//-------------------------------------------------------------------------------------------
//
// PointMapPython Class
// ********************
//
//	PointMap designed to be used in python. As it is hard to transfert unordered map to
//	python dict structure, we rebuild the hashmap every update (not very efficient).
//
//-------------------------------------------------------------------------------------------

class MapVoxelData
{
public:
	// Elements
	// ********

	bool occupied;
	int count;
	PointXYZ centroid;
	PointXYZ normal;
	float score;

	// Methods
	// *******

	// Constructor
	MapVoxelData()
	{
		occupied = false;
		count = 0;
		score = -1.0f;
		centroid = PointXYZ();
		normal = PointXYZ();
	}
	MapVoxelData(const PointXYZ p0, const PointXYZ n0, const float s0, const int c0)
	{
		occupied = true;
		count = c0;
		score = s0;
		centroid = p0;
		normal = n0;
	}

	MapVoxelData(const PointXYZ p0)
	{
		// We initiate only the centroid
		count = 1;
		centroid = p0;

		// Other varaible are kept null
		occupied = false;
		score = -1.0f;
		normal = PointXYZ();
	}

	void update_centroid(const PointXYZ p0)
	{
		count += 1;
		centroid += p0;
	}

	void update_normal(const float s0, const PointXYZ n0)
	{
		// We keep the worst normal
		occupied = true;

		// Rule for normal update:
		// IF current_score=2 : normal was computed with planarity in the map, do not modify
		// IF s0 < score - 0.1 : Too bad score dont update (This includes the condition above)
		// IF s0 > score + 0.1 : Better score, use new normal
		// IF abs(s0 - score) < 0.1 : Similar score, avergae normal
		// When averaging be careful of orientation. Dont worry about norm, we renormalize every normal in the end

		if (s0 > score + 0.1)
		{
			score = s0;
			normal = n0;
		}
		else if (s0 > score - 0.1)
		{
			if (s0 > score)
				score = s0;
			if (normal.dot(n0) > 0)
				normal += n0;
			else
				normal -= n0;
		}
	}
};

class PointMapPython
{
public:
	// Elements
	// ********

	float dl;
	vector<PointXYZ> points;
	vector<PointXYZ> normals;
	vector<float> scores;
	vector<int> counts;

	// Methods
	// *******

	// Constructor
	PointMapPython()
	{
		dl = 1.0f;
	}
	PointMapPython(const float dl0)
	{
		dl = dl0;
	}

	// Methods
	void update(vector<PointXYZ> &points0, vector<PointXYZ> &normals0, vector<float> &scores0);

	void init_samples(const PointXYZ originCorner,
					  const PointXYZ maxCorner,
					  unordered_map<size_t, MapVoxelData> &samples);

	void add_samples(const vector<PointXYZ> &points0,
					 const vector<PointXYZ> &normals0,
					 const vector<float> &scores0,
					 const PointXYZ originCorner,
					 const PointXYZ maxCorner,
					 unordered_map<size_t, MapVoxelData> &samples);

	size_t size() { return points.size(); }
};


//-------------------------------------------------------------------------------------------
//
// PointMap Class
// **************
//
//	PointMap designed to be used in C++. Everything should be more efficient here.
//
//-------------------------------------------------------------------------------------------

class PointMap
{
public:
	// Elements
	// ********

	// Voxel size
	float dl;

	// Count the number of frames used tu update this map
	int update_idx;

	// Map limits
	VoxKey minVox;
	VoxKey maxVox;

	// Containers for the data
	PointCloud cloud;
	vector<PointXYZ> normals;
	vector<float> scores;
	vector<int> counts;
	vector<int> latest;

	// Sparse hashmap that contain voxels (each voxel data is in the contiguous vector containers)
	unordered_map<VoxKey, size_t> samples;

	// KDTree for neighbors query
	PointXYZ_Dynamic_KDTree tree;

	// Methods
	// *******

	// Constructor
	PointMap() : tree(3, cloud, KDTree_Params(10 /* max leaf */))
	{
		dl = 1.0f;
		update_idx = 0;
	}
	PointMap(const float dl0) : tree(3, cloud, KDTree_Params(10 /* max leaf */))
	{
		dl = dl0;
		update_idx = 0;
	}
	PointMap(const float dl0,
			 vector<PointXYZ> &init_points,
			 vector<PointXYZ> &init_normals,
			 vector<float> &init_scores) : tree(3, cloud, KDTree_Params(10 /* max leaf */))
	{
		// Set voxel size
		dl = dl0;

		// Init limits
		maxVox.x = numeric_limits<int>::min();
		maxVox.y = numeric_limits<int>::min();
		maxVox.z = numeric_limits<int>::min();
		minVox.x = numeric_limits<int>::max();
		minVox.y = numeric_limits<int>::max();
		minVox.z = numeric_limits<int>::max();

		// Optionally init map
		update_idx = 0;
		if (init_points.size() > 0)
		{
			update_idx = -1;
			update(init_points, init_normals, init_scores, -1);
		}
	}
	
	PointMap(const PointMap &map0) : tree(3, cloud, KDTree_Params(10 /* max leaf */))
	{
		dl = map0.dl;
		update_idx = map0.update_idx;
		cloud = map0.cloud;
		normals = map0.normals;
		scores = map0.scores;
		counts = map0.counts;
		latest = map0.latest;
		samples = map0.samples;
		tree.addPoints(0, cloud.pts.size() - 1);
	}

	PointMap& operator=(const PointMap &map0)
	{
		dl = map0.dl;
		update_idx = map0.update_idx;
		cloud = map0.cloud;
		normals = map0.normals;
		scores = map0.scores;
		counts = map0.counts;
		latest = map0.latest;
		samples = map0.samples;
		tree.addPoints(0, cloud.pts.size() - 1);
      	return *this;
	}

	// Size of the map (number of point/voxel in the map)
	size_t size() { return cloud.pts.size(); }

	// Init of voxel centroid
	void init_sample(const VoxKey &k, const PointXYZ &p0, const PointXYZ &n0, const float &s0, const int &c0)
	{
		// We place anew key in the hashmap
		samples.emplace(k, cloud.pts.size());

		// We add new voxel data but initiate only the centroid
		cloud.pts.push_back(p0);
		normals.push_back(n0);
		scores.push_back(s0);

		// Count is useless, instead save index of first frame placing a point in this cell
		counts.push_back(c0);
		latest.push_back(c0);
	}

	// Init of voxel centroid
	void reinit_sample(const VoxKey &k, const PointXYZ &p0, const PointXYZ &n0, const float &s0, const int &c0)
	{
		// We place anew key in the hashmap
		samples[k], cloud.pts.size();

		// We add new voxel data but initiate only the centroid
		cloud.pts.push_back(p0);
		normals.push_back(n0);
		scores.push_back(s0);

		// Count is useless, instead save index of first frame placing a point in this cell
		counts.push_back(c0);
		latest.push_back(c0);
	}

	// Update of voxel centroid
	void update_sample(const size_t idx, const PointXYZ &p0, const PointXYZ &n0, const float &s0, const int &c0)
	{
		// latest frame idx
		latest[idx] = c0;

		// Update normal if we have a clear view of it  and closer distance (see computation of score)
		if (s0 > scores[idx])
		{
			scores[idx] = s0;
			normals[idx] = n0;
		}
	}

	void update_limits(const VoxKey &k)
	{
		if (k.x < minVox.x)
			minVox.x = k.x;
		if (k.y < minVox.y)
			minVox.y = k.y;
		if (k.z < minVox.z)
			minVox.z = k.z;

		if (k.x > maxVox.x)
			maxVox.x = k.x;
		if (k.y > maxVox.y)
			maxVox.y = k.y;
		if (k.z > maxVox.z)
			maxVox.z = k.z;
	}

	// Update map with a set of new points
	void update(vector<PointXYZ> &points0, vector<PointXYZ> &normals0, vector<float> &scores0, int ind0)
	{

		// Reserve new space if needed
		if (samples.size() < 1)
			samples.reserve(10 * points0.size());
		if (cloud.pts.capacity() < cloud.pts.size() + points0.size())
		{
			cloud.pts.reserve(cloud.pts.capacity() + points0.size());
			counts.reserve(counts.capacity() + points0.size());
			latest.reserve(latest.capacity() + points0.size());
			normals.reserve(normals.capacity() + points0.size());
			scores.reserve(scores.capacity() + points0.size());
		}

		//std::cout << std::endl << "--------------------------------------" << std::endl;
		//std::cout << "current max_load_factor: " << samples.max_load_factor() << std::endl;
		//std::cout << "current size: " << samples.size() << std::endl;
		//std::cout << "current bucket_count: " << samples.bucket_count() << std::endl;
		//std::cout << "current load_factor: " << samples.load_factor() << std::endl;
		//std::cout << "--------------------------------------" << std::endl << std::endl;

		// Initialize variables
		float inv_dl = 1 / dl;
		size_t i = 0;
		VoxKey k0;
		size_t num_added = 0;

		for (auto &p : points0)
		{
			// Position of point in sample map
			PointXYZ p_pos = p * inv_dl;

			// Corresponding key
			k0.x = (int)floor(p_pos.x);
			k0.y = (int)floor(p_pos.y);
			k0.z = (int)floor(p_pos.z);

			// Update the point count
			if (samples.count(k0) < 1)
			{
				init_sample(k0, p, normals0[i], scores0[i], ind0);
				num_added++;
			}
			else
			{
				// Case of previously deleted points
				size_t idx = samples[k0];
				if (tree.isRemoved(idx))
				{
					// We want to add previously deleted points, we have to recreate it from scratch
					reinit_sample(k0, p, normals0[i], scores0[i], ind0);
					num_added++;
				}
				else
					update_sample(idx, p, normals0[i], scores0[i], ind0);
			}
			i++;
		}

		// Update tree
		tree.addPoints(cloud.pts.size() - num_added, cloud.pts.size() - 1);

		// Update frame count
		update_idx++;
	}

	
	// Remove old indices from tree
	int remove_old(int min_i, int last_min_i)
	{
		int removed_count = 0;
		int i = 0;
		for (auto &latest_i: latest)
		{
			if (latest_i < min_i && latest_i >= last_min_i)
			{
				tree.removePoint(i);
				removed_count++;
			}
			i++;
		}
		return removed_count;
	}


	// Debug method that saves the map as ply file
	void debug_save_ply(string &path, int idx)
	{
		cout << endl
			 << "---------------------------------------------" << endl;
		char buffer[200];
		sprintf(buffer, "pointmap_%05d.ply", (int)idx);
		string filepath = path + string(buffer);
		cout << filepath << endl;
		save_cloud(filepath, cloud.pts, normals, scores);
		cout << "---------------------------------------------" << endl
			 << endl;
	}
};

class OccupGrid2D
{
public:
	// Elements
	// ********

	// Voxel size
	float dl;

	// Maximum value of the counts
	int max_count;

	// Map limits
	PixKey minPix;
	PixKey maxPix;

	vector<float> scores;
	vector<int> counts;
	vector<PointXYZ> points;

	// Sparse hashmap that contain voxels (each voxel data is in the contiguous vector containers)
	unordered_map<PixKey, size_t> samples;

	// Methods
	// *******

	// Constructor
	OccupGrid2D()
	{
		dl = 1.0f;
		max_count = 10;
	}
	OccupGrid2D(const float dl0, const int m0)
	{
		dl = dl0;
		max_count = m0;
	}

	// Size of the map (number of point/pixel in the map)
	size_t size() { return points.size(); }

	// Init of pixel centroid
	void init_sample(const PixKey &k, const PointXYZ &p0, const float &s0)
	{
		// We place anew key in the hashmap
		samples.emplace(k, points.size());

		// We add new voxel data but initiate only the centroid
		points.push_back(p0);
		counts.push_back(1);
		scores.push_back(s0);
	}

	// Update of voxel centroid
	void update_sample(const size_t idx, const float &s0)
	{
		// Update only count for optional removal count of points and centroid of the cell
		if (counts[idx] < max_count)
			scores[idx] += (s0 - scores[idx]) / ++counts[idx];
		else
			scores[idx] += (s0 - scores[idx]) / max_count;
	}

	void update_height(const size_t idx, const float &h0)
	{
		points[idx].z += (h0 - points[idx].z) / 4;
	}

	void update_limits(const PixKey &k)
	{
		if (k.x < minPix.x)
			minPix.x = k.x;
		if (k.y < minPix.y)
			minPix.y = k.y;

		if (k.x > maxPix.x)
			maxPix.x = k.x;
		if (k.y > maxPix.y)
			maxPix.y = k.y;
	}

	// Update map with a set of new points
	void init_from_3D_map(vector<PointXYZ> &points3D, Plane3D &ground_P, float zMin, float zMax)
	{
		////////////////
		// Init steps //
		////////////////

		// Initialize variables
		float inv_dl = 1 / dl;
		PixKey k0;

		//////////////////////////
		// Convert to 2D ranges //
		//////////////////////////

		// Get distances to ground
		vector<float> distances;
		ground_P.point_distances(points3D, distances);

		////////////////////////
		// Update full pixels //
		////////////////////////

		// Loop over 3D points
		size_t p_i = 0;
		for (auto &p : points3D)
		{
			// Check height limits
			if (distances[p_i] < zMin*3 || distances[p_i] > zMax)
			{
				p_i++;
				continue;
			}

			// Corresponding key
			k0.x = (int)floor(p.x * inv_dl);
			k0.y = (int)floor(p.y * inv_dl);

			// Update the point count
			if (samples.count(k0) < 1)
			{
				// Create a new sample at this location
				init_sample(k0, PointXYZ(((float)k0.x + 0.5) * dl, ((float)k0.y + 0.5) * dl, p.z), 1.0);

				// Update grid limits
				update_limits(k0);
			}

			p_i++;
		}
	}

	// Update map with a set of new points
	void update_from_3D(vector<PointXYZ> &points3D, vector<int> &rings, size_t n_rings, PointXYZ &center3D, float zMin, float zMax, float min_range)
	{
		////////////////
		// Init steps //
		////////////////

		// std::cout << "------------------------------------" << std::endl;
		// std::cout << zMin << std::endl;
		// std::cout << zMax << std::endl;

		// std::cout << min_point(points3D).z << std::endl;
		// std::cout << max_point(points3D).z << std::endl;

		// std::cout << center3D.z << std::endl;
		// std::cout << center3D.z << std::endl;
		// std::cout << "********" << std::endl;

		// TODO: Every once and a while delete the pixels that have a low score (<0.1).
		// 		 Just create again the samples, vectors etc.
		//

		// Reserve new space if needed
		if (points.size() < 1)
		{
			samples.reserve(points3D.size());
			points.reserve(points3D.size());
			counts.reserve(points3D.size());
			scores.reserve(points3D.size());
		}

		// Initialize variables
		float inv_dl = 1 / dl;
		PixKey k0;
		float min_range2 = min_range * min_range;

		// Every pixel can be updated only once.
		vector<bool> not_updated(points.size(), true);

		//////////////////////////
		// Convert to 2D ranges //
		//////////////////////////

		// Init free range table (1D grid containing range for each angle)
		float angle_res = 0.5 * M_PI / 180.0;
		float inv_angle_res = 1 / angle_res;
		size_t n_angles = (size_t)floor(2.0 * M_PI / angle_res) + 1;
		vector<float> range_table(n_angles, -1.0);
		
		// map theta to ring index
		float theta_res = 0.33 * M_PI / 180.0;
		float inv_theta_res = 1 / theta_res;
		size_t n_thetas = (size_t)floor( M_PI / theta_res) + 1;
		vector<float> range_low_table(n_angles * n_rings, -1.0);
		vector<int> theta_to_ring(n_thetas, -1);

		// // Get distances to ground
		// vector<float> heights;
		// ground_P.point_distances(points3D, heights);

		////////////////////////
		// Update full pixels //
		////////////////////////

		// Loop over 3D points
		float pi_s_2 = M_PI / 2;
		size_t p_i = 0;
		for (auto &p : points3D)
		{
			
			//	Check min range
			PointXYZ diff3D(p - center3D);
			float d2 = diff3D.sq_norm();
			if (d2 > min_range2)
			{
				// Check height limits
				if (p.z > zMin && p.z < zMax)
				{
					// Corresponding key
					k0.x = (int)floor(p.x * inv_dl);
					k0.y = (int)floor(p.y * inv_dl);

					// Update the point count
					if (samples.count(k0) < 1)
					{
						// Create a new sample at this location
						init_sample(k0, PointXYZ(((float)k0.x + 0.5) * dl, ((float)k0.y + 0.5) * dl, p.z), 0.9);

						// Update grid limits
						update_limits(k0);
					}
					else
					{
						size_t i0 = samples[k0];
						if (i0 < not_updated.size() && not_updated[i0])
						{
							update_sample(i0, 1.0);
							update_height(i0, p.z);
							not_updated[i0] = false;
						}
					}

					// Add the angle and its corresponding free_range
					PointXY diff2D(diff3D);
					size_t angle_idx = (size_t)floor((atan2(diff2D.y, diff2D.x) + M_PI) * inv_angle_res);
					if (range_table[angle_idx] < 0 || d2 < pow(range_table[angle_idx], 2))
						range_table[angle_idx] = sqrt(d2);
				}
				else
				{
					// Save as low heigh free range
					// Add the angle and its corresponding free_range
					float tmp1 = diff3D.x * diff3D.x + diff3D.y * diff3D.y;
					float phi = atan2(diff3D.y, diff3D.x); // azimuth angle
					float theta = atan2(diff3D.z, sqrt(tmp1));
					size_t phi_idx = (size_t)floor((phi + M_PI) * inv_angle_res);
					int theta_idx = (int)floor((theta + pi_s_2) * inv_theta_res);

					if (theta_to_ring[theta_idx] < 0)
					{
						// Update ring theta
						theta_to_ring[theta_idx] = rings[p_i];
					}

					// Get index in the low range table
					size_t angle_idx = phi_idx + n_angles * rings[p_i];
					if (range_low_table[angle_idx] < 0 || d2 < range_low_table[angle_idx])
						range_low_table[angle_idx] = d2;
				}
			}

			p_i++;
		}

		// Make sure that if an obstacle is above a ray, it gets deleted like if it was on the ray
		// By setting the -1 theta_to_ring to the next lowest ring
		int last_ring_i = -1;
		for (int i = 0; i < (int)theta_to_ring.size(); i++)
		{
			if (theta_to_ring[i] >= 0)
				last_ring_i = theta_to_ring[i];
			else
				theta_to_ring[i] = last_ring_i;
		}

		///////////////////////////////
		// Interpolate the 2D ranges //
		///////////////////////////////

		// First find the last valid value
		int last_i, next_i;
		last_i = range_table.size() - 1;
		while (last_i >= 0)
		{
			if (range_table[last_i] > 0)
				break;
			last_i--;
		}

		// Interpolate
		next_i = 0;
		last_i -= range_table.size();
		while (next_i < range_table.size())
		{
			if (range_table[next_i] > 0)
			{
				if (last_i < 0)
				{
					int diff = next_i - last_i;
					if (diff > 1)
					{
						for (int i = last_i + 1; i < next_i; i++)
						{
							int real_i = i;
							if (real_i < 0)
								real_i += range_table.size();
							float t = (i - last_i) / diff;
							int real_last_i = last_i + range_table.size();
							range_table[real_i] = t * range_table[real_last_i] + (1 - t) * range_table[next_i];
						}
					}
				}
				else
				{
					int diff = next_i - last_i;
					if (diff > 1)
					{
						for (int i = last_i + 1; i < next_i; i++)
						{
							float t = (i - last_i) / diff;
							range_table[i] = t * range_table[last_i] + (1 - t) * range_table[next_i];
						}
					}
				}
				last_i = next_i;
			}
			next_i++;
		}

		////////////////////////
		// Update free pixels //
		////////////////////////

		// Apply margin to free ranges
		float margin = dl;
		for (auto &r : range_table)
			r -= margin;
		for (auto &r : range_low_table)
			r -= pow(0.05, 2);

		// Update free pixels
		float min_free_h = 0.3;
		p_i = 0;
		for (auto &p : points)
		{
			// Ignore points updated just now
			if (p_i >= not_updated.size())
				break;
			if (!not_updated[p_i])
			{
				p_i++;
				continue;
			}

			if (p.z > min_free_h)
			{
				// Compute angle and range
				PointXY diff2D(p - center3D);
				size_t angle_idx = (atan2(diff2D.y, diff2D.x) + M_PI) * inv_angle_res;
				float d2 = diff2D.sq_norm();

				// Update score
				if (d2 > min_range2 && d2 < pow(range_table[angle_idx], 2))
				{
					update_sample(p_i, 0.0);
				}
			}
			else
			{
				// Low height case
				PointXYZ diff3D(p - center3D);
				float tmp1 = diff3D.x * diff3D.x + diff3D.y * diff3D.y;
				float d2 = tmp1 + diff3D.z * diff3D.z;
				float phi = atan2(diff3D.y, diff3D.x); // azimuth angle
				float theta = atan2(diff3D.z, sqrt(tmp1));
				size_t phi_idx = (size_t)floor((phi + M_PI) * inv_angle_res);
				int theta_idx = (int)floor((theta + pi_s_2) * inv_theta_res);

				// Update ring theta
				int ring_i = theta_to_ring[theta_idx];
				if (ring_i >= 0)
				{
					
					// Get index in the low range table
					size_t angle_idx = phi_idx + n_angles * ring_i;

					// Update score
					if (d2 > min_range2 && d2 < range_low_table[angle_idx] )
					{
						update_sample(p_i, 0.0);
					}
				}

			}

			p_i++;
		}
	}

	// Debug method that saves the map as ply file
	void debug_save_ply(string &path, int idx)
	{
		vector<PointXYZ> points3D;
		points3D.reserve(points.size());
		for (auto &p : points)
			points3D.push_back(PointXYZ(p.x, p.y, 0));

		cout << endl << "---------------------------------------------" << endl;
		vector<float> features(scores);
		features.reserve(scores.size() * 2);
		for (auto &c : counts)
			features.push_back((float)c);

		char buffer[200];
		sprintf(buffer, "debug_map2D_%05d.ply", (int)idx);
		string filepath = path + string(buffer);
		cout << filepath << endl;
		save_cloud(filepath, points3D, features);
		cout << "---------------------------------------------" << endl << endl;
	}
};