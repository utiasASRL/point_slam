#pragma once

#include <cstdint>
#include <cstdio>
#include <ctime>
#include <random>
#include <unordered_set>
#include <numeric>
#include <chrono>
#include <thread>

#include "grid_subsampling/grid_subsampling.h"
#include "polar_processing/polar_processing.h"
#include "pointmap/pointmap.h"
#include "icp/icp.h"

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/GetMap.h>

#include "tf/transform_listener.h"
#include "tf/transform_broadcaster.h"
#include "tf/transform_datatypes.h"
#include "tf_conversions/tf_eigen.h"
#include "eigen_conversions/eigen_msg.h"

// #include <costmap_converter_msgs/msg/obstacle_msg.h>
#include <costmap_converter/ObstacleArrayMsg.h>
#include <vox_msgs/VoxGrid.h>

using namespace std;


// KDTree type definition
typedef nanoflann::KDTreeSingleIndexAdaptor<nanoflann::L2_Simple_Adaptor<float, PointCloud>, PointCloud, 3> PointXYZ_KDTree;
typedef Eigen::Matrix<double, 6, 6> Matrix6d;
typedef Eigen::Matrix<double, 6, 1> Vector6d;

void dummy_callback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
void dummy_callback_2(const sensor_msgs::PointCloud2::ConstPtr& msg);
void dummy_callback_3(const vox_msgs::VoxGrid::ConstPtr& msg);
void dummy_callback_4(const costmap_converter::ObstacleArrayMsg::ConstPtr& msg);


// Utilities
// *********

Eigen::Matrix4d transformListenerToEigenMatrix(const tf::TransformListener& listener, const string& target, const string& source, const ros::Time& stamp);
Eigen::Matrix4d odomMsgToEigenMatrix(const nav_msgs::Odometry& odom);
nav_msgs::Odometry eigenMatrixToOdomMsg(const Eigen::Matrix4d& inTr, const string& frame_id, const ros::Time& stamp);

// SLAM params class
// *****************

class SLAM_params
{
public:
	// Elements
	// ********

	// Number of lines of scan in the lidar
	int lidar_n_lines;

	// Size of the map voxels
	float map_voxel_size;

	// Size of the voxels for frame subsampling
	float frame_voxel_size;

	// max distance travelled before frames are removed from local map
	float local_map_dist;

	// Account for motion distortion (fasle in the case of simulated data)
	bool motion_distortion;

	// normal computation parameters
	vector<float> polar_r2s;
	float polar_r;
	float min_theta_radius;
	float theta_radius_ratio;

	// Counting processed frames
	int frame_i;

	// Number of threads used
	int n_threads;


	// Transformation matrix from velodyne frame to base frame
	Eigen::Matrix4d H_velo_base;

	// Params of ICP used in this SLAM
	ICP_params icp_params;

	// Params of frame normal computation
	float h_scale;
	float r_scale;
	int outl_rjct_passes;
	float outl_rjct_thresh;

	// ROS related
	string odom_frame;
	string map_frame;
	string base_frame;
	bool filtering, gt_filter;
	vector<int> loc_labels;
	int verbose;
	float map2d_pixel_size;
	int map2d_max_count;
	float map2d_zMin, map2d_zMax;
	string log_path;

	// Methods
	// *******

	// Constructor
	SLAM_params() : loc_labels(7)
	{
		lidar_n_lines = 32;
		min_theta_radius = 0.025;
		theta_radius_ratio = 1.5;
		map_voxel_size = 0.08;
		frame_voxel_size = 0.16;
		local_map_dist = 10.0;
		motion_distortion = false;
		H_velo_base = Eigen::Matrix4d::Identity(4, 4);

		h_scale = 0.3;
		r_scale = 10.0;
		outl_rjct_passes = 2;
		outl_rjct_thresh = 0.003;

		n_threads = 1;

		odom_frame = "odom";
		map_frame = "map";
		base_frame = "base_link";
		filtering = false;
		gt_filter = true;
		std::iota(loc_labels.begin(), loc_labels.end(), 0);
		map2d_pixel_size = 0.08;
		map2d_max_count = 10;
		map2d_zMin = 0.5;
		map2d_zMax = 1.5;
		verbose = 0;
		log_path = "";
	}
};

// SLAM class
// **********

class PointMapSLAM
{
public:
	// Elements
	// ********

	// Parameters
	SLAM_params params;

	// Map used by the algorithm
	PointMap map;

	// Pose of the last mapped frame
	Eigen::Matrix4d last_H;
	vector<Eigen::Matrix4d> all_H;
	vector<ros::Time> f_times;

	// Current pose correction from odometry to map
	Eigen::Matrix4d H_OdomToMap;

	// Current number of aligned frames
	int n_frames;
	float t_min, t_max;

	// ROS parameters
	ros::Time latest_stamp;
	tf::TransformListener tfListener;
	tf::TransformBroadcaster tfBroadcaster;
	ros::Publisher sst;
	ros::Publisher sstm;
	OccupGrid2D map2D;

	// Methods
	// *******

	// Constructor
	PointMapSLAM(SLAM_params slam_params0,
				 vector<PointXYZ>& init_points,
				 vector<PointXYZ>& init_normals,
				 vector<float>& init_scores) :
		map(slam_params0.map_voxel_size, init_points, init_normals, init_scores),
		map2D(slam_params0.map2d_pixel_size, slam_params0.map2d_max_count),
		tfListener(ros::Duration(30))
	{
		// Init paramters
		params = slam_params0;
		n_frames = 0;

		// Dummy first last_H
		last_H = Eigen::Matrix4d::Identity(4, 4);
		H_OdomToMap = Eigen::Matrix4d::Identity(4, 4);
		latest_stamp = ros::Time::now();

		// Optionally init map2D from map3D
		if (init_points.size() > 0)
		{
			Plane3D ground_P = frame_ground_ransac(init_points, init_normals, 20.0, 0.1);
			map2D.init_from_3D_map(init_points, ground_P, slam_params0.map2d_zMin, slam_params0.map2d_zMax);
		}
	}

	// Mapping methods
	void init_map() { return; }
	void gotClassifCloud(const sensor_msgs::PointCloud2::ConstPtr& msg);
	void gotVeloCloud(const sensor_msgs::PointCloud2::ConstPtr& msg);
	void processCloud(const sensor_msgs::PointCloud2::ConstPtr& msg, bool filtering, bool update_map_2D = true);
	void processClassifCloud(const sensor_msgs::PointCloud2::ConstPtr& msg, bool filtering, bool update_map_2D = true);
	void publish_2D_map();

	// Debug method
	void save_trajectory(string& path)
	{
		// Name of the file
		string filepath = path + "map_traj.ply";

		// Variables
		uint64_t num_poses = all_H.size();

		// Convert all poses in points and quaternions
		vector<double> times;
		vector<Eigen::Vector3d> trans;
		vector<Eigen::Quaternion<double>> rots;
		times.reserve(num_poses);
		trans.reserve(num_poses);
		rots.reserve(num_poses);
		for (auto& H : all_H)
		{
			trans.push_back(H.block(0, 3, 3, 1));
			Eigen::Matrix3d R = H.block(0, 0, 3, 3);
			rots.push_back(Eigen::Quaternion<double>(R));
		}
		for (auto& t : f_times)
			times.push_back(t.toSec());

		// Open file
		npm::PLYFileOut file(filepath);

		// Push fields
		file.pushField(num_poses, 1, npm::PLY_DOUBLE, {"time"}, times);
		file.pushField(num_poses, 3, npm::PLY_DOUBLE, {"pos_x", "pos_y", "pos_z"}, trans);
		file.pushField(num_poses, 4, npm::PLY_DOUBLE, {"rot_x", "rot_y", "rot_z", "rot_w"}, rots);
		file.write();
	}
};

// Function declaration
// ********************

void preprocess_frame(vector<PointXYZ> &f_pts,
					  vector<float> &f_ts,
					  vector<ushort> &f_rings,
					  vector<PointXYZ> &sub_pts,
					  vector<PointXYZ> &normals,
					  vector<float> &norm_scores,
					  vector<double> &icp_scores,
					  vector<size_t> &sub_inds,
					  Plane3D &frame_ground,
					  vector<float> &heights,
					  SLAM_params &params,
					  vector<clock_t> &t);

// Main
// ****

int main(int argc, char **argv);