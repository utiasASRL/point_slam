#include "frame_update.h"


//-----------------------------------------------------------------------------------------------------------------------------
// Temporary debug stuff
// *********************


void dummy_callback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
  ROS_INFO("I heard one pred visu message");
}

void dummy_callback_2(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
  ROS_INFO("I heard one pred cloud message");
}

void dummy_callback_3(const vox_msgs::VoxGrid::ConstPtr& msg)
{
  ROS_INFO("I heard one pred message");
}

void dummy_callback_4(const costmap_converter::ObstacleArrayMsg::ConstPtr& msg)
{
  ROS_INFO("I heard one obstacle message");
}


//-----------------------------------------------------------------------------------------------------------------------------
// Utilities
// *********


Eigen::Matrix4d transformListenerToEigenMatrix(const tf::TransformListener& listener, const string& target, const string& source, const ros::Time& stamp)
{
	tf::StampedTransform stampedTr;
	if (!listener.waitForTransform(target, source, stamp, ros::Duration(0.1)))
	{
		ROS_WARN_STREAM("Cannot get transformation from " << source << " to " << target);
		return Eigen::Matrix4d::Zero(4, 4);
	}
	else
	{
		listener.lookupTransform(target, source, stamp, stampedTr);
		Eigen::Affine3d eigenTr;
		tf::transformTFToEigen(stampedTr, eigenTr);
		return eigenTr.matrix();
	}
}

Eigen::Matrix4d odomMsgToEigenMatrix(const nav_msgs::Odometry& odom)
{
	Eigen::Affine3d eigenTr;
	tf::poseMsgToEigen(odom.pose.pose, eigenTr);
	return eigenTr.matrix();
}

nav_msgs::Odometry eigenMatrixToOdomMsg(const Eigen::Matrix4d& inTr, const string& frame_id, const ros::Time& stamp)
{
	nav_msgs::Odometry odom;
	odom.header.stamp = stamp;
	odom.header.frame_id = frame_id;

	// Fill pose
	const Eigen::Affine3d eigenTr(inTr);
	tf::poseEigenToMsg(eigenTr, odom.pose.pose);

	// Fill velocity, TODO: find proper computation from delta poses to twist
	//odom.child_frame_id = cloudMsgIn.header.frame_id;
	// odom.twist.covariance[0+0*6] = -1;
	// odom.twist.covariance[1+1*6] = -1;
	// odom.twist.covariance[2+2*6] = -1;
	// odom.twist.covariance[3+3*6] = -1;
	// odom.twist.covariance[4+4*6] = -1;
	// odom.twist.covariance[5+5*6] = -1;

	return odom;
}

tf::Transform eigenMatrixToTransform(const Eigen::Matrix4d& in_H)
{
	tf::Transform tfTr;
	const Eigen::Affine3d eigenTr(in_H);
	tf::transformEigenToTF(eigenTr, tfTr);
	return tfTr;
}

void PointMapSLAM::publish_2D_map()
{
	float score_threshold = 0.6;
	if (params.filtering)
		score_threshold = 0.95;
	score_threshold = 0.95;


	// Init meta-data
	nav_msgs::GetMap::Response map_message;
	map_message.map.info.width = map2D.maxPix.x - map2D.minPix.x + 1;
	map_message.map.info.height = map2D.maxPix.y - map2D.minPix.y + 1;
	map_message.map.info.origin.position.x = map2D.minPix.x * map2D.dl;
	map_message.map.info.origin.position.y = map2D.minPix.y * map2D.dl;
	map_message.map.info.origin.position.z = 0;

	map_message.map.info.origin.orientation.x = 0.0;
	map_message.map.info.origin.orientation.y = 0.0;
	map_message.map.info.origin.orientation.z = 0.0;
	map_message.map.info.origin.orientation.w = 1.0;
	map_message.map.info.resolution = map2D.dl;

	// Fill the ROS map object
	map_message.map.data = vector<int8_t>(map_message.map.info.width * map_message.map.info.height, 0);

	// Only consider point with height between 30cm and 1m30cm to avoid ground and still pass through doors
	for (auto& pix : map2D.samples)
	{
		size_t mapIdx = (size_t)((pix.first.x - map2D.minPix.x) + map_message.map.info.width * (pix.first.y - map2D.minPix.y));
		if (map2D.scores[pix.second] > score_threshold)
			map_message.map.data[mapIdx] = 100;
	}

	//make sure to set the header information on the map
	map_message.map.header.stamp = ros::Time::now();
	map_message.map.header.frame_id = tfListener.resolve(params.map_frame);
	map_message.map.header.seq = n_frames;

	// Publish map and map metadata
	sst.publish(map_message.map);
	sstm.publish(map_message.map.info);
}


void PointMapSLAM::publish_sub_frame(vector<PointXYZ> &pts0, ros::Time& stamp0)
{

  	const uint32_t POINT_STEP = 12;
	sensor_msgs::PointCloud2 pt_message;
	
	// make sure to set the header information on the map
	pt_message.header.stamp = stamp0;
	pt_message.header.frame_id = tfListener.resolve(params.map_frame);
	pt_message.header.seq = n_frames;

	// Data size
	pt_message.height = 1;
	pt_message.width = pts0.size();

	// PointField[] 
	pt_message.fields.resize(3);
	pt_message.fields[0].name = "x";
	pt_message.fields[0].offset = 0;
	pt_message.fields[0].datatype = sensor_msgs::PointField::FLOAT32;
	pt_message.fields[0].count = 1;

	pt_message.fields[1].name = "y";
	pt_message.fields[1].offset = 4;
	pt_message.fields[1].datatype = sensor_msgs::PointField::FLOAT32;
	pt_message.fields[1].count = 1;

	pt_message.fields[2].name = "z";
	pt_message.fields[2].offset = 8;
	pt_message.fields[2].datatype = sensor_msgs::PointField::FLOAT32;
	pt_message.fields[2].count = 1;

	pt_message.data.resize(std::max((size_t)1, pts0.size()) * POINT_STEP, 0x00);

	pt_message.point_step = POINT_STEP;
	pt_message.row_step = pt_message.data.size();

	pt_message.is_bigendian = false;
	pt_message.is_dense = true;

  	pt_message.data.resize(std::max((size_t)1, pts0.size()) * POINT_STEP, 0x00);
	uint8_t *ptr = pt_message.data.data();
	for (size_t i = 0; i < pts0.size(); i++)
	{
		*(reinterpret_cast<float*>(ptr + 0)) = pts0[i].x;
		*(reinterpret_cast<float*>(ptr + 4)) = pts0[i].y;
		*(reinterpret_cast<float*>(ptr + 8)) = pts0[i].z;
		ptr += POINT_STEP;
	}

	// Publish map and map metadata
	sub_frame_pub.publish(pt_message);
}



//-----------------------------------------------------------------------------------------------------------------------------
// SLAM functions
// **************

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
					  vector<double> &t)
{

	//////////////////////////////////////////
	// Preprocess frame and compute normals //
	//////////////////////////////////////////

	// Parameters

	// Create a copy of points in polar coordinates
	vector<PointXYZ> polar_pts(f_pts);
	cart2pol_(polar_pts, params.n_threads);
	
	t.push_back(omp_get_wtime());

	// Get angle for each lidar point, and the corresponding polar_r2
	if (params.polar_r2s.size() < 1)
	{
		vector<float> lidar_angles;
		get_lidar_angles(polar_pts, lidar_angles, params.lidar_n_lines);

		// Fill from last to first to respect ring order
		int j = lidar_angles.size() - 1;
		float tmp = params.theta_radius_ratio * (lidar_angles[j] - lidar_angles[j-1]);
		params.polar_r2s.push_back(pow(max(tmp, params.min_theta_radius), 2));
		j--;
		while (j > 0)
		{
			tmp = params.theta_radius_ratio * min(lidar_angles[j+1] - lidar_angles[j], lidar_angles[j] - lidar_angles[j-1]);
			params.polar_r2s.push_back(pow(max(tmp, params.min_theta_radius), 2));
			j--;
		}
		tmp = params.theta_radius_ratio * (lidar_angles[j + 1] - lidar_angles[j]);
		params.polar_r2s.push_back(pow(max(tmp, params.min_theta_radius), 2));


		// Get lidar angle resolution
		float minTheta, maxTheta;
		float lidar_angle_res = get_lidar_angle_res(polar_pts, minTheta, maxTheta, params.lidar_n_lines);

		// Define the polar neighbors radius in the scaled polar coordinates
		params.polar_r = params.theta_radius_ratio * lidar_angle_res;

	}

	// cout << "angle_res = " << lidar_angle_res << endl;
	// cout << "angles :" << endl;
	// for (auto& angle : lidar_angles)
	// {
	// 	cout << angle << endl;
	// }

	// Apply log scale to radius coordinate (in place)
	lidar_log_radius(polar_pts, params.polar_r, params.r_scale);

	// Apply horizontal scaling (to have smaller neighborhoods in horizontal direction)
	lidar_horizontal_scale(polar_pts, params.h_scale);
	

	// // Remove outliers (only for real frames)
	// if (params.motion_distortion)
	// {
	// 	// Get an outlier score
	// 	vector<float> scores(polar_pts.size(), 0.0);
	// 	detect_outliers(polar_pts, scores, params.lidar_n_lines, lidar_angle_res, minTheta, params.outl_rjct_passes, params.outl_rjct_thresh);

	// 	// Remove points with negative score
	// 	filter_pointcloud(f_pts, scores, 0);
	// 	filter_pointcloud(polar_pts, scores, 0);
	// }

	t.push_back(omp_get_wtime());

	// Get subsampling of the frame in carthesian coordinates (New points are barycenters or not?)
	grid_subsampling_centers(f_pts, sub_pts, sub_inds, params.frame_voxel_size);
	//grid_subsampling_spheres(f_pts, sub_pts, params.frame_voxel_size);

	// Convert sub_pts to polar and rescale
	vector<PointXYZ> polar_queries0(sub_pts);
	cart2pol_(polar_queries0);
	vector<PointXYZ> polar_queries(polar_queries0);
	lidar_log_radius(polar_queries, params.polar_r, params.r_scale);
	lidar_horizontal_scale(polar_queries, params.h_scale);

	// Get sub_rings
	vector<int> sub_rings;
	sub_rings.reserve(sub_inds.size());
	for (int j = 0; j < (int)sub_inds.size(); j++)
		sub_rings.push_back((int)f_rings[sub_inds[j]]);

	t.push_back(omp_get_wtime());

	/////////////////////
	// Compute normals //
	/////////////////////
	
	// Call polar processing function
	extract_lidar_frame_normals(f_pts, polar_pts, sub_pts, polar_queries, sub_rings, normals, norm_scores, params.polar_r2s, params.n_threads);

	t.push_back(omp_get_wtime());

	/////////////////////////
	// Get ground in frame //
	/////////////////////////

	// TODO: We can use this frame ground for other stuff, including ensuring planar ground???
	// 		 We can also use the region growing code for ground detection. Do that?

	// TODO: For complex corridor scenario, look into using the 3 normal orientations sampling of JE

	// Ransac ground extraction
	float vertical_thresh_deg = 20.0;
	float max_dist = 0.1;
	frame_ground = frame_ground_ransac(sub_pts, normals, vertical_thresh_deg, max_dist);

	// Ensure ground normal is pointing upwards
	if (frame_ground.u.z < 0)
		frame_ground.reverse();

	// Get height above ground
	frame_ground.point_distances_signed(sub_pts, heights);

	////////////////
	// Get scores //
	////////////////

	// Better normal score vased on distance and incidence angle
	smart_normal_score(sub_pts, polar_queries0, normals, norm_scores);

	// ICP score between 1.0 and 6.0 (chance of being sampled during ICP)
	icp_scores = vector<double>(norm_scores.begin(), norm_scores.end());
	smart_icp_score(polar_queries0, normals, heights, icp_scores);

	// Remove points with a negative score
	float min_score = 0.0001;
	filter_pointcloud(sub_pts, norm_scores, min_score);
	filter_pointcloud(normals, norm_scores, min_score);
	filter_anyvector(sub_inds, norm_scores, min_score);
	filter_anyvector(icp_scores, norm_scores, min_score);
	filter_anyvector(heights, norm_scores, min_score);
	filter_floatvector(norm_scores, min_score);
}


void PointMapSLAM::gotVeloCloud(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
	// Here we process raw lidar pointclouds
	// In this implementation, we never update the 3D map. We assume we already have a map
	if (params.filtering)
	{
		// If predictions are provided, we do not update the 2D global costmap
		processCloud(msg, false, false);

	}
	else
	{
		// If no predictions, we update the 2D global costmap
		processCloud(msg, false, true);
	}
}


void PointMapSLAM::gotClassifCloud(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
	// Here we process the classified point clouds (3 lidar frames already aligned)
	if (params.filtering)
	{
		processClassifCloud(msg, true, true);

	}
	else
	{
		processClassifCloud(msg, false, true);
	}

	return;
}


void PointMapSLAM::processCloud(const sensor_msgs::PointCloud2::ConstPtr& msg, bool filtering, bool update_map_2D)
{
	//////////////////////
	// Optional verbose //
	//////////////////////

	// params.verbose = 2;

	bool update_map = false;

	vector<string> clock_str;
	vector<double> t;
	if (params.verbose)
	{
		clock_str.reserve(20);
		t.reserve(20);
		clock_str.push_back("Msg filtering ..... ");
		clock_str.push_back("tf listener ....... ");
		clock_str.push_back("Pre-polar ......... ");
		clock_str.push_back("Pre-scale ......... ");
		clock_str.push_back("Pre-sub ........... ");
		clock_str.push_back("Pre-normals ....... ");
		clock_str.push_back("Pre-filter ........ ");
		clock_str.push_back("ICP localization .. ");
		clock_str.push_back("Publish tf ........ ");
		clock_str.push_back("Align sub_pts ..... ");
		clock_str.push_back("Map update ........ ");
		clock_str.push_back("Align f_pts ....... ");
		clock_str.push_back("Map2D update ...... ");
		clock_str.push_back("Map2D publish ..... ");
	}
	t.push_back(omp_get_wtime());

	//////////////////////////////
	// Read point cloud message //
	//////////////////////////////

	// Get the number of points
	size_t N = (size_t)(msg->width * msg->height);

	// Get timestamp
	ros::Time stamp = msg->header.stamp;

	// Ignore frames if not enough points
	if (msg->header.seq < 1 || N < 100)
	{
		ROS_WARN_STREAM("Frame #" << msg->header.seq << " with only " << N << " points is ignored.");
		return;
	}

	// Loop over points and copy in vector container. Do the filtering if necessary
	// float tan_theta2;
	// float last_tan_theta2 = 0;
	ushort last_ring = 0;
	int scan_col = 0;
	bool using_col = false;
	int col_stride = 2;
	vector<PointXYZ> f_pts;
	vector<float> f_ts;
	vector<ushort> f_rings;
	
	
	f_pts.reserve(N);
	if (filtering)
	{
		sensor_msgs::PointCloud2ConstIterator<float> iter_i(*msg, "intensity"), iter_x(*msg, "x"), iter_y(*msg, "y"), iter_time(*msg, "time");
		sensor_msgs::PointCloud2ConstIterator<ushort> iter_ring(*msg, "ring");
		for (sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");
			 iter_z != iter_z.end();
			 ++iter_x, ++iter_y, ++iter_z, ++iter_i, ++iter_ring, ++iter_time)
		{
			// Reject points with wrong labels
			if (find(params.loc_labels.begin(), params.loc_labels.end(), (int)*iter_i) == params.loc_labels.end())
				continue;

			// Reject NaN values
			if (isnan(*iter_x) || isnan(*iter_y) || isnan(*iter_z))
			{
				ROS_WARN_STREAM("rejected for NaN in point(" << *iter_x << ", " << *iter_y << ", " << *iter_z << ")");
				continue;
			}

			// Add kept points to the vector container
			f_pts.push_back(PointXYZ(*iter_x, *iter_y, *iter_z));
		}
	}
	else if (col_stride > 1)
	{
		sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x"), iter_y(*msg, "y"), iter_time(*msg, "time");
		sensor_msgs::PointCloud2ConstIterator<ushort> iter_ring(*msg, "ring");
		for (sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");
			 iter_z != iter_z.end();
			 ++iter_x, ++iter_y, ++iter_z, ++iter_ring, ++iter_time)
		{
			// // Eliminate according to angle theta
			// tan_theta2 = *iter_z * *iter_z / (*iter_x * *iter_x + *iter_x * *iter_x);
			// if (tan_theta2 < last_tan_theta2)
			// {
			// 	if (++scan_col >= col_stride)
			// 	{
			// 		scan_col = 0;
			// 		using_col = true;
			// 	}
			// 	else
			// 		using_col = false;
			// }
			// last_tan_theta2 = tan_theta2;

			// Eliminate according to angle theta
			if (*iter_ring < last_ring)
			{
				if (++scan_col >= col_stride)
				{
					scan_col = 0;
					using_col = true;
				}
				else
					using_col = false;
			}
			last_ring = *iter_ring;
			

			// Add all points to the vector container
			if (using_col)
			{
				f_pts.push_back(PointXYZ(*iter_x, *iter_y, *iter_z));
				f_ts.push_back(*iter_time);
				f_rings.push_back(*iter_ring);
			}
		}
	}
	else
	{
		sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x"), iter_y(*msg, "y"), iter_time(*msg, "time");
		sensor_msgs::PointCloud2ConstIterator<ushort> iter_ring(*msg, "ring");
		for (sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");
			 iter_z != iter_z.end();
			 ++iter_x, ++iter_y, ++iter_z, ++iter_ring, ++iter_time)
		{
			// Add all points to the vector container
			f_pts.push_back(PointXYZ(*iter_x, *iter_y, *iter_z));
			f_ts.push_back(*iter_time);
			f_rings.push_back(*iter_ring);
		}
	}

	t.push_back(omp_get_wtime());

	///////////////////////////////////////////
	// Get init matrix from current odometry //
	///////////////////////////////////////////

	// Get current pose of the scanner in the odom frame
	Eigen::Matrix4d H_OdomToScanner;
	try
	{
		H_OdomToScanner = transformListenerToEigenMatrix(tfListener, msg->header.frame_id, params.odom_frame, stamp);
	}
	catch (tf::ExtrapolationException e)
	{
		ROS_ERROR_STREAM("Extrapolation Exception. stamp = " << stamp << " now = " << ros::Time::now() << " delta = " << ros::Time::now() - stamp);
		return;
	}

	// If no odometry is given, previous one
	if (H_OdomToScanner.lpNorm<1>() < 0.001)
		H_OdomToScanner = last_H.inverse() * H_OdomToMap;

	// Get the pose of the scanner in the map
	Eigen::Matrix4d H_scannerToMap_init = H_OdomToMap * H_OdomToScanner.inverse();

	t.push_back(omp_get_wtime());

	// ROS_WARN_STREAM("TOdomToScanner(" << params.odom_frame << " to " << msg->header.frame_id << "):\n" << H_OdomToScanner);
	// ROS_WARN_STREAM("TOdomToMap(" << params.odom_frame << " to " << params.map_frame << "):\n" << H_OdomToMap);
	// ROS_WARN_STREAM("TscannerToMap (" << msg->header.frame_id << " to " << params.map_frame << "):\n" << H_scannerToMap_init);

	//////////////////////////////////////////
	// Preprocess frame and compute normals //
	//////////////////////////////////////////

	vector<PointXYZ> sub_pts;
	vector<PointXYZ> normals;
	vector<float> norm_scores;
	vector<double> icp_scores;
	vector<size_t> sub_inds;
	Plane3D frame_ground;
	vector<float> heights;
	preprocess_frame(f_pts, f_ts, f_rings, sub_pts, normals, norm_scores, icp_scores, sub_inds, frame_ground, heights, params, t);
	
	// Min and max times (dont loop on the whole frame as it is useless)
	float loop_ratio = 0.01;
	get_min_max_times(f_ts, t_min, t_max, loop_ratio);
	
	// Initial interpolation time
	float t0;
	if (map.size() < 1 || params.icp_params.max_iter < 1)
		t0 = t_min - (t_max - t_min);
	else
		t0 = params.icp_params.last_time + (float)(latest_stamp.toSec() - stamp.toSec());

	// Get the motion_distortion values from timestamps
	// 0 for t0 (old t_min) and 1 for t_max
	vector<float> sub_alphas;
	if (params.motion_distortion)
	{
		float inv_factor = 1 / (t_max - t0);
		sub_alphas.reserve(sub_inds.size());
		for (int j = 0; j < (int)sub_inds.size(); j++)
			sub_alphas.push_back((f_ts[sub_inds[j]] - t0) * inv_factor);
	}

	

	t.push_back(omp_get_wtime());

	/////////////////////////////////
	// Align frame on map with ICP //
	/////////////////////////////////

	// Create result containers
	ICP_results icp_results;

	if (n_frames < 1)
	{
		// In all cases if it is the first frame init the last_transform0
		params.icp_params.last_transform0 = H_scannerToMap_init;
	}
	if (map.size() < 1)
	{ // Case where we do not have a map yet. Override the first cloud position so that ground is at z=0
		
		// Get transformation that make the ground plane horizontal
		Eigen::Matrix3d ground_R;
		if (!rot_u_to_v(frame_ground.u, PointXYZ(0, 0, 1), ground_R))
			ground_R = Eigen::Matrix3d::Identity();
		
		// Get a point that belongs to the ground
		float min_dist = 1e9;
		PointXYZ ground_P;
		for (int j = 0; j < (int)heights.size(); j++)
		{
			float dist = abs(heights[j]);
			if (dist < min_dist)
			{
				min_dist = dist;
				ground_P = sub_pts[j];
			}
		}

		// Rotate point and get new ground height
		Eigen::Map<Eigen::Matrix<float, 3, 1>> ground_P_mat((float*)&ground_P, 3, 1);
		ground_P_mat = ground_R.cast<float>() * ground_P_mat;

		// // Update icp parameters to trigger flat ground
		params.icp_params.ground_w = 9.0;
		params.icp_params.ground_z = 0.0;

		// Update result transform
		icp_results.transform = Eigen::Matrix4d::Identity();
		icp_results.transform.block(0, 0, 3, 3) = ground_R;
		icp_results.transform(2, 3) = - ground_P_mat(2);
		params.icp_params.last_transform0 = icp_results.transform;
	}
	else
	{
		if (n_frames < 1)
		{
			// Case where we have a map, and the first frame needs to be aligned
			// We assume robot is still in the beginning so no motion distortion

			// 1. Initial RANSAC alignment



			// 2. ICP refine

			// TODO: Here we need to handle initial alignment case
			H_scannerToMap_init = Eigen::Matrix4d::Identity();
			H_scannerToMap_init(2, 3) = 0.7;

			// Case where we have a map, and the first frame needs to be aligned
			// We assume robot is still in the beginning so no motion distortion
			params.icp_params.init_transform = H_scannerToMap_init;
			params.icp_params.motion_distortion = false;
			PointToMapICP(sub_pts, sub_alphas, icp_scores, map, params.icp_params, icp_results);
			params.icp_params.motion_distortion = params.motion_distortion;

			// We override last_transform0 too to neglate motion distortion for this first frame
			params.icp_params.last_transform0 = icp_results.transform;
		}
		else
		{
			params.icp_params.init_transform = H_scannerToMap_init;
			PointToMapICP(sub_pts, sub_alphas, icp_scores, map, params.icp_params, icp_results);
		}

		// Safe Check
		if (icp_results.all_plane_rms.size() > 0.4 * params.icp_params.max_iter)
			ROS_WARN_STREAM("WARNING: ICP num_iter = " << icp_results.all_plane_rms.size());

		if (icp_results.all_plane_rms.size() > 0.9 * params.icp_params.max_iter)
			ROS_ERROR_STREAM("ERROR: ICP num_iter = " << icp_results.all_plane_rms.size());
	}

	t.push_back(omp_get_wtime());

	///////////////////////
	// Publish transform //
	///////////////////////
	
	// Update the last pose for future frames
	float alpha0 = (t_min - t0) / (t_max - t0);
	Eigen::Matrix4d new_H_scannerToMap = pose_interp(alpha0, params.icp_params.last_transform0, icp_results.transform, 0);

	// Compute tf
	//publishStamp = stamp;
	//publishLock.lock();
	H_OdomToMap = new_H_scannerToMap * H_OdomToScanner;

	// Publish tf
	if (stamp > latest_stamp)
	{
		tfBroadcaster.sendTransform(tf::StampedTransform(eigenMatrixToTransform(H_OdomToMap), stamp, params.map_frame, params.odom_frame));
	}
	// ROS_WARN_STREAM("TOdomToMap:\n" << H_OdomToMap);

	t.push_back(omp_get_wtime());

	////////////////////////////////
	// Publish aligned sub points //
	////////////////////////////////

	// //debug deep copies
	// vector<PointXYZ> sub_pts_dist = sub_pts;
	// vector<PointXYZ> normals_dist = normals;
	// vector<PointXYZ> f_pts_dist = f_pts;

	if (params.publish_sub_pts || update_map)
	{
		if (params.motion_distortion)
		{
			// Update map taking motion distortion into account
			size_t i_inds = 0;
			Eigen::Map<Eigen::Matrix<float, 3, Eigen::Dynamic>> pts_mat((float *)sub_pts.data(), 3, sub_pts.size());
			Eigen::Map<Eigen::Matrix<float, 3, Eigen::Dynamic>> norms_mat((float *)normals.data(), 3, normals.size());
			for (auto& alpha : sub_alphas)
			{
				Eigen::Matrix4d H_rect = pose_interp(alpha, params.icp_params.last_transform0, icp_results.transform, 0);
				Eigen::Matrix3f R_rect = (H_rect.block(0, 0, 3, 3)).cast<float>();
				Eigen::Vector3f T_rect = (H_rect.block(0, 3, 3, 1)).cast<float>();
				pts_mat.col(i_inds) = (R_rect * pts_mat.col(i_inds)) + T_rect;
				norms_mat.col(i_inds) = (R_rect * norms_mat.col(i_inds));
				i_inds++;
			}

			// //debug 
			// Eigen::Map<Eigen::Matrix<float, 3, Eigen::Dynamic>> pts_mat_dist((float *)sub_pts_dist.data(), 3, sub_pts_dist.size());
			// Eigen::Map<Eigen::Matrix<float, 3, Eigen::Dynamic>> norms_mat_dist((float *)normals_dist.data(), 3, normals_dist.size());
			// Eigen::Matrix3f R_tot = (new_H_scannerToMap.block(0, 0, 3, 3)).cast<float>();
			// Eigen::Vector3f T_tot = (new_H_scannerToMap.block(0, 3, 3, 1)).cast<float>();
			// pts_mat_dist = (R_tot * pts_mat_dist).colwise() + T_tot;
			// norms_mat_dist = R_tot * norms_mat_dist;

		}
		else
		{
			Eigen::Map<Eigen::Matrix<float, 3, Eigen::Dynamic>> pts_mat((float *)sub_pts.data(), 3, sub_pts.size());
			Eigen::Map<Eigen::Matrix<float, 3, Eigen::Dynamic>> norms_mat((float *)normals.data(), 3, normals.size());
			Eigen::Matrix3f R_tot = (icp_results.transform.block(0, 0, 3, 3)).cast<float>();
			Eigen::Vector3f T_tot = (icp_results.transform.block(0, 3, 3, 1)).cast<float>();
			pts_mat = (R_tot * pts_mat).colwise() + T_tot;
			norms_mat = R_tot * norms_mat;
		}
	}

	if (params.publish_sub_pts)
	{
		publish_sub_frame(sub_pts, stamp);
	}

	t.push_back(omp_get_wtime());
	
	////////////////////
	// Update the map //
	////////////////////

	if (update_map)
	{
		// The update function is called only on subsampled points as the others have no normal
		map.update(sub_pts, normals, norm_scores, n_frames);
	}

	t.push_back(omp_get_wtime());


	// Update the 2D map
	PointXYZ center;
	vector<PointXYZ> f_2Dpts;
	vector<int> f_2Drings;
	vector<float> f_2Dts;
	// vector<float> heights_2D;

	vector<ushort> wanted_rings = {0, 1, 2, 14};

	if (update_map_2D)
	{
		// Only use some rings not all
		f_2Dpts.reserve(f_pts.size());
		f_2Dts.reserve(f_ts.size());
		
		for (size_t i_2D = 0; i_2D < f_rings.size(); i_2D++)
		{
			auto found_r = find(wanted_rings.begin(), wanted_rings.end(), f_rings[i_2D]);
			if (found_r != wanted_rings.end())
			{
				f_2Dpts.push_back(f_pts[i_2D]);
				f_2Dts.push_back(f_ts[i_2D]);
				f_2Drings.push_back((int)(found_r - wanted_rings.begin()));
			}
		}

		// // Compute height for these rings
		// frame_ground.point_distances_signed(f_2Dpts, heights_2D);

		if (params.motion_distortion)
		{
			Eigen::Map<Eigen::Matrix<float, 3, Eigen::Dynamic>> pts_mat_f((float *)f_2Dpts.data(), 3, f_2Dpts.size());
			float inv_factor = 1 / (t_max - t0);
			#pragma omp parallel for schedule(static) num_threads(params.n_threads)
			for (int i = 0; i < f_2Dpts.size(); i++)
			{
				float alpha = (f_2Dts[i] - t0) * inv_factor;
				Eigen::Matrix4d H_rect = pose_interp(alpha, params.icp_params.last_transform0, icp_results.transform, 0);	
				Eigen::Matrix3f R_rect = (H_rect.block(0, 0, 3, 3)).cast<float>();
				Eigen::Vector3f T_rect = (H_rect.block(0, 3, 3, 1)).cast<float>();
				pts_mat_f.col(i) = (R_rect * pts_mat_f.col(i)) + T_rect;
				if (i == f_2Dpts.size() / 2)
				{
					center.x = T_rect.x();
					center.y = T_rect.y();
					center.z = T_rect.z();
				}
			}
		}
		else
		{
			Eigen::Map<Eigen::Matrix<float, 3, Eigen::Dynamic>> pts_mat((float *)f_2Dpts.data(), 3, f_2Dpts.size());
			Eigen::Matrix3f R_tot = (icp_results.transform.block(0, 0, 3, 3)).cast<float>();
			Eigen::Vector3f T_tot = (icp_results.transform.block(0, 3, 3, 1)).cast<float>();
			pts_mat = (R_tot * pts_mat).colwise() + T_tot;
			center.x = T_tot.x();
			center.y = T_tot.y();
			center.z = T_tot.z();
		}
	}

	t.push_back(omp_get_wtime());


	if (update_map_2D || map2D.size() < 1)
	{
		// Update 2D map
		map2D.update_from_3D(f_2Dpts, f_2Drings, wanted_rings.size(), center, params.map2d_zMin, params.map2d_zMax);
	}

	// // DEBUG ////////////////////////////

	// // Debug is case of strong angular rotation
	// Eigen::Matrix3d R2 = icp_results.transform.block(0, 0, 3, 3);
	// Eigen::Matrix3d R1 = params.icp_params.last_transform0.block(0, 0, 3, 3);
	// R1 = R2 * R1.transpose();
	// float dR = acos((R1.trace() - 1) / 2);

	// if (dR * 180 / M_PI > 8)
	// {
	// 	cout << string(90, ' ') << " DEBUG because of high rotation: " << dR * 180 / M_PI << " deg" << endl;

	// 	string path = "/home/administrator/1-Deep-Collider/results/tmp/";
	// 	char buffer[200];
	// 	sprintf(buffer, "debug_%05d_f_pts.ply", n_frames);
	// 	save_cloud(path + string(buffer), f_pts);

	// 	sprintf(buffer, "debug_%05d_f_2Ds.ply", n_frames);
	// 	save_cloud(path + string(buffer), f_2Dpts, heights_2D);

	// 	sprintf(buffer, "debug_%05d_f_sub.ply", n_frames);
	// 	vector<float> f12(icp_scores.begin(), icp_scores.end());
	// 	f12.insert(f12.end(), norm_scores.begin(),  norm_scores.end());
	// 	save_cloud(path + string(buffer), sub_pts, normals, f12);
	// }

	// /////////////////////////////////////

	t.push_back(omp_get_wtime());

	// Once for the initialization of ros global_costmap
	if (n_frames < 2 || true)
		publish_2D_map();

	t.push_back(omp_get_wtime());
	
	// Update the last pose for future frames
	if (stamp > latest_stamp)
	{
		params.icp_params.last_transform1 = icp_results.transform;
		params.icp_params.last_transform0 = new_H_scannerToMap;
		params.icp_params.last_time = t_min;
	}

	// Update the pose correction from map to odom
	H_OdomToMap = params.icp_params.last_transform1 * H_OdomToScanner;

	// Update the last pose for future frames
	last_H = icp_results.transform;


	// Save all poses
	if (stamp > latest_stamp)
	{
		all_H.push_back(icp_results.transform);
		f_times.push_back(stamp);
		latest_stamp = stamp;
		n_frames++;
	}

	////////////////////////
	// Debugging messages //
	////////////////////////

	double duration = 1000 * (t[t.size() - 1] - t[0]);
	
	cout << "[point_slam]: Processed Frame " << msg->header.frame_id << " with stamp " << stamp << " in " <<  duration << " ms" << endl;

	if (params.verbose)
	{
		for (size_t i = 0; i < min(t.size() - 1, clock_str.size()); i++)
		{
			double duration = 1000 * (t[i + 1] - t[i]);
			cout << clock_str[i] << duration << " ms" << endl;
		}
		cout << endl << "***********************" << endl << endl;
	}

	return;
}


void PointMapSLAM::processClassifCloud(const sensor_msgs::PointCloud2::ConstPtr& msg, bool filtering, bool update_map_2D)
{
	//////////////////////
	// Optional verbose //
	//////////////////////

	// params.verbose = 2;

	bool update_map = false;

	vector<string> clock_str;
	vector<double> t;
	if (params.verbose)
	{
		clock_str.reserve(20);
		t.reserve(20);
		clock_str.push_back("Msg filtering ..... ");
		clock_str.push_back("tf listener ....... ");
		clock_str.push_back("Pre-polar ......... ");
		clock_str.push_back("Pre-scale ......... ");
		clock_str.push_back("Pre-sub ........... ");
		clock_str.push_back("Pre-normals ....... ");
		clock_str.push_back("Pre-filter ........ ");
		clock_str.push_back("ICP localization .. ");
		clock_str.push_back("Publish tf ........ ");
		clock_str.push_back("Align sub_pts ..... ");
		clock_str.push_back("Map update ........ ");
		clock_str.push_back("Align f_pts ....... ");
		clock_str.push_back("Map2D update ...... ");
		clock_str.push_back("Map2D publish ..... ");
	}
	t.push_back(omp_get_wtime());

	//////////////////////////////
	// Read point cloud message //
	//////////////////////////////

	// Get the number of points
	size_t N = (size_t)(msg->width * msg->height);

	// Get timestamp
	ros::Time stamp = msg->header.stamp;

	// Ignore frames if not enough points
	if (msg->header.seq < 1 || N < 100)
	{
		ROS_WARN_STREAM("Frame #" << msg->header.seq << " with only " << N << " points is ignored.");
		return;
	}

	// Loop over points and copy in vector container. Do the filtering if necessary
	vector<PointXYZ> f_pts;
	f_pts.reserve(N);
	if (filtering)
	{
		sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x"), iter_y(*msg, "y");
		sensor_msgs::PointCloud2ConstIterator<uint8_t> iter_i(*msg, "label");
		for (sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");
			 iter_z != iter_z.end();
			 ++iter_x, ++iter_y, ++iter_z, ++iter_i)
		{
			// Reject points with wrong labels
			if (find(params.loc_labels.begin(), params.loc_labels.end(), (int)*iter_i) == params.loc_labels.end())
				continue;

			// Reject NaN values
			if (isnan(*iter_x) || isnan(*iter_y) || isnan(*iter_z))
			{
				ROS_WARN_STREAM("rejected for NaN in point(" << *iter_x << ", " << *iter_y << ", " << *iter_z << ")");
				continue;
			}

			// Add kept points to the vector container
			f_pts.push_back(PointXYZ(*iter_x, *iter_y, *iter_z));
		}
	}
	else
	{
		sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x"), iter_y(*msg, "y");
		for (sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");
			 iter_z != iter_z.end();
			 ++iter_x, ++iter_y, ++iter_z)
		{
			// Reject NaN values
			if (isnan(*iter_x) || isnan(*iter_y) || isnan(*iter_z))
			{
				ROS_WARN_STREAM("rejected for NaN in point(" << *iter_x << ", " << *iter_y << ", " << *iter_z << ")");
				continue;
			}

			// Add all points to the vector container
			f_pts.push_back(PointXYZ(*iter_x, *iter_y, *iter_z));
		}
	}

	t.push_back(omp_get_wtime());

	// ///////////////////////////////////////////
	// // Get init matrix from current odometry //
	// ///////////////////////////////////////////

	// // Get current pose of the scanner in the odom frame
	// Eigen::Matrix4d H_OdomToScanner;
	// try
	// {
	// 	H_OdomToScanner = transformListenerToEigenMatrix(tfListener, msg->header.frame_id, params.odom_frame, stamp);
	// }
	// catch (tf::ExtrapolationException e)
	// {
	// 	ROS_ERROR_STREAM("Extrapolation Exception. stamp = " << stamp << " now = " << ros::Time::now() << " delta = " << ros::Time::now() - stamp);
	// 	return;
	// }

	// // If no odometry is given, previous one
	// if (H_OdomToScanner.lpNorm<1>() < 0.001)
	// 	H_OdomToScanner = last_H.inverse() * H_OdomToMap;

	// // Get the pose of the scanner in the map
	// Eigen::Matrix4d H_scannerToMap_init = H_OdomToMap * H_OdomToScanner.inverse();

	// t.push_back(omp_get_wtime());

	// // ROS_WARN_STREAM("TOdomToScanner(" << params.odom_frame << " to " << msg->header.frame_id << "):\n" << H_OdomToScanner);
	// // ROS_WARN_STREAM("TOdomToMap(" << params.odom_frame << " to " << params.map_frame << "):\n" << H_OdomToMap);
	// // ROS_WARN_STREAM("TscannerToMap (" << msg->header.frame_id << " to " << params.map_frame << "):\n" << H_scannerToMap_init);

	// //////////////////////////////////////////
	// // Preprocess frame and compute normals //
	// //////////////////////////////////////////

	// vector<PointXYZ> sub_pts;
	// vector<PointXYZ> normals;
	// vector<float> norm_scores;
	// vector<double> icp_scores;
	// vector<size_t> sub_inds;
	// Plane3D frame_ground;
	// vector<float> heights;
	// preprocess_frame(f_pts, f_ts, f_rings, sub_pts, normals, norm_scores, icp_scores, sub_inds, frame_ground, heights, params, t);
	

	
	// // Min and max times (dont loop on the whole frame as it is useless)
	// float loop_ratio = 0.01;
	// get_min_max_times(f_ts, t_min, t_max, loop_ratio);
	
	// // Initial interpolation time
	// float t0;
	// if (map.size() < 1 || params.icp_params.max_iter < 1)
	// 	t0 = t_min - (t_max - t_min);
	// else
	// 	t0 = params.icp_params.last_time + (float)(latest_stamp.toSec() - stamp.toSec());



	// // Get the motion_distortion values from timestamps
	// // 0 for t0 (old t_min) and 1 for t_max
	// vector<float> sub_alphas;
	// if (params.motion_distortion)
	// {
	// 	float inv_factor = 1 / (t_max - t0);
	// 	sub_alphas.reserve(sub_inds.size());
	// 	for (int j = 0; j < (int)sub_inds.size(); j++)
	// 		sub_alphas.push_back((f_ts[sub_inds[j]] - t0) * inv_factor);
	// }

	

	// t.push_back(omp_get_wtime());

	// /////////////////////////////////
	// // Align frame on map with ICP //
	// /////////////////////////////////

	// // Create result containers
	// ICP_results icp_results;

	// if (n_frames < 1)
	// {
	// 	// In all cases if it is the first frame init the last_transform0
	// 	params.icp_params.last_transform0 = H_scannerToMap_init;
	// }
	// if (map.size() < 1)
	// { // Case where we do not have a map yet. Override the first cloud position so that ground is at z=0
		
	// 	// Get transformation that make the ground plane horizontal
	// 	Eigen::Matrix3d ground_R;
	// 	if (!rot_u_to_v(frame_ground.u, PointXYZ(0, 0, 1), ground_R))
	// 		ground_R = Eigen::Matrix3d::Identity();
		
	// 	// Get a point that belongs to the ground
	// 	float min_dist = 1e9;
	// 	PointXYZ ground_P;
	// 	for (int j = 0; j < (int)heights.size(); j++)
	// 	{
	// 		float dist = abs(heights[j]);
	// 		if (dist < min_dist)
	// 		{
	// 			min_dist = dist;
	// 			ground_P = sub_pts[j];
	// 		}
	// 	}

	// 	// Rotate point and get new ground height
	// 	Eigen::Map<Eigen::Matrix<float, 3, 1>> ground_P_mat((float*)&ground_P, 3, 1);
	// 	ground_P_mat = ground_R.cast<float>() * ground_P_mat;

	// 	// // Update icp parameters to trigger flat ground
	// 	params.icp_params.ground_w = 9.0;
	// 	params.icp_params.ground_z = 0.0;

	// 	// Update result transform
	// 	icp_results.transform = Eigen::Matrix4d::Identity();
	// 	icp_results.transform.block(0, 0, 3, 3) = ground_R;
	// 	icp_results.transform(2, 3) = - ground_P_mat(2);
	// 	params.icp_params.last_transform0 = icp_results.transform;
	// }
	// else
	// {
	// 	if (n_frames < 1)
	// 	{
	// 		// Case where we have a map, and the first frame needs to be aligned
	// 		// We assume robot is still in the beginning so no motion distortion

	// 		// 1. Initial RANSAC alignment



	// 		// 2. ICP refine

	// 		// TODO: Here we need to handle initial alignment case
	// 		H_scannerToMap_init = Eigen::Matrix4d::Identity();
	// 		H_scannerToMap_init(2, 3) = 0.7;

	// 		// Case where we have a map, and the first frame needs to be aligned
	// 		// We assume robot is still in the beginning so no motion distortion
	// 		params.icp_params.init_transform = H_scannerToMap_init;
	// 		params.icp_params.motion_distortion = false;
	// 		PointToMapICP(sub_pts, sub_alphas, icp_scores, map, params.icp_params, icp_results);
	// 		params.icp_params.motion_distortion = params.motion_distortion;

	// 		// We override last_transform0 too to neglate motion distortion for this first frame
	// 		params.icp_params.last_transform0 = icp_results.transform;
	// 	}
	// 	else
	// 	{
	// 		params.icp_params.init_transform = H_scannerToMap_init;
	// 		PointToMapICP(sub_pts, sub_alphas, icp_scores, map, params.icp_params, icp_results);
	// 	}

	// 	// Safe Check
	// 	if (icp_results.all_plane_rms.size() > 0.4 * params.icp_params.max_iter)
	// 		ROS_WARN_STREAM("WARNING: ICP num_iter = " << icp_results.all_plane_rms.size());

	// 	if (icp_results.all_plane_rms.size() > 0.9 * params.icp_params.max_iter)
	// 		ROS_ERROR_STREAM("ERROR: ICP num_iter = " << icp_results.all_plane_rms.size());
	// }

	// t.push_back(omp_get_wtime());

	// ///////////////////////
	// // Publish transform //
	// ///////////////////////
	
	// // Update the last pose for future frames
	// float alpha0 = (t_min - t0) / (t_max - t0);
	// Eigen::Matrix4d new_H_scannerToMap = pose_interp(alpha0, params.icp_params.last_transform0, icp_results.transform, 0);

	// // Compute tf
	// //publishStamp = stamp;
	// //publishLock.lock();
	// H_OdomToMap = new_H_scannerToMap * H_OdomToScanner;

	// // Publish tf
	// if (stamp > latest_stamp)
	// {
	// 	tfBroadcaster.sendTransform(tf::StampedTransform(eigenMatrixToTransform(H_OdomToMap), stamp, params.map_frame, params.odom_frame));
	// }
	// // ROS_WARN_STREAM("TOdomToMap:\n" << H_OdomToMap);

	// t.push_back(omp_get_wtime());

	// ////////////////////
	// // Update the map //
	// ////////////////////

	// // //debug deep copies
	// // vector<PointXYZ> sub_pts_dist = sub_pts;
	// // vector<PointXYZ> normals_dist = normals;
	// // vector<PointXYZ> f_pts_dist = f_pts;

	// if (update_map)
	// {
	// 	if (params.motion_distortion)
	// 	{
	// 		// Update map taking motion distortion into account
	// 		size_t i_inds = 0;
	// 		Eigen::Map<Eigen::Matrix<float, 3, Eigen::Dynamic>> pts_mat((float *)sub_pts.data(), 3, sub_pts.size());
	// 		Eigen::Map<Eigen::Matrix<float, 3, Eigen::Dynamic>> norms_mat((float *)normals.data(), 3, normals.size());
	// 		for (auto& alpha : sub_alphas)
	// 		{
	// 			Eigen::Matrix4d H_rect = pose_interp(alpha, params.icp_params.last_transform0, icp_results.transform, 0);
	// 			Eigen::Matrix3f R_rect = (H_rect.block(0, 0, 3, 3)).cast<float>();
	// 			Eigen::Vector3f T_rect = (H_rect.block(0, 3, 3, 1)).cast<float>();
	// 			pts_mat.col(i_inds) = (R_rect * pts_mat.col(i_inds)) + T_rect;
	// 			norms_mat.col(i_inds) = (R_rect * norms_mat.col(i_inds));
	// 			i_inds++;
	// 		}

	// 		// //debug 
	// 		// Eigen::Map<Eigen::Matrix<float, 3, Eigen::Dynamic>> pts_mat_dist((float *)sub_pts_dist.data(), 3, sub_pts_dist.size());
	// 		// Eigen::Map<Eigen::Matrix<float, 3, Eigen::Dynamic>> norms_mat_dist((float *)normals_dist.data(), 3, normals_dist.size());
	// 		// Eigen::Matrix3f R_tot = (new_H_scannerToMap.block(0, 0, 3, 3)).cast<float>();
	// 		// Eigen::Vector3f T_tot = (new_H_scannerToMap.block(0, 3, 3, 1)).cast<float>();
	// 		// pts_mat_dist = (R_tot * pts_mat_dist).colwise() + T_tot;
	// 		// norms_mat_dist = R_tot * norms_mat_dist;

	// 	}
	// 	else
	// 	{
	// 		Eigen::Map<Eigen::Matrix<float, 3, Eigen::Dynamic>> pts_mat((float *)sub_pts.data(), 3, sub_pts.size());
	// 		Eigen::Map<Eigen::Matrix<float, 3, Eigen::Dynamic>> norms_mat((float *)normals.data(), 3, normals.size());
	// 		Eigen::Matrix3f R_tot = (icp_results.transform.block(0, 0, 3, 3)).cast<float>();
	// 		Eigen::Vector3f T_tot = (icp_results.transform.block(0, 3, 3, 1)).cast<float>();
	// 		pts_mat = (R_tot * pts_mat).colwise() + T_tot;
	// 		norms_mat = R_tot * norms_mat;
	// 	}
	// }

	// t.push_back(omp_get_wtime());

	// if (update_map)
	// {
	// 	// The update function is called only on subsampled points as the others have no normal
	// 	map.update(sub_pts, normals, norm_scores, n_frames);
	// }

	// t.push_back(omp_get_wtime());


	// // Update the 2D map
	// PointXYZ center;
	// vector<PointXYZ> f_2Dpts;
	// vector<int> f_2Drings;
	// vector<float> f_2Dts;
	// // vector<float> heights_2D;

	// vector<ushort> wanted_rings = {0, 1, 2, 14};

	// if (update_map_2D)
	// {
	// 	// Only use some rings not all
	// 	f_2Dpts.reserve(f_pts.size());
	// 	f_2Dts.reserve(f_ts.size());
		
	// 	for (size_t i_2D = 0; i_2D < f_rings.size(); i_2D++)
	// 	{
	// 		auto found_r = find(wanted_rings.begin(), wanted_rings.end(), f_rings[i_2D]);
	// 		if (found_r != wanted_rings.end())
	// 		{
	// 			f_2Dpts.push_back(f_pts[i_2D]);
	// 			f_2Dts.push_back(f_ts[i_2D]);
	// 			f_2Drings.push_back((int)(found_r - wanted_rings.begin()));
	// 		}
	// 	}

	// 	// // Compute height for these rings
	// 	// frame_ground.point_distances_signed(f_2Dpts, heights_2D);

	// 	if (params.motion_distortion)
	// 	{
	// 		Eigen::Map<Eigen::Matrix<float, 3, Eigen::Dynamic>> pts_mat_f((float *)f_2Dpts.data(), 3, f_2Dpts.size());
	// 		float inv_factor = 1 / (t_max - t0);
	// 		#pragma omp parallel for schedule(static) num_threads(params.n_threads)
	// 		for (int i = 0; i < f_2Dpts.size(); i++)
	// 		{
	// 			float alpha = (f_2Dts[i] - t0) * inv_factor;
	// 			Eigen::Matrix4d H_rect = pose_interp(alpha, params.icp_params.last_transform0, icp_results.transform, 0);	
	// 			Eigen::Matrix3f R_rect = (H_rect.block(0, 0, 3, 3)).cast<float>();
	// 			Eigen::Vector3f T_rect = (H_rect.block(0, 3, 3, 1)).cast<float>();
	// 			pts_mat_f.col(i) = (R_rect * pts_mat_f.col(i)) + T_rect;
	// 			if (i == f_2Dpts.size() / 2)
	// 			{
	// 				center.x = T_rect.x();
	// 				center.y = T_rect.y();
	// 				center.z = T_rect.z();
	// 			}
	// 		}
	// 	}
	// 	else
	// 	{
	// 		Eigen::Map<Eigen::Matrix<float, 3, Eigen::Dynamic>> pts_mat((float *)f_2Dpts.data(), 3, f_2Dpts.size());
	// 		Eigen::Matrix3f R_tot = (icp_results.transform.block(0, 0, 3, 3)).cast<float>();
	// 		Eigen::Vector3f T_tot = (icp_results.transform.block(0, 3, 3, 1)).cast<float>();
	// 		pts_mat = (R_tot * pts_mat).colwise() + T_tot;
	// 		center.x = T_tot.x();
	// 		center.y = T_tot.y();
	// 		center.z = T_tot.z();
	// 	}
	// }

	// t.push_back(omp_get_wtime());


	// if (update_map_2D || map2D.size() < 1)
	// {
	// 	// Update 2D map
	// 	map2D.update_from_3D(f_2Dpts, f_2Drings, wanted_rings.size(), center, params.map2d_zMin, params.map2d_zMax);
	// }

	// // // DEBUG ////////////////////////////

	// // // Debug is case of strong angular rotation
	// // Eigen::Matrix3d R2 = icp_results.transform.block(0, 0, 3, 3);
	// // Eigen::Matrix3d R1 = params.icp_params.last_transform0.block(0, 0, 3, 3);
	// // R1 = R2 * R1.transpose();
	// // float dR = acos((R1.trace() - 1) / 2);

	// // if (dR * 180 / M_PI > 8)
	// // {
	// // 	cout << string(90, ' ') << " DEBUG because of high rotation: " << dR * 180 / M_PI << " deg" << endl;

	// // 	string path = "/home/administrator/1-Deep-Collider/results/tmp/";
	// // 	char buffer[200];
	// // 	sprintf(buffer, "debug_%05d_f_pts.ply", n_frames);
	// // 	save_cloud(path + string(buffer), f_pts);

	// // 	sprintf(buffer, "debug_%05d_f_2Ds.ply", n_frames);
	// // 	save_cloud(path + string(buffer), f_2Dpts, heights_2D);

	// // 	sprintf(buffer, "debug_%05d_f_sub.ply", n_frames);
	// // 	vector<float> f12(icp_scores.begin(), icp_scores.end());
	// // 	f12.insert(f12.end(), norm_scores.begin(),  norm_scores.end());
	// // 	save_cloud(path + string(buffer), sub_pts, normals, f12);
	// // }

	// // /////////////////////////////////////

	// t.push_back(omp_get_wtime());

	// // Once for the initialization of ros global_costmap
	// if (n_frames < 2 || true)
	// 	publish_2D_map();

	// t.push_back(omp_get_wtime());
	
	// // Update the last pose for future frames
	// if (stamp > latest_stamp)
	// {
	// 	params.icp_params.last_transform1 = icp_results.transform;
	// 	params.icp_params.last_transform0 = new_H_scannerToMap;
	// 	params.icp_params.last_time = t_min;
	// }

	// // Update the pose correction from map to odom
	// H_OdomToMap = params.icp_params.last_transform1 * H_OdomToScanner;

	// // Update the last pose for future frames
	// last_H = icp_results.transform;


	// // Save all poses
	// if (stamp > latest_stamp)
	// {
	// 	all_H.push_back(icp_results.transform);
	// 	f_times.push_back(stamp);
	// 	latest_stamp = stamp;
	// 	n_frames++;
	// }

	// ////////////////////////
	// // Debugging messages //
	// ////////////////////////

	// double duration = 1000 * (t[t.size() - 1] - t[0]);
	
	// cout << "[point_slam]: Processed Frame " << msg->header.frame_id << " with stamp " << stamp << " in " <<  duration << " ms" << endl;

	// if (params.verbose)
	// {
	// 	for (size_t i = 0; i < min(t.size() - 1, clock_str.size()); i++)
	// 	{
	// 		double duration = 1000 * (t[i + 1] - t[i]);
	// 		cout << clock_str[i] << duration << " ms" << endl;
	// 	}
	// 	cout << endl << "***********************" << endl << endl;
	// }

	return;
}


//void PointMapSLAM::update_transforms(const tf::tfMessage::ConstPtr& msg)
//{
//    // TODO
//
//    return;
//}

//-----------------------------------------------------------------------------------------------------------------------------
// Main call
// *********

int main(int argc, char **argv)
{

	///////////////////
	// Init ROS node //
	///////////////////

	// ROS init
	ROS_WARN("Initializing PointSLAM");
	ros::init(argc, argv, "PointSLAM");

	// Node handler and publishers
	ros::NodeHandle nh;
	ros::NodeHandle private_nh("~");
	//ros::Publisher pose_pub = nh.advertise<geometry_msgs::PoseStamped>("slam_pose", 1000);
	//ros::Publisher map_pub = nh.advertise<sensor_msgs::PointCloud2>("pointmap", 1000);

	//------------------------------------------------------------------------------------------------------------------
	//
	//    // TODO: Here useful functions to get the original map from a path variable
	//
	//    std::string tour_name("test1");
	//
	//    if (!nh.getParam("tour_name", tour_name)){
	//        std::cout << "ERROR READING TOUR NAME\n";
	//    }
	//    TourParser parser(tour_name);
	//    std::vector<ignition::math::Pose3d> route = parser.GetRoute();
	//
	//    std::string username = "default";
	//    if (const char * user = std::getenv("USER")){
	//        username = user;
	//    }
	//
	//    std::string start_time;
	//    if (!nh.getParam("start_time", start_time)){
	//        std::cout << "ERROR SETTING START TIME\n";
	//    }
	//
	//    std::string filepath = "/home/" + username + "/Myhal_Simulation/simulated_runs/" + start_time + "/";
	//
	//    std::ofstream log_file;
	//    log_file.open(filepath + "/logs-" + start_time + "/log.txt", std::ios_base::app);
	//
	//    ROS_WARN("USING TOUR %s\n", tour_name.c_str());
	//
	//------------------------------------------------------------------------------------------------------------------

	vector<PointXYZ> init_pts;
	vector<PointXYZ> init_normals;
	vector<float> init_scores;

	////////////////////////
	// Init Pointmap SLAM //
	////////////////////////

	// Parameters initialized with default values
	SLAM_params slam_params;

	// TODO: Load params values
	if (!private_nh.getParam("n_threads", slam_params.n_threads))
	{
		ROS_WARN("Warning: Cannot read n_threads");
	}
	if (!private_nh.getParam("map_voxel_size", slam_params.map_voxel_size))
	{
		ROS_WARN("Warning: Cannot read map_voxel_size");
	}
	if (!private_nh.getParam("frame_voxel_size", slam_params.frame_voxel_size))
	{
		ROS_WARN("Warning: Cannot read frame_voxel_size");
	}
	if (!private_nh.getParam("map2d_pixel_size", slam_params.map2d_pixel_size))
	{
		ROS_WARN("Warning: Cannot read map2d_pixel_size");
	}
	if (!private_nh.getParam("map2d_max_count", slam_params.map2d_max_count))
	{
		ROS_WARN("Warning: Cannot read map2d_max_count");
	}
	if (!private_nh.getParam("map2d_z_min", slam_params.map2d_zMin))
	{
		ROS_WARN("Warning: Cannot read map2d_zMin");
	}
	if (!private_nh.getParam("map2d_z_max", slam_params.map2d_zMax))
	{
		ROS_WARN("Warning: Cannot read map2d_zMax");
	}
	if (!private_nh.getParam("lidar_n_lines", slam_params.lidar_n_lines))
	{
		ROS_WARN("Warning: Cannot read lidar_n_lines");
	}
	if (!private_nh.getParam("motion_distortion", slam_params.motion_distortion))
	{
		ROS_WARN("Warning: Cannot read motion_distortion");
	}
	if (!private_nh.getParam("h_scale", slam_params.h_scale))
	{
		ROS_WARN("Warning: Cannot read h_scale");
	}
	if (!private_nh.getParam("r_scale", slam_params.r_scale))
	{
		ROS_WARN("Warning: Cannot read r_scale");
	}
	if (!private_nh.getParam("outl_rjct_passes", slam_params.outl_rjct_passes))
	{
		ROS_WARN("Warning: Cannot read outl_rjct_passes");
	}
	if (!private_nh.getParam("outl_rjct_thresh", slam_params.outl_rjct_thresh))
	{
		ROS_WARN("Warning: Cannot read outl_rjct_thresh");
	}
	int tmp = (int)slam_params.icp_params.n_samples;
	if (!private_nh.getParam("icp_samples", tmp))
	{
		ROS_WARN("Warning: Cannot read icp_samples");
	}
	slam_params.icp_params.n_samples = (size_t)tmp;
	if (!private_nh.getParam("icp_pairing_dist", slam_params.icp_params.max_pairing_dist))
	{
		ROS_WARN("Warning: Cannot read icp_pairing_dist");
	}
	if (!private_nh.getParam("icp_planar_dist", slam_params.icp_params.max_planar_dist))
	{
		ROS_WARN("Warning: Cannot read icp_planar_dist");
	}
	tmp = (int)slam_params.icp_params.avg_steps;
	if (!private_nh.getParam("icp_avg_steps", tmp))
	{
		ROS_WARN("Warning: Cannot read icp_avg_steps");
	}
	slam_params.icp_params.avg_steps = (size_t)tmp;
	tmp = (int)slam_params.icp_params.max_iter;
	if (!private_nh.getParam("icp_max_iter", tmp))
	{
		ROS_WARN("Warning: Cannot read icp_max_iter");
	}
	slam_params.icp_params.max_iter = (size_t)tmp;
	if (!private_nh.getParam("odom_frame", slam_params.odom_frame))
	{
		ROS_WARN("Warning: Cannot read odom_frame");
	}
	if (!private_nh.getParam("map_frame", slam_params.map_frame))
	{
		ROS_WARN("Warning: Cannot read map_frame");
	}
	if (!private_nh.getParam("base_frame", slam_params.base_frame))
	{
		ROS_WARN("Warning: Cannot read base_frame");
	}
	if (!private_nh.getParam("filter", slam_params.filtering))
	{
		ROS_WARN("Warning: Cannot read filter");
	}
	if (!private_nh.getParam("gt_classify", slam_params.gt_filter))
	{
		ROS_WARN("Warning: Cannot read gt_classify");
	}
	if (!private_nh.getParam("publish_sub_pts", slam_params.publish_sub_pts))
	{
		ROS_WARN("Warning: Cannot read publish_sub_pts");
	}

	// Update motion distortion in ICP params
	slam_params.icp_params.motion_distortion = slam_params.motion_distortion;

	// Get log path
	string user_name = "administrator";
	if (const char *user = getenv("USER"))
	{
		user_name = user;
	}
	string start_time;
	if (!nh.getParam("start_time", start_time))
	{
		slam_params.log_path = "/home/" + user_name + "/1-Deep-Collider/results/runs/Myhal/";
		// slam_params.log_path = "/home/" + user_name + "/1-Deep-Collider/results/runs/tmp/";
	}
	else
	{
		slam_params.log_path = "/home/" + user_name + "/1-Deep-Collider/results/runs/utias/";
		// slam_params.log_path = "/home/" + user_name + "/1-Deep-Collider/results/runs/";
		slam_params.log_path += start_time + "/logs-" + start_time + "/";
	}

	// Init filtered categories here
	if (slam_params.filtering)
	{
		if (slam_params.gt_filter)
			slam_params.loc_labels = vector<int>{0, 1, 4, 5, 6};
		else
			slam_params.loc_labels = vector<int>{0, 1, 2, 3};
	}

	/////////////////////
	// Get initial map //
	/////////////////////

	// // Get path from previous map if given
	// int init_map_ind;
	// if (!private_nh.getParam("init_map_ind", init_map_ind))
	// {
	// 	ROS_WARN("Warning: Cannot read init_map_ind");
	// 	init_map_ind = 0;
	// }
	// string init_day;
	// string init_path;
	// if (!private_nh.getParam("init_map_day", init_day))
	// {
	// 	init_path = "";
	// }
	// else
	// {
	// 	init_path = "/home/" + user_name + "/Myhal_Simulation/slam_offline/" + init_day;
	// 	char buffer[200];
	// 	sprintf(buffer, "/map_update_%04d.ply", init_map_ind);
	// 	init_path += string(buffer);
	// }

	string init_path;
	if (!private_nh.getParam("init_map_path", init_path))
	{
		init_path = "";
	}


	ROS_WARN_STREAM("Trying to load :" << init_path);

	// Load the previous map
	vector<int> counts;
	string float_scalar_name = "scores";
	string int_scalar_name = "";
	if (init_path.size() > 0)
	{
		ROS_WARN_STREAM("Loading :" << init_path);
		load_cloud_normals(init_path, init_pts, init_normals, init_scores, float_scalar_name, counts, int_scalar_name);
	}
	ROS_WARN_STREAM("Loading map: OK");

	///////////////////////
	// Init mapper class //
	///////////////////////

	// Create a the SLAM class
	ROS_WARN_STREAM("Building map ...");
	PointMapSLAM mapper(slam_params, init_pts, init_normals, init_scores);
	ROS_WARN_STREAM("Building map: OK");

	mapper.sst = nh.advertise<nav_msgs::OccupancyGrid>("map", 1, true);
	mapper.sstm = nh.advertise<nav_msgs::MapMetaData>("map_metadata", 1, true);
	mapper.sub_frame_pub = nh.advertise<sensor_msgs::PointCloud2>("sub_points", 1, true);
	

	///////////////////////
	// Start subscribing //
	///////////////////////

	// Dummy subscriber added by Hugues for debugging
	string visu_topic = "/static_visu";
	string visu_topic2 = "/dynamic_visu";
	ros::Subscriber visu_sub = nh.subscribe(visu_topic, 1, dummy_callback);
	ros::Subscriber visu_sub2 = nh.subscribe(visu_topic2, 1, dummy_callback);
	// ros::Subscriber visu_sub2 = nh.subscribe("/plan_costmap_3D", 1, dummy_callback_3);
	// ros::Subscriber visu_sub3 = nh.subscribe("/move_base/TebLocalPlannerROS/obstacles", 1, dummy_callback_4);

	

	// Subscribe to the lidar topic and the transforms topic
	//ros::Subscriber tf_sub = nh.subscribe("tf", 1000, mapper.update_transforms);
	string velo_topic = "/velodyne_points";
	string pred_topic = "/classified_points";

	// There are three possible behaviors
	if (slam_params.filtering)
	{
		// 1. Using the prediction form the collider
		ROS_WARN_STREAM("PointSlam in Collider mode: subscribing to " << pred_topic << " and " << velo_topic);
		ros::Subscriber pred_sub = nh.subscribe(pred_topic, 1, &PointMapSLAM::gotClassifCloud, &mapper);
		ros::Subscriber lidar_sub = nh.subscribe(velo_topic, 1, &PointMapSLAM::gotVeloCloud, &mapper);
		ros::spin();

	}
	else
	{
		// 3. Using raw point clouds
		ROS_WARN_STREAM("PointSlam in normal mode: subscribing to " << velo_topic);
		ros::Subscriber pred_sub = nh.subscribe(pred_topic, 1, dummy_callback_2);
		ros::Subscriber lidar_sub = nh.subscribe(velo_topic, 1, &PointMapSLAM::gotVeloCloud, &mapper);
		ros::spin();
	}

	// When shutting down save map and trajectories
	if (mapper.map.size() > 0 && mapper.n_frames > 1)
	{
		mapper.map.debug_save_ply(slam_params.log_path, 0);
		mapper.save_trajectory(slam_params.log_path);
	}

	return 0;
}