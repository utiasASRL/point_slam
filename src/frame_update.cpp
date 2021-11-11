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


//-----------------------------------------------------------------------------------------------------------------------------
// Utilities
// *********

Plane3D extract_ground(vector<PointXYZ>& points,
					   vector<PointXYZ>& normals,
					   float angle_vertical_thresh,
					   float dist_thresh,
					   int max_iter,
					   bool mode_2D)
{

	// In case of 2D mode, take the minimum height and use vertical normal
	if (mode_2D)
	{
		PointXYZ A = points[0];
		for (auto& p : points)
		{
			if (p.z < A.z)
				A = p;
		}
		return Plane3D(A, PointXYZ(0, 0, 1));
	}

	// Get the points with vertical normal (angle_vertical_thresh should be in radians)
	vector<PointXYZ> valid_points;
	valid_points.reserve(points.size());
	size_t i = 0;
	float cos_thresh = cos(angle_vertical_thresh);
	for (auto& n : normals)
	{
		if (abs(n.z) > cos_thresh)
		{
			valid_points.push_back(points[i]);
		}
		i++;
	}

	// Random generator
	default_random_engine generator;
	uniform_int_distribution<size_t> distribution(0, valid_points.size() - 1);

	// RANSAC loop
	int best_votes = 0;
	Plane3D best_P;
	for (int i = 0; i < max_iter; i++)
	{
		// Draw 3 random points
		unordered_set<size_t> unique_inds;
		vector<PointXYZ> ABC;
		while (unique_inds.size() < 3)
		{
			size_t ind = distribution(generator);
			unique_inds.insert(ind);
			if (unique_inds.size() > ABC.size())
				ABC.push_back(valid_points[ind]);
		}

		// Get the corresponding plane
		Plane3D candidate_P(ABC[0], ABC[1], ABC[2]);

		// Avoid ill defined planes
		if (candidate_P.u.sq_norm() < 1e-5)
		{
			i--;
			continue;
		}

		// Get the number of votes for this plane
		int votes = candidate_P.in_range(valid_points, dist_thresh);

		// Save best plane
		if (votes > best_votes)
		{
			best_votes = votes;
			best_P = candidate_P;
		}
	}
	return best_P;
}

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
		if (map2D.scores[pix.second] > 0.4)
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

//-----------------------------------------------------------------------------------------------------------------------------
// SLAM function
// *************

void PointMapSLAM::gotLocCloud(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
	// ROS_WARN_STREAM("Processing Loc Cloud");
	// Automatically in behavior 1: Using the prediction form the collider
	// Here we process raw clouds (no prediction so we do not update the map and map2D)
	processCloud(msg, false, false);
}

void PointMapSLAM::gotCloud(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
	// ROS_WARN_STREAM("Processing Cloud");

	if (params.filtering)
	{
		// 1. Using the prediction from the collider
		// Here we process classified clouds so we update the map and map2D)
		processCloud(msg, true, true);

	}
	else
	{
		// 3. Using raw point clouds
		processCloud(msg, false, true);
	}

	return;
}

void PointMapSLAM::processCloud(const sensor_msgs::PointCloud2::ConstPtr& msg, bool filtering, bool update_map)
{
	//////////////////////
	// Optional verbose //
	//////////////////////

	params.verbose = 0;

	vector<string> clock_str;
	vector<clock_t> t;
	if (params.verbose)
	{
		clock_str.reserve(20);
		t.reserve(20);
		clock_str.push_back("Msg filtering ..... ");
		clock_str.push_back("tf listener ....... ");
		clock_str.push_back("Polar convertion .. ");
		clock_str.push_back("Outlier reject .... ");
		clock_str.push_back("Grid subsampling .. ");
		clock_str.push_back("Frame normals ..... ");
		clock_str.push_back("Normals filter .... ");
		clock_str.push_back("ICP localization .. ");
		clock_str.push_back("Publish tf ........ ");
		clock_str.push_back("Align sub_pts ..... ");
		clock_str.push_back("Map update ........ ");
		clock_str.push_back("Align f_pts ....... ");
		clock_str.push_back("Map2D update ...... ");
		clock_str.push_back("Map2D publish ..... ");
	}
	t.push_back(std::clock());

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
	int col_stride = 3;
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

	t.push_back(std::clock());

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

	t.push_back(std::clock());

	// ROS_WARN_STREAM("TOdomToScanner(" << params.odom_frame << " to " << msg->header.frame_id << "):\n" << H_OdomToScanner);
	// ROS_WARN_STREAM("TOdomToMap(" << params.odom_frame << " to " << params.map_frame << "):\n" << H_OdomToMap);
	// ROS_WARN_STREAM("TscannerToMap (" << msg->header.frame_id << " to " << params.map_frame << "):\n" << H_scannerToMap_init);

	//////////////////////////////////////////
	// Preprocess frame and compute normals //
	//////////////////////////////////////////

	// Create a copy of points in polar coordinates
	vector<PointXYZ> polar_pts(f_pts);
	cart2pol_(polar_pts);

	t.push_back(std::clock());

	// Get angle for each lidar point, and the corresponding polar_r2
	if (params.polar_r2s.size() < 1)
	{
		vector<float> lidar_angles;
		get_lidar_angles(polar_pts, lidar_angles, params.lidar_n_lines);
		params.polar_r2s.reserve(lidar_angles.size());
		int j = 0;
		float tmp = 1.5 * (lidar_angles[j + 1] - lidar_angles[j]);
		params.polar_r2s.push_back(tmp * tmp);
		j++;
		while (j < lidar_angles.size() - 1)
		{
			tmp = 1.5 * min(lidar_angles[j+1] - lidar_angles[j], lidar_angles[j] - lidar_angles[j-1]);
			params.polar_r2s.push_back(tmp * tmp);
			j++;
		}
		tmp = 1.5 * (lidar_angles[j] - lidar_angles[j-1]);
		params.polar_r2s.push_back(tmp * tmp);

		// Get lidar angle resolution
		float minTheta, maxTheta;
		float lidar_angle_res = get_lidar_angle_res(polar_pts, minTheta, maxTheta, params.lidar_n_lines);
		params.polar_r = 1.5 * lidar_angle_res;
	}

	// Apply log scale to radius coordinate (in place)
	lidar_log_radius(polar_pts, params.polar_r, params.r_scale);

	// // Remove outliers (only for real frames)
	// if (params.motion_distortion)
	// {

	// 	// TODO: HERE modify outlier detection, using lidar angles
	// 	//			reduce the number of operations done by saving dtheta_min, in variable for example

	// 	string path = "/home/administrator/1-Deep-Collider/results/test_ptmap/";
	// 	char buffer[200];
	// 	sprintf(buffer, "debug_rtp_%05d.ply", n_frames);
	// 	string filepath = path + string(buffer);
	// 	save_cloud(filepath, polar_pts);

	// 	// Get an outlier score
	// 	vector<float> scores(polar_pts.size(), 0.0);
	// 	detect_outliers(polar_pts, scores, params.lidar_n_lines, lidar_angles, minTheta, params.outl_rjct_passes, params.outl_rjct_thresh);

	// 	// Remove points with negative score
	// 	filter_pointcloud(f_pts, scores, 0);
	// 	filter_pointcloud(polar_pts, scores, 0);

	// 	char buffer2[200];
	// 	sprintf(buffer2, "clean_rtp_%05d.ply", n_frames);
	// 	string filepath2 = path + string(buffer2);
	// 	save_cloud(filepath2, polar_pts);
	// }

	t.push_back(std::clock());

	// Get subsampling of the frame in carthesian coordinates
	vector<PointXYZ> sub_pts;
	vector<size_t> sub_inds;
	grid_subsampling_centers(f_pts, sub_pts, sub_inds, params.frame_voxel_size);

	t.push_back(std::clock());

	// Convert sub_pts to polar and rescale
	vector<PointXYZ> polar_queries0(sub_pts);
	cart2pol_(polar_queries0);
	vector<PointXYZ> polar_queries(polar_queries0);
	lidar_log_radius(polar_queries, params.polar_r, params.r_scale);
	lidar_horizontal_scale(polar_queries, params.h_scale);

	// ROS_WARN_STREAM(" ------> " << f_pts.size() << " " << sub_pts.size() << " " << sub_inds.size() << " " << params.frame_voxel_size);

	/////////////////////
	// Compute normals //
	/////////////////////

	// Init result containers
	vector<PointXYZ> normals;
	vector<float> norm_scores;

	// Apply horizontal scaling (to have smaller neighborhoods in horizontal direction)
	lidar_horizontal_scale(polar_pts, params.h_scale);

	// Get sub_rings
	vector<int> sub_rings;
	sub_rings.reserve(sub_inds.size());
	for (int j = 0; j < sub_inds.size(); j++)
		sub_rings.push_back((int)f_rings[sub_inds[j]]);

	// Call polar processing function
	extract_lidar_frame_normals(f_pts, polar_pts, sub_pts, polar_queries, sub_rings, normals, norm_scores, params.polar_r2s);

	t.push_back(std::clock());

	// Better normal score based on distance and incidence angle
	vector<float> icp_scores(norm_scores);
	smart_icp_score(polar_queries0, icp_scores);
	smart_normal_score(sub_pts, polar_queries0, normals, norm_scores);

	// Remove points with a low score
	float min_score = 0.01;
	min_score = -10000;
	filter_pointcloud(sub_pts, norm_scores, min_score);
	filter_pointcloud(normals, norm_scores, min_score);
	filter_anyvector(sub_inds, norm_scores, min_score);
	filter_anyvector(icp_scores, norm_scores, min_score);
	filter_floatvector(norm_scores, min_score);
	
	// Min and max times (dont loop on the whole frame as it is useless)
	float loop_ratio = 0.01;
	float t_min = f_ts[0];
	float t_max = f_ts[0];
	for (int j = 0; (float)j < loop_ratio * (float)f_ts.size();  j++)
	{
		if (f_ts[j] < t_min)
			t_min = f_ts[j];
	}
	for (int j = (int)floor((1 - loop_ratio) * f_ts.size()); j < f_ts.size();  j++)
	{
		if (f_ts[j] > t_max)
			t_max = f_ts[j];
	}
	
	// Initial interpolation time
	float t0;
	if (map.size() < 1 || params.icp_params.max_iter < 1)
		t0 = t_min - (t_max - t_min);
	else
		t0 = params.icp_params.last_time + (float)(latest_stamp.toSec() - stamp.toSec());



	// Get the motion_distorsion values from timestamps
	// 0 for t0 (old t_min) and 1 for t_max
	vector<float> sub_alphas;
	if (params.motion_distortion)
	{
		float inv_factor = 1 / (t_max - t0);
		sub_alphas.reserve(sub_inds.size());
		for (int j = 0; j < sub_inds.size(); j++)
			sub_alphas.push_back((f_ts[sub_inds[j]] - t0) * inv_factor);
	}

	

	t.push_back(std::clock());

	/////////////////////////////////
	// Align frame on map with ICP //
	/////////////////////////////////

	// Create result containers
	ICP_results icp_results;

	// If no map is available, use identity as the first pose
	if (map.size() < 1)
	{
		// icp_results.transform = H_scannerToMap_init;
		icp_results.transform = Eigen::Matrix4d::Identity();
		icp_results.transform(2, 3) = 0.7;
	
		// initialize last transformation
		params.icp_params.last_transform0 = icp_results.transform;
		params.icp_params.last_transform1 = icp_results.transform;
		params.icp_params.last_time = t_min;
	}
	else
	{
		params.icp_params.init_transform = params.icp_params.last_transform1;
		PointToMapICP(sub_pts, sub_alphas, icp_scores, map, params.icp_params, icp_results);
	}

	t.push_back(std::clock());

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

	t.push_back(std::clock());

	////////////////////
	// Update the map //
	////////////////////

	if (update_map)
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

	t.push_back(std::clock());

	if (update_map)
	{
		// The update function is called only on subsampled points as the others have no normal
		map.update(sub_pts, normals, norm_scores);
	}

	t.push_back(std::clock());

	// Update the 2D map
	PointXYZ center;
	if (update_map)
	{
		if (params.motion_distortion)
		{
			// TODO: Here do not use the whole f_pts cloud
			Eigen::Map<Eigen::Matrix<float, 3, Eigen::Dynamic>> pts_mat_f((float *)f_pts.data(), 3, f_pts.size());
			float inv_factor = 1 / (t_max - t0);
			for (int i = 0; i < f_pts.size(); i++)
			{
				float alpha = (f_ts[i] - t0) * inv_factor;
				Eigen::Matrix4d H_rect = pose_interp(alpha, params.icp_params.last_transform0, icp_results.transform, 0);	
				Eigen::Matrix3f R_rect = (H_rect.block(0, 0, 3, 3)).cast<float>();
				Eigen::Vector3f T_rect = (H_rect.block(0, 3, 3, 1)).cast<float>();
				pts_mat_f.col(i) = (R_rect * pts_mat_f.col(i)) + T_rect;
				center.x = T_rect.x();
				center.y = T_rect.y();
				center.z = T_rect.z();
			}
		}
		else
		{
			Eigen::Map<Eigen::Matrix<float, 3, Eigen::Dynamic>> pts_mat((float *)f_pts.data(), 3, f_pts.size());
			Eigen::Matrix3f R_tot = (icp_results.transform.block(0, 0, 3, 3)).cast<float>();
			Eigen::Vector3f T_tot = (icp_results.transform.block(0, 3, 3, 1)).cast<float>();
			pts_mat = (R_tot * pts_mat).colwise() + T_tot;
			center.x = T_tot.x();
			center.y = T_tot.y();
			center.z = T_tot.z();
		}
	}

	t.push_back(std::clock());

	if (update_map || map2D.size() < 1)
	{

		// Detect ground plane for height filtering
		Plane3D ground_P = extract_ground(sub_pts, normals);

		// Update 2D map
		map2D.update_from_3D(f_pts, center, ground_P, params.map2d_zMin, params.map2d_zMax);
	}

	// DEBUG ////////////////////////////

	if (map2D.size() > 0 && n_frames % 1 == 0)
	{
		// string path = "/home/administrator/1-Deep-Collider/results/test_ptmap/";
		string path = "/home/administrator/catkin_ws/src/point_slam_2/src/test_maps/maps/";
		char buffer[200];
		sprintf(buffer, "debug_frame_%05d.ply", n_frames);
		string filepath = path + string(buffer);
		vector<float> distances;
		// ground_P.point_distances(f_pts, distances);
		// save_cloud(filepath, f_pts, distances);
		save_cloud(filepath, f_pts);

		map.debug_save_ply(path, n_frames);
		map2D.debug_save_ply(path, n_frames);

		// ROS_WARN_STREAM(">>>>>>>>>>>>> " << n_frames << ": " << ground_P.u << " " << ground_P.d);
	}

	/////////////////////////////////////

	t.push_back(std::clock());

	publish_2D_map();

	t.push_back(std::clock());
	
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

	if (update_map)
	{
		// Update number of frames
		n_frames++;
	}

	// Save all poses
	if (stamp > latest_stamp)
	{
		all_H.push_back(icp_results.transform);
		f_times.push_back(stamp);
		latest_stamp = stamp;
	}

	////////////////////////
	// Debugging messages //
	////////////////////////

	double duration = 1000 * (t[t.size() - 1] - t[0]) / (double)CLOCKS_PER_SEC;
	ROS_WARN_STREAM("Processed Frame " << msg->header.frame_id << " with stamp " << stamp << " in " <<  duration << " ms");

	if (params.verbose)
	{
		for (size_t i = 0; i < min(t.size() - 1, clock_str.size()); i++)
		{
			double duration = 1000 * (t[i + 1] - t[i]) / (double)CLOCKS_PER_SEC;
			cout << clock_str[i] << duration << " ms" << endl;
		}
		cout << endl << "***********************" << endl << endl;
	}

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

	// Get log path
	string user_name = "administrator";
	if (const char *user = getenv("USER"))
	{
		user_name = user;
	}
	string start_time;
	if (!nh.getParam("start_time", start_time))
	{
		// slam_params.log_path = "/home/" + user_name + "/1-Deep-Collider/results/runs/tmp/";
		slam_params.log_path = "/home/" + user_name + "/catkin_ws/src/point_slam_2/src/test_maps/logs/";
	}
	else
	{
		// slam_params.log_path = "/home/" + user_name + "/1-Deep-Collider/results/runs/";
		slam_params.log_path = "/home/" + user_name + "/catkin_ws/src/point_slam_2/src/test_maps/logs/";
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

	///////////////////////
	// Init mapper class //
	///////////////////////

	// Create a the SLAM class
	PointMapSLAM mapper(slam_params, init_pts, init_normals, init_scores);

	mapper.sst = nh.advertise<nav_msgs::OccupancyGrid>("map", 1, true);
	mapper.sstm = nh.advertise<nav_msgs::MapMetaData>("map_metadata", 1, true);

	///////////////////////
	// Start subscribing //
	///////////////////////

	// Dummy subscriber added by Hugues for debugging
	string visu_topic = "/collision_visu";
	ros::Subscriber visu_sub = nh.subscribe(visu_topic, 1, dummy_callback); 

	// Subscribe to the lidar topic and the transforms topic
	//ros::Subscriber tf_sub = nh.subscribe("tf", 1000, mapper.update_transforms);
	string velo_topic = "/velodyne_points";
	string pred_topic = "/classified_points";

	// There are three possible behaviors
	if (slam_params.filtering)
	{
		// 1. Using the prediction form the collider
		ROS_WARN_STREAM("PointSlam in Collider mode: subscribing to " << pred_topic << " and " << velo_topic);
		ros::Subscriber pred_sub = nh.subscribe(pred_topic, 1, &PointMapSLAM::gotCloud, &mapper);
		ros::Subscriber lidar_sub = nh.subscribe(velo_topic, 1, &PointMapSLAM::gotLocCloud, &mapper);
		ros::spin();

	}
	else
	{
		// 3. Using raw point clouds
		ROS_WARN_STREAM("PointSlam in normal mode: subscribing to " << velo_topic);
		ros::Subscriber pred_sub = nh.subscribe(pred_topic, 1, dummy_callback);
		ros::Subscriber lidar_sub = nh.subscribe(velo_topic, 1, &PointMapSLAM::gotCloud, &mapper);
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