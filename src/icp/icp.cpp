
#include "icp.h"



// Utils
// *****

// global counter
int count_iter = 0;

void regu_pose_cycle(vector<Eigen::Matrix4d>& H, vector<float>& H_w)
{
	// Bundle regularization
	//
	// H[i] = H(i,i-1) transformation that alignes frame i on frame i-1 (i is modulo B)
	// Example for B = 5
	// We Have H04 H10 H21 H32 H43 
	// For H10 we compute H_bis = H40 * H34 * H23 * H12 which we want equal to H10
	// Therefore newH10 = pose_interp(w1, H10, H_bis, 0)
	// To regularize all poses together, all wi = 1/B. But if you favor one transformation (the largest for example). Change w

	// Ensure sum of w equals 1
	float reg_w = 1.0 / std::accumulate(H_w.begin(), H_w.end(), 0);
	for (auto& w : H_w)
		w *= reg_w;

	size_t B = H.size();
	for (size_t b = 0; b < B; b++)
	{
		Eigen::Matrix4d H_bis = Eigen::Matrix4d::Identity(4, 4);
		for (size_t bb = b + 1; bb < b + B; bb++)
		{
			size_t bb_0 = bb % B;
			H_bis = H[bb_0].inverse() * H_bis;
		}
		H[b] = pose_interp(1.0 / B, H[b], H_bis, 0);
	}
}


// Minimizer
// *********


void SolvePoint2PlaneLinearSystem(const Matrix6d& A, const Vector6d& b, Vector6d& x)
{
	// Define a slover on matrix A
	Eigen::FullPivHouseholderQR<Matrix6d> Aqr(A);

	if (!Aqr.isInvertible())
	{
		cout << "WARNING: Full minimization instead of cholesky" << endl;
		// Solve system (but solver is slow???)
		x = Aqr.solve(b);

		/*
		// Solve reduced problem R1 x = Q1^T b instead of QR x = b, where Q = [Q1 Q2] and R = [ R1 ; R2 ] such that ||R2|| is small (or zero) and therefore A = QR ~= Q1 * R1
		const int rank = Aqr.rank();
		const int rows = A.rows();
		const Matrix Q1t = Aqr.matrixQ().transpose().block(0, 0, rank, rows);
		const Matrix R1 = (Q1t * A * Aqr.colsPermutation()).block(0, 0, rank, rows);

		const bool findMinimalNormSolution = true; // TODO is that what we want?

		// The under-determined system R1 x = Q1^T b is made unique ..
		if (findMinimalNormSolution) {
			// by getting the solution of smallest norm (x = R1^T * (R1 * R1^T)^-1 Q1^T b.
			x = R1.template triangularView<Eigen::Upper>().transpose() * (R1 * R1.transpose()).llt().solve(Q1t * b);
		}
		else {
			// by solving the simplest problem that yields fewest nonzero components in x
			x.block(0, 0, rank, 1) = R1.block(0, 0, rank, rank).template triangularView<Eigen::Upper>().solve(Q1t * b);
			x.block(rank, 0, rows - rank, 1).setZero();
		}

		x = Aqr.colsPermutation() * x;

		BOOST_AUTO(ax, (A * x).eval());
		if (!b.isApprox(ax, 1e-5)) {
			LOG_INFO_STREAM("PointMatcher::icp - encountered almost singular matrix while minimizing point to plane distance. QR solution was too inaccurate. Trying more accurate approach using double precision SVD.");
			x = A.template cast<double>().jacobiSvd(ComputeThinU | ComputeThinV).solve(b.template cast<double>()).template cast<T>();
			ax = A * x;

			if ((b - ax).norm() > 1e-5 * std::max(A.norm() * x.norm(), b.norm())) {
				LOG_WARNING_STREAM("PointMatcher::icp - encountered numerically singular matrix while minimizing point to plane distance and the current workaround remained inaccurate."
					<< " b=" << b.transpose()
					<< " !~ A * x=" << (ax).transpose().eval()
					<< ": ||b- ax||=" << (b - ax).norm()
					<< ", ||b||=" << b.norm()
					<< ", ||ax||=" << ax.norm());
			}
		}
		*/
	}
	else
	{
		// Cholesky decomposition
		x = A.llt().solve(b);
	}
}

void PointToPlaneErrorMinimizer(vector<PointXYZ> &targets,
								vector<PointXYZ> &references,
								vector<PointXYZ> &refNormals,
								vector<float> &weights,
								vector<pair<size_t, size_t>> &sample_inds,
								Eigen::Matrix4d &mOut,
								vector<bool> &is_ground,
								double ground_z)
{
	// See: "Linear Least-Squares Optimization for Point-to-Plane ICP Surface Registration" (Kok-Lim Low)
	// init A and b matrice
	size_t N = sample_inds.size();
	Eigen::Matrix<double, Eigen::Dynamic, 6> A(N, 6);
	Eigen::Matrix<double, Eigen::Dynamic, 6> wA(N, 6);
	Eigen::Matrix<double, Eigen::Dynamic, 1> b(N, 1);

	// Fill matrices values
	bool tgt_weights = weights.size() == targets.size();
	bool ref_weights = weights.size() == references.size();
	bool force_flat_ground = is_ground.size() == sample_inds.size();
	int i = 0;
	for (const auto& ind : sample_inds)
	{
		// Target point
		double sx = (double)targets[ind.first].x;
		double sy = (double)targets[ind.first].y;
		double sz = (double)targets[ind.first].z;

		// Reference point
		double dx = (double)references[ind.second].x;
		double dy = (double)references[ind.second].y;
		double dz;

		// Reference point normal
		double nx;
		double ny;
		double nz;

		// Special case to force a flat ground
		if (force_flat_ground && is_ground[i])
		{
			dz = ground_z;
			nx = 0;
			ny = 0;
			nz = 1;
		}
		else
		{
			dz = (double)references[ind.second].z;
			nz = (double)refNormals[ind.second].z;
			nx = (double)refNormals[ind.second].x;
			ny = (double)refNormals[ind.second].y;
		}

		// setup least squares system
		A(i, 0) = nz * sy - ny * sz;
		A(i, 1) = nx * sz - nz * sx;
		A(i, 2) = ny * sx - nx * sy;
		A(i, 3) = nx;
		A(i, 4) = ny;
		A(i, 5) = nz;
		b(i, 0) = nx * dx + ny * dy + nz * dz - nx * sx - ny * sy - nz * sz;

		// Apply weights if needed
		if (tgt_weights)
			wA.row(i) = A.row(i) * (double)weights[ind.first];
		else if (ref_weights)
			wA.row(i) = A.row(i) * (double)weights[ind.second];
		i++;
	}

	// linear least square matrices
	Matrix6d A_ = wA.transpose() * A;
	Vector6d b_ = wA.transpose() * b;

	// Solve linear optimization
	Vector6d x;
	SolvePoint2PlaneLinearSystem(A_, b_, x);

	// Get transformation rotation
	Eigen::Transform<double, 3, Eigen::Affine> transform;
	transform = Eigen::AngleAxis<double>(x.head(3).norm(), x.head(3).normalized());

	// Reverse roll-pitch-yaw conversion, very useful piece of knowledge, keep it with you all time!
	//const float pitch = -asin(transform(2,0));
	//const float roll = atan2(transform(2,1), transform(2,2));
	//const float yaw = atan2(transform(1,0) / cos(pitch), transform(0,0) / cos(pitch));
	//std::cerr << "d angles" << x(0) - roll << ", " << x(1) - pitch << "," << x(2) - yaw << std::endl;

	// Get transformation translation
	transform.translation() = x.segment(3, 3);

	// Convert to 4x4 matrix
	mOut = transform.matrix();

	if (mOut != mOut)
	{
		// Degenerate situation. This can happen when the source and reading clouds
		// are identical, and then b and x above are 0, and the rotation matrix cannot
		// be determined, it comes out full of NaNs. The correct rotation is the identity.
		mOut.block(0, 0, 3, 3) = Eigen::Matrix4d::Identity(3, 3);
	}
}



// ICP functions
// *************






void PointToMapICP(vector<PointXYZ>& tgt_pts, vector<float>& tgt_t,
	vector<double>& tgt_w,
	PointMap& map,
	ICP_params& params,
	ICP_results& results)
{
	// Parameters
	// **********

	size_t N = tgt_pts.size();
	float max_pair_d2 = params.max_pairing_dist * params.max_pairing_dist;
	size_t first_steps = 2;

	// Initially use a large associating dist (we change that during the interations)
	float max_planar_d = 4 * params.max_planar_dist;

	// Create search parameters
	nanoflann::SearchParams search_params;
	//search_params.sorted = false;

	// Start ICP loop
	// **************

	// Matrix of original data (only shallow copy of ref clouds)
	Eigen::Map<Eigen::Matrix<float, 3, Eigen::Dynamic>> map_mat((float*)map.cloud.pts.data(), 3, map.cloud.pts.size());

	// Aligned points (Deep copy of targets)
	vector<PointXYZ> aligned(tgt_pts);

	// Matrix for original/aligned data (Shallow copy of parts of the points vector)
	Eigen::Map<Eigen::Matrix<float, 3, Eigen::Dynamic>> targets_mat((float*)tgt_pts.data(), 3, N);
	Eigen::Map<Eigen::Matrix<float, 3, Eigen::Dynamic>> aligned_mat((float*)aligned.data(), 3, N);

	// Apply initial transformation
	results.transform = params.init_transform;
	if (params.motion_distortion)
	{
		size_t i_inds = 0;	
		for (auto& t : tgt_t)	
		{	
			Eigen::Matrix4d H_rect = pose_interp(t, params.last_transform0, results.transform, 0);			
			Eigen::Matrix3f R_rect = (H_rect.block(0, 0, 3, 3)).cast<float>();	
			Eigen::Vector3f T_rect = (H_rect.block(0, 3, 3, 1)).cast<float>();	
			aligned_mat.col(i_inds) = (R_rect * targets_mat.col(i_inds)) + T_rect;	
			i_inds++;	
		}
	}
	else
	{
		Eigen::Matrix3f R_tot = (results.transform.block(0, 0, 3, 3)).cast<float>();
		Eigen::Vector3f T_tot = (results.transform.block(0, 3, 3, 1)).cast<float>();
		aligned_mat = (R_tot * targets_mat).colwise() + T_tot;
	}

	// Random generator
  	unsigned seed = chrono::system_clock::now().time_since_epoch().count();
	default_random_engine generator(seed);
	discrete_distribution<int> distribution(tgt_w.begin(), tgt_w.end());

	// Init result containers
	Eigen::Matrix4d H_icp;
	results.all_rms.reserve(params.max_iter);
	results.all_plane_rms.reserve(params.max_iter);


	// Convergence varaibles
	float mean_dT = 0;
	float mean_dR = 0;
	size_t max_it = params.max_iter;
	bool stop_cond = false;

	vector<clock_t> t(6);
	vector<string> clock_str;
	clock_str.push_back("Random_Sample ... ");
	clock_str.push_back("KNN_search ...... ");
	clock_str.push_back("Optimization .... ");
	clock_str.push_back("Regularization .. ");
	clock_str.push_back("Result .......... ");

	// // Debug (save map.cloud.pts)
	// string path = "/home/administrator/catkin_ws/src/point_slam/src/test_maps/test_ply/";
	// char buffer[100];
	// char buffer_check[100];
	// sprintf(buffer, "map_before_ICP_%03d.ply", int(count_iter));
	// sprintf(buffer_check, "aligned_before_ICP_%03d.ply", int(count_iter));
	// string filepath = path + string(buffer);
	// string filepath_check = path + string(buffer_check);
	// save_cloud(filepath, map.cloud.pts);
	// save_cloud(filepath_check, aligned, tgt_t);

	for (size_t step = 0; step < max_it; step++)
	{
		/////////////////
		// Association //
		/////////////////

		// Pick random queries (use unordered set to ensure uniqueness)
		vector<pair<size_t, size_t>> sample_inds;
		if (params.n_samples < N)
		{
			// Random picking
			unordered_set<size_t> unique_inds;
			int count_tries = 0;
			while (unique_inds.size() < params.n_samples && count_tries < (int)params.n_samples * 10)
			{
				// Be sure to ignore points that are considered outliers
				size_t picked = (size_t)distribution(generator);
				if (tgt_w[picked] > 0.05)
					unique_inds.insert(picked);
				count_tries++;
			}

			// Debugging if we could not pick enough indices
			if (unique_inds.size() < params.n_samples)
			{
				for (int iiii = 0; iiii < (int)tgt_w.size(); iiii++)
				{
					if (tgt_w[iiii] > 20.5 || tgt_w[iiii] < 0.3)
					cout << "tgt_w: " << iiii << " -> " << tgt_w[iiii] << endl;
				}
				count_tries = 0;
				while (unique_inds.size() < params.n_samples && count_tries < (int)params.n_samples)
				{
					size_t picked_ind = (size_t)distribution(generator);
					unique_inds.insert(picked_ind);
					count_tries++;
					cout << count_tries << ": " << picked_ind << " -> " << unique_inds.size() << "/" << params.n_samples << endl;
				}
				throw std::invalid_argument( "Impossible to pick enough icp samples" );
			}

			sample_inds = vector<pair<size_t, size_t>>(params.n_samples);
			size_t i = 0;
			for (const auto& ind : unique_inds)
			{
				sample_inds[i].first = ind;
				// chosen_inds[ind] = 1.0f;
				i++;
			}
		}
		else
		{
			sample_inds = vector<pair<size_t, size_t>>(N);
			for (size_t i = 0; i < N; i++)
			{
				sample_inds[i].first = i;
				i++;
			}
		}

		t[1] = std::clock();

		// Init neighbors container
		vector<float> nn_dists(sample_inds.size());

		// Find nearest neigbors
		// #pragma omp parallel for shared(max_neighbs) schedule(dynamic, 10) num_threads(n_thread)
		for (size_t i = 0; i < sample_inds.size(); i++)
		{
			nanoflann::KNNResultSet<float> resultSet(1);
			resultSet.init(&sample_inds[i].second, &nn_dists[i]);
			map.tree.findNeighbors(resultSet, (float*)&aligned[sample_inds[i].first], search_params);
		}

		t[2] = std::clock();


		///////////////////////
		// Distances metrics //
		///////////////////////

		// Update association distance after a few iterations
		if (step == first_steps)
			max_planar_d = params.max_planar_dist;


		// Erase sample_inds if dists is too big
		vector<pair<size_t, size_t>> filtered_sample_inds;
		filtered_sample_inds.reserve(sample_inds.size());
		float rms2 = 0;
		float prms2 = 0;
		for (size_t i = 0; i < sample_inds.size(); i++)
		{
			if (nn_dists[i] < max_pair_d2)
			{
				// Check planar distance (only after a few steps for initial alignment)
				PointXYZ diff = (map.cloud.pts[sample_inds[i].second] - aligned[sample_inds[i].first]);
				float planar_dist = abs(diff.dot(map.normals[sample_inds[i].second]));
				if (planar_dist < max_planar_d)
				{
					// Keep samples
					filtered_sample_inds.push_back(sample_inds[i]);

					// Update pt2pt rms
					rms2 += nn_dists[i];

					// update pt2pl rms
					prms2 += planar_dist;
				}

			}
		}

		// Compute RMS
		results.all_rms.push_back(sqrt(rms2 / (float)filtered_sample_inds.size()));
		results.all_plane_rms.push_back(sqrt(prms2 / (float)filtered_sample_inds.size()));

		t[3] = std::clock();


		//////////////////
		// Optimization //
		//////////////////

		// Minimize error
		vector<bool> is_ground;
		if (params.ground_w > 0)
		{
			is_ground.reserve(filtered_sample_inds.size());
			for (size_t i = 0; i < filtered_sample_inds.size(); i++)
				is_ground.push_back(tgt_w[filtered_sample_inds[i].first] > params.ground_w);
		}
		PointToPlaneErrorMinimizer(aligned,
								   map.cloud.pts,
								   map.normals,
								   map.scores,
								   filtered_sample_inds,
								   H_icp,
								   is_ground,
								   params.ground_z);

		t[4] = std::clock();


		//////////////////////////////////////
		// Alignment with Motion distortion //
		//////////////////////////////////////

		// Apply the incremental transformation found by ICP
		results.transform = H_icp * results.transform;

		// Align targets taking motion distortion into account
		
		// debug Deep copies
		// aligned_dist = aligned;
		// Eigen::Map<Eigen::Matrix<float, 3, Eigen::Dynamic>> aligned_mat_dist((float*)aligned_dist.data(), 3, N);
		if (params.motion_distortion)
		{
			size_t i_inds = 0;	
			for (auto& t : tgt_t)	
			{	
				Eigen::Matrix4d H_rect = pose_interp(t, params.last_transform0, results.transform, 0);			
				Eigen::Matrix3f R_rect = (H_rect.block(0, 0, 3, 3)).cast<float>();	
				Eigen::Vector3f T_rect = (H_rect.block(0, 3, 3, 1)).cast<float>();	
				aligned_mat.col(i_inds) = (R_rect * targets_mat.col(i_inds)) + T_rect;	
				i_inds++;	
			}
		}
		else
		{
			Eigen::Matrix3f R_tot = (results.transform.block(0, 0, 3, 3)).cast<float>();
			Eigen::Vector3f T_tot = (results.transform.block(0, 3, 3, 1)).cast<float>();
			aligned_mat = (R_tot * targets_mat).colwise() + T_tot;
		}

		t[5] = std::clock();


		// Update all result matrices
		if (step == 0)
			results.all_transforms = Eigen::MatrixXd(results.transform);
		else
		{
			Eigen::MatrixXd temp(results.all_transforms.rows() + 4, 4);
			temp.topRows(results.all_transforms.rows()) = results.all_transforms;
			temp.bottomRows(4) = Eigen::MatrixXd(results.transform);
			results.all_transforms = temp;
		}

		t[5] = std::clock();

		///////////////////////
		// Check convergence //
		///////////////////////

		// Update variations
		if (!stop_cond && step > 0)
		{
			float avg_tot = (float)params.avg_steps;
			if (step == 1)
				avg_tot = 1.0;

			if (step > 0)
			{
				// Get last transformation variations
				Eigen::Matrix3d R2 = results.all_transforms.block(results.all_transforms.rows() - 4, 0, 3, 3);
				Eigen::Matrix3d R1 = results.all_transforms.block(results.all_transforms.rows() - 8, 0, 3, 3);
				Eigen::Vector3d T2 = results.all_transforms.block(results.all_transforms.rows() - 4, 3, 3, 1);
				Eigen::Vector3d T1 = results.all_transforms.block(results.all_transforms.rows() - 8, 3, 3, 1);
				R1 = R2 * R1.transpose();
				T1 = T2 - T1;
				float dT_b = T1.norm();
				float dR_b = acos((R1.trace() - 1) / 2);
				mean_dT += (dT_b - mean_dT) / avg_tot;
				mean_dR += (dR_b - mean_dR) / avg_tot;
			}
		}

		// Stop condition
		if (!stop_cond && step > params.avg_steps)
		{
			if (mean_dT < params.transDiffThresh && mean_dR < params.rotDiffThresh)
			{
				// Do not stop right away. Have a last few averaging steps
				stop_cond = true;
				max_it = step + params.avg_steps;

				// For these last steps, reduce the max distance (half of wall thickness)
				max_planar_d = params.max_planar_dist / 2;
			}
		}

		// Last call, average the last transformations
		if (step > max_it - 2)
		{
			Eigen::Matrix4d mH = Eigen::Matrix4d::Identity(4, 4);
			for (size_t s = 0; s < params.avg_steps; s++)
			{
				Eigen::Matrix4d H = results.all_transforms.block(results.all_transforms.rows() - 4 * (1 + s), 0, 4, 4);
				mH = pose_interp(1.0 / (float)(s + 1), mH, H, 0);
			}
			results.transform = mH;
			results.all_transforms.block(results.all_transforms.rows() - 4, 0, 4, 4) = mH;
		}


		// ///////////// DEBUG /////////////
		// if (step == 0 || step > max_it - 2)
		// {
		// 	string path = "/home/administrator/catkin_ws/src/point_slam/src/test_maps/test_ply/";
		// 	char buffer[100];
		// 	// char buffer_dist[100];
		// 	sprintf(buffer, "f_%03d_step_%03d.ply", (int)count_iter, int(step));
		// 	// sprintf(buffer_dist, "frame_dist_%03d_%03d.ply", (int)count_iter, int(step));
		// 	string filepath = path + string(buffer);
		// 	// string filepath_dist = path + string(buffer_dist);
		// 	save_cloud(filepath, aligned, tgt_t);
		// }
		// ///////////// DEBUG /////////////


	}

	count_iter++;
	

}


