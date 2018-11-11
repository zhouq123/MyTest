#pragma once
#include "stdafx.h"
#include "librealsense2/rs.hpp"
#include <algorithm>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <Eigen/Core>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/time.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/statistical_outlier_removal.h>


// Align a rigid object to a scene with clutter and occlusions
int my_pcl_registration_test(const std::string& objectPath, const  std::string& scenePath)
{
	// Types
	typedef pcl::PointNormal PointNT;
	typedef pcl::PointCloud<PointNT> PointCloudT;
	typedef pcl::FPFHSignature33 FeatureT;
	typedef pcl::FPFHEstimation<PointNT, PointNT, FeatureT> FeatureEstimationT;
	typedef pcl::PointCloud<FeatureT> FeatureCloudT;
	typedef pcl::visualization::PointCloudColorHandlerCustom<PointNT> ColorHandlerT;
	// Point clouds
	PointCloudT::Ptr object(new PointCloudT);
	PointCloudT::Ptr object_aligned(new PointCloudT);
	PointCloudT::Ptr scene(new PointCloudT);
	FeatureCloudT::Ptr object_features(new FeatureCloudT);
	FeatureCloudT::Ptr scene_features(new FeatureCloudT);

	// Load object and scene
	if (pcl::io::loadPCDFile<PointNT>(objectPath, *object) < 0 ||
		pcl::io::loadPCDFile<PointNT>(scenePath, *scene) < 0)
	{
		std::cout << "Error loading object/scene file!" << std::endl;
		return -1;
	}

	// Downsample
	std::cout << "Downsampling..." << std::endl;
	pcl::VoxelGrid<PointNT> grid;
	const float leaf = 0.005f;
	grid.setLeafSize(leaf, leaf, leaf);
	grid.setInputCloud(object);
	grid.filter(*object);
	grid.setInputCloud(scene);
	grid.filter(*scene);

	// Estimate normals for scene
	std::cout << "Estimating scene normals..." << std::endl;
	pcl::NormalEstimation<PointNT, PointNT> nest;
	nest.setRadiusSearch(0.01);
	nest.setInputCloud(scene);
	nest.compute(*scene);

	//此处需要添加法向量计算
	nest.setInputCloud(object);
	nest.compute(*object);

	// Estimate features
	std::cout << "Estimating features..." << std::endl;
	FeatureEstimationT fest;
	fest.setRadiusSearch(0.025);
	fest.setInputCloud(object);
	fest.setInputNormals(object);
	fest.compute(*object_features);
	fest.setInputCloud(scene);
	fest.setInputNormals(scene);
	fest.compute(*scene_features);

	// Perform alignment
	std::cout << "Starting alignment...\n" << std::endl;
	pcl::SampleConsensusPrerejective<PointNT, PointNT, FeatureT> align;
	align.setInputSource(object);
	align.setSourceFeatures(object_features);
	align.setInputTarget(scene);
	align.setTargetFeatures(scene_features);
	align.setMaximumIterations(50000); // Number of RANSAC iterations
	align.setNumberOfSamples(3); // Number of points to sample for generating/prerejecting a pose
	align.setCorrespondenceRandomness(5); // Number of nearest features to use
	align.setSimilarityThreshold(0.9f); // Polygonal edge length similarity threshold
	align.setMaxCorrespondenceDistance(2.5f * leaf); // Inlier threshold
	align.setInlierFraction(0.25f); // Required inlier fraction for accepting a pose hypothesis
	{
		pcl::ScopeTime t("Alignment");
		align.align(*object_aligned);
	}

	if (align.hasConverged())
	{
		// Print results
		std::cout << std::endl;
		Eigen::Matrix4f transformation = align.getFinalTransformation();
		std::cout << "    | %6.3f %6.3f %6.3f | \n" << transformation(0, 0) << transformation(0, 1) << transformation(0, 2) << std::endl;
		std::cout << " R = | %6.3f %6.3f %6.3f | \n" << transformation(1, 0) << transformation(1, 1) << transformation(1, 2) << std::endl;
		std::cout << "     | %6.3f %6.3f %6.3f | \n" << transformation(2, 0) << transformation(2, 1) << transformation(2, 2) << std::endl;
		std::cout << " \n" << std::endl;
		std::cout << " t = < %0.3f, %0.3f, %0.3f >\n" << transformation(0, 3) << transformation(1, 3) << transformation(2, 3) << std::endl;
		std::cout << " \n" << std::endl;
		std::cout << " Inliers: %i/%i\n" << align.getInliers().size() << object->size() << std::endl;

		//pcl::transformPointCloud(*object, *object_aligned, transformation);
		//*scene += *object_aligned;

		// Show alignment
		pcl::visualization::PCLVisualizer visu("Alignment");
		visu.addPointCloud(scene, ColorHandlerT(scene, 0.0, 255.0, 0.0), "scene");
		visu.addPointCloud(object_aligned, ColorHandlerT(object_aligned, 0.0, 0.0, 255.0), "object_aligned");
		visu.spin();
	}
	else
	{
		std::cout << "Alignment failed!" << std::endl;
		return -1;
	}

	return (0);
}

pcl::PointCloud<pcl::PointXYZ>::Ptr my_points_to_pcl(const rs2::points& points)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	auto sp = points.get_profile().as<rs2::video_stream_profile>();
	cloud->width = sp.width();
	cloud->height = sp.height();
	cloud->is_dense = false;
	cloud->points.resize(points.size());
	auto ptr = points.get_vertices();
	for (auto& p : cloud->points)
	{
		p.x = ptr->x;
		p.y = ptr->y;
		p.z = ptr->z;
		ptr++;
	}
	return cloud;
}
//realsense读取转换到pcl并显示测试
int rs_pcl_show_test()
{
	// Declare pointcloud object, for calculating pointclouds and texture mappings
	rs2::pointcloud pc;
	// We want the points object to be persistent so we can display the last cloud when a frame drops
	rs2::points points;
	// Declare RealSense pipeline, encapsulating the actual device and sensors
	rs2::pipeline pipe;
	// Start streaming with default recommended configuration
	pipe.start();

	pcl::visualization::PCLVisualizer viewer("PCL Viewer");
	viewer.setBackgroundColor(0, 0, 0);
	bool isFirst = true;
	while (true) // Application still alive?
	{
		// Wait for the next set of frames from the camera
		auto frames = pipe.wait_for_frames();

		auto depth = frames.get_depth_frame();

		// Generate the pointcloud and texture mappings
		points = pc.calculate(depth);

		auto color = frames.get_color_frame();

		// For cameras that don't have RGB sensor, we'll map the pointcloud to infrared instead of color
		if (!color)
			color = frames.get_infrared_frame();

		// Tell pointcloud object to map to this color frame
		pc.map_to(color);

		auto pcl_points = my_points_to_pcl(points);

		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PassThrough<pcl::PointXYZ> pass;
		pass.setInputCloud(pcl_points);
		pass.setFilterFieldName("z");
		pass.setFilterLimits(0.0, 1.0);
		pass.filter(*cloud_filtered);

		//viewer.removeAllPointClouds();
		if (isFirst) {
			viewer.addPointCloud<pcl::PointXYZ>(cloud_filtered);
			isFirst = false;
		}
		else {
			viewer.updatePointCloud(cloud_filtered);
		}
		viewer.spinOnce();
	}
	return 1;
}


pcl::PointCloud<pcl::PointNormal>::Ptr my_points_to_pcl_normal(const rs2::points& points)
{
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud(new pcl::PointCloud<pcl::PointNormal>);
	auto sp = points.get_profile().as<rs2::video_stream_profile>();
	cloud->width = sp.width();
	cloud->height = sp.height();
	cloud->is_dense = false;
	cloud->points.resize(points.size());
	auto ptr = points.get_vertices();
	for (auto& p : cloud->points)
	{
		p.x = ptr->x;
		p.y = ptr->y;
		p.z = ptr->z;
		ptr++;
	}
	return cloud;
}

//double normofTransform(cv::Mat rvec, cv::Mat tvec)
//{
//	return fabs(min(cv::norm(rvec), 2 * M_PI - cv::norm(rvec))) + fabs(cv::norm(tvec));
//}

//实时重建测试
int my_rs_pcl_rebuild_test(int test_count)
{
	Sleep(3000);

	const int COLOR_WIDTH = 640;
	const int COLOR_HEIGHT = 480;
	const int COLOR_FPS = 30;
	const int DEPTH_WIDTH = 640;
	const int DEPTH_HEIGHT = 480;
	const int DEPTH_FPS = 30;
	rs2::config cfg;
	cfg.enable_stream(RS2_STREAM_COLOR, COLOR_WIDTH, COLOR_HEIGHT, RS2_FORMAT_RGB8, COLOR_FPS);
	cfg.enable_stream(RS2_STREAM_DEPTH, DEPTH_WIDTH, DEPTH_HEIGHT, RS2_FORMAT_Z16, DEPTH_FPS);
	// Declare pointcloud object, for calculating pointclouds and texture mappings
	rs2::pointcloud pc;
	// We want the points object to be persistent so we can display the last cloud when a frame drops
	rs2::points points;
	// Declare RealSense pipeline, encapsulating the actual device and sensors
	rs2::pipeline pipe;
	// Start streaming with default recommended configuration
	pipe.start(cfg);

	// Types
	typedef pcl::PointNormal PointNT;
	typedef pcl::PointCloud<PointNT> PointCloudT;
	typedef pcl::FPFHSignature33 FeatureT;
	typedef pcl::FPFHEstimation<PointNT, PointNT, FeatureT> FeatureEstimationT;
	typedef pcl::PointCloud<FeatureT> FeatureCloudT;
	typedef pcl::visualization::PointCloudColorHandlerCustom<PointNT> ColorHandlerT;

	PointCloudT::Ptr scene(new PointCloudT);
	FeatureCloudT::Ptr cloud_features(new FeatureCloudT);
	FeatureCloudT::Ptr scene_features(new FeatureCloudT);

	//体素滤波
	pcl::VoxelGrid<PointNT> grid;
	const float leaf = 0.02f;
	grid.setLeafSize(leaf, leaf, leaf);

	//直通滤波
	pcl::PassThrough<pcl::PointNormal> pass;
	pass.setFilterFieldName("z");
	pass.setFilterLimits(0.0, 1.0);

	pcl::StatisticalOutlierRemoval<pcl::PointNormal> sor;
	sor.setMeanK(20);
	sor.setStddevMulThresh(1.0);

	//法向量估计
	pcl::NormalEstimation<PointNT, PointNT> nest;
	nest.setRadiusSearch(0.02);
	//特征计算
	FeatureEstimationT fest;
	fest.setRadiusSearch(0.025);

	//拼接
	pcl::SampleConsensusPrerejective<PointNT, PointNT, FeatureT> align;
	align.setMaximumIterations(50000); // Number of RANSAC iterations
	align.setNumberOfSamples(3); // Number of points to sample for generating/prerejecting a pose
	align.setCorrespondenceRandomness(5); // Number of nearest features to use
	align.setSimilarityThreshold(0.9f); // Polygonal edge length similarity threshold
	align.setMaxCorrespondenceDistance(2.5f * leaf); // Inlier threshold
	align.setInlierFraction(0.25f); // Required inlier fraction for accepting a pose hypothesis

	pcl::IterativeClosestPoint<pcl::PointNormal, pcl::PointNormal> icp;
	icp.setMaximumIterations(50000);

	bool isFirst = true;
	int count = 0;
	while (count < test_count) // Application still alive?
	{
		// Wait for the next set of frames from the camera
		auto frames = pipe.wait_for_frames();

		auto depth = frames.get_depth_frame();

		// Generate the pointcloud and texture mappings
		points = pc.calculate(depth);

		//auto color = frames.get_color_frame();

		//// For cameras that don't have RGB sensor, we'll map the pointcloud to infrared instead of color
		//if (!color)
		//	color = frames.get_infrared_frame();

		//// Tell pointcloud object to map to this color frame
		//pc.map_to(color);

		auto cloud = my_points_to_pcl_normal(points);

		std::cout << "PassThrough" << std::endl;
		//去除远的
		pass.setInputCloud(cloud);
		pass.filter(*cloud);

		//std::cout << "StatisticalOutlierRemoval" << std::endl;
		//sor.setInputCloud(cloud);
		//sor.filter(*cloud);

		std::cout << "Downsample" << std::endl;
		// Downsample
		grid.setInputCloud(cloud);
		grid.filter(*cloud);



		std::cout << "Estimate normals" << std::endl;
		// Estimate normals for scene
		nest.setInputCloud(cloud);
		nest.compute(*cloud);

		if (isFirst) {
			isFirst = false;
			*scene += *cloud;
			count++;
			continue;
		}
		nest.setInputCloud(scene);
		nest.compute(*scene);

		std::cout << "Estimate features" << std::endl;
		//特征计算
		fest.setInputCloud(cloud);
		fest.setInputNormals(cloud);
		fest.compute(*cloud_features);

		fest.setInputCloud(scene);
		fest.setInputNormals(scene);
		fest.compute(*scene_features);

		std::cout << "alignment" << std::endl;

		if (true) {
			//使用icp
			icp.setInputSource(cloud);
			icp.setInputTarget(scene);
			icp.align(*cloud);

			if (icp.hasConverged())
			{
				double score = icp.getFitnessScore();

				if (score < 0.02) {
					*scene += *cloud;
					// Downsample
					grid.setInputCloud(scene);
					grid.filter(*scene);
				}
				else {
					std::cout << std::to_string(score) << std::endl;
				}
			}
			else
			{
				std::cout << "no converged" << std::endl;
				return (-1);
			}

		}
		else {
			////使用SampleConsensusPrerejective
			align.setInputSource(cloud);
			align.setSourceFeatures(cloud_features);
			align.setInputTarget(scene);
			align.setTargetFeatures(scene_features);
			align.align(*cloud);
			if (align.hasConverged())
			{
				//拼接
				*scene += *cloud;
				// Downsample
				grid.setInputCloud(scene);
				grid.filter(*scene);
			}
			else {
				std::cout << "no converged" << std::endl;
			}
		}

		count++;
	}
	//显示窗口
	pcl::visualization::PCLVisualizer viewer("PCL Viewer");
	viewer.setBackgroundColor(0, 0, 0);
	viewer.addPointCloud(scene, ColorHandlerT(scene, 0.0, 0.0, 255.0), "scene");

	viewer.spin();
	return 0;
}