#pragma once

#include <librealsense2/rs.hpp> 
#include <algorithm>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
//#include <pcl/visualization/cloud_viewer.h>
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

pcl::PointCloud<pcl::PointXYZ>::Ptr points_to_pcl(const rs2::points& points)
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

		auto pcl_points = points_to_pcl(points);

		pcl_ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
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


pcl::PointCloud<pcl::PointNormal>::Ptr points_to_pcl_normal(const rs2::points& points)
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

//实时重建测试
int rs_pcl_rebuild_test()
{
	// Declare pointcloud object, for calculating pointclouds and texture mappings
	rs2::pointcloud pc;
	// We want the points object to be persistent so we can display the last cloud when a frame drops
	rs2::points points;
	// Declare RealSense pipeline, encapsulating the actual device and sensors
	rs2::pipeline pipe;
	// Start streaming with default recommended configuration
	pipe.start();

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
	const float leaf = 0.005f;
	grid.setLeafSize(leaf, leaf, leaf);
	//显示窗口
	pcl::visualization::PCLVisualizer viewer("PCL Viewer");
	viewer.setBackgroundColor(0, 0, 0);
	//直通滤波
	pcl::PassThrough<pcl::PointNormal> pass;
	pass.setFilterFieldName("z");
	pass.setFilterLimits(0.0, 1.0);

	//法向量估计
	pcl::NormalEstimation<PointNT, PointNT> nest;
	nest.setRadiusSearch(0.01);
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

	bool isFirst = true;
	while (true) // Application still alive?
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

		auto cloud = points_to_pcl_normal(points);

		//去除远的
		pass.setInputCloud(cloud);
		pass.filter(*cloud);

		// Downsample
		grid.setInputCloud(cloud);
		grid.filter(*cloud);

		// Estimate normals for scene
		nest.setInputCloud(cloud);
		nest.compute(*cloud);

		//特征计算
		fest.setInputCloud(cloud);
		fest.setInputNormals(cloud);
		fest.compute(*cloud_features);

		if (isFirst) {
			isFirst = false;
			*scene += *cloud;
			continue;
		}

		fest.setInputCloud(scene);
		fest.setInputNormals(scene);
		fest.compute(*scene_features);

		//Perform alignment
		align.setInputSource(cloud);
		align.setSourceFeatures(cloud_features);
		align.setInputTarget(scene);
		align.setTargetFeatures(scene_features);
		{
			//pcl::ScopeTime t("Alignment");
			align.align(*cloud);
		}

		if (align.hasConverged())
		{
			//拼接
			*scene += *cloud;
		}
	}
	return 0;
}


