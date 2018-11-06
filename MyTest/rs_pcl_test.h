#pragma once

#include <librealsense2/rs.hpp> 
#include <algorithm>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/visualization/cloud_viewer.h>

using pcl_ptr = pcl::PointCloud<pcl::PointXYZ>::Ptr;

pcl_ptr points_to_pcl(const rs2::points& points)
{
	pcl_ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
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

int rs_pcl_test()
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
