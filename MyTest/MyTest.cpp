#pragma once
#include "stdafx.h"
#include <iostream>
#include <vector>
#include <Eigen/Eigen>
#include <opencv2/opencv.hpp>
#include "rs_pcl_test.h"
#include "Config.h"

using namespace std;

//从pts2到pts1的R T
void pose_estimation_3d3d(
	const vector<cv::Point3f>& pts1,
	const vector<cv::Point3f>& pts2,
	Eigen::Matrix3d& R, Eigen::Vector3d& t
)
{
	cv::Point3f p1, p2;     // center of mass
	int N = pts1.size();
	for (int i = 0; i<N; i++)
	{
		p1 += pts1[i];
		p2 += pts2[i];
	}
	p1 = cv::Point3f(cv::Vec3f(p1) / N);
	p2 = cv::Point3f(cv::Vec3f(p2) / N);
	vector<cv::Point3f>  q1(N), q2(N); // remove the center
	for (int i = 0; i<N; i++)
	{
		q1[i] = pts1[i] - p1;
		q2[i] = pts2[i] - p2;
	}

	// compute q1*q2^T
	Eigen::Matrix3d W = Eigen::Matrix3d::Zero();
	for (int i = 0; i<N; i++)
	{
		W += Eigen::Vector3d(q1[i].x, q1[i].y, q1[i].z) * Eigen::Vector3d(q2[i].x, q2[i].y, q2[i].z).transpose();
	}
	// SVD on W
	Eigen::JacobiSVD<Eigen::Matrix3d> svd(W, Eigen::ComputeFullU | Eigen::ComputeFullV);
	Eigen::Matrix3d U = svd.matrixU();
	Eigen::Matrix3d V = svd.matrixV();

	R = U * (V.transpose());
	t = Eigen::Vector3d(p1.x, p1.y, p1.z) - R * Eigen::Vector3d(p2.x, p2.y, p2.z);
}

//测试eigen的svd分解实现icp
int pose_estimation_3d3d_test()
{
	vector<cv::Point3f> pts1;
	vector<cv::Point3f> pts2;
	for (size_t i = 0; i < 20; ++i)
	{
		pts1.push_back(cv::Point3f(1024 * rand() / (RAND_MAX + 1.0f), 1024 * rand() / (RAND_MAX + 1.0f), 1024 * rand() / (RAND_MAX + 1.0f)));
		pts2.push_back(cv::Point3f(0, 0, 0));
	}

	for (size_t i = 0; i < 20; ++i)
	{
		pts2[i].x = pts1[i].x + 1500;
		pts2[i].y = pts1[i].y + 2500;
		pts2[i].z = pts1[i].z + 3500;
	}
	Eigen::Matrix3d r;
	Eigen::Vector3d t;
	pose_estimation_3d3d(pts1, pts2, r, t);
	vector<Eigen::Vector3d> p1;
	for (size_t i = 0; i < 20; ++i)
	{
		p1.push_back(r *Eigen::Vector3d(pts2[i].x, pts2[i].y, pts2[i].z) + t);
	}

	ofstream file1;
	ofstream file2;
	file1.open("test1.xyz");
	file2.open("test2.xyz");

	for (size_t i = 0; i < 20; ++i)
	{
		file1 << to_string(pts1[i].x) << " " << to_string(pts1[i].y) << " " << to_string(pts1[i].z) << " " << endl;
		file1 << to_string(pts2[i].x) << " " << to_string(pts2[i].y) << " " << to_string(pts2[i].z) << " " << endl;
		file2 << to_string(pts2[i].x) << " " << to_string(pts2[i].y) << " " << to_string(pts2[i].z) << " " << endl;
		file2 << to_string(p1[i][0]) << " " << to_string(p1[i][1]) << " " << to_string(p1[i][2]) << " " << endl;
	}

	file1.close();
	file2.close();
	return 0;
}

int main(int argc, char** argv)
{
	map<string, string> m;
	ReadConfig("test.cfg", m);
	int test_count = atoi(m.find("test_count")->second.c_str());
	

	//pose_estimation_3d3d_test();
	//rs_pcl_test();
	//pcl_registration_test("object.pcd","scene.pcd" );
	my_rs_pcl_rebuild_test(test_count);
}
