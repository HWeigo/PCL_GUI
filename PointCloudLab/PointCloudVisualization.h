#pragma once

// Qt
#include <QMainWindow>

// Point Cloud Library
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

// Visualization Toolkit (VTK)
#include <vtkRenderWindow.h>

#include <vector>

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

using namespace std;

class PointCloudVisualization {
private:
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
	PointCloudT::Ptr cloud;
	string id;
	int frontIdx;
	int backIdx;
	int cloudSize;

	int pointSize = 3;
	int red = 255;
	int green = 255;
	int blue = 255;
	bool isShown = true;


public:
	PointCloudVisualization(boost::shared_ptr<pcl::visualization::PCLVisualizer>, PointCloudT::Ptr, string);
	~PointCloudVisualization() {};

	void Show();
	void Hide();
	vector<int> GetColor();
	int GetPointSize();
	int GetCloudSize();

	void SetColor(int r, int g, int b);
	void SetPointSize(int sz);
	bool IsShown() { return isShown; };


};