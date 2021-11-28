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
#include <unordered_set>

#include "BaseVisualization.h"

//typedef pcl::PointXYZRGBA PointT;
//typedef pcl::PointCloud<PointT> PointCloudT;

class PointCloudVisualization : 
	public BaseVisualization
{
private:
	// Pointer pointing to current point cloud 
	PointCloudT::Ptr cloudPtr;

	// Point number of point cloud, pointNum == cloudPtr->size()
	int pointNum;

	// Point cloud point size
	int pointSize = 3;

	// Point cloud color
	int red = 255;
	int green = 255;
	int blue = 255;

	// --- NOT USED --- 
	int frontIdx;
	int backIdx;

public:
	PointCloudVisualization(boost::shared_ptr<pcl::visualization::PCLVisualizer>, PointCloudT::Ptr, string);
	~PointCloudVisualization() {};

	// Show pointcloud
	void Show();

	// Hide pointcloud
	void Hide();

	// Save pointcloud to filepath (default)
	void Save(string filepath);

	// Save pointcloud to filepath as specific type
	void Save(string filepath, string type);

	// Delete point cloud (release memory)
	void Delete();

	// Return number of points in this point cloud
	int GetPointNum();

	// Return the pointer pointing to current point cloud (pointPtr)
	PointCloudT::Ptr GetCloudPtr();

	// Set point cloud color to red = r, green = g, blue = b
	void SetColor(const int r, const int g, const int b);

	// Return point cloud's color
	vector<int> GetColor();

	// Set point cloud point size to sz
	void SetPointSize(const int sz);

	// Return point cloud's point size
	int GetPointSize();

	// Add newCloud's points into current point cloud
	void AddCloud(PointCloudT::Ptr newCloud);

	// Delete points 
	void DeletePointFromVector(const unordered_set<int> &setSelected);

	// Used to indicate current visualization if for point cloud (POINTCLOUD_TYPE)
	inline string GetType() {return POINTCLOUD_TYPE;};

};