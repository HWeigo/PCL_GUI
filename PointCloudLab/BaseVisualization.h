#pragma once

// Qt
#include <QMainWindow>

// Point Cloud Library
#include <pcl/io/pcd_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/point_cloud.h>
#include <pcl/PolygonMesh.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

// Visualization Toolkit (VTK)
#include <vtkRenderWindow.h>

#include <vector>
#include <unordered_set>

using namespace std;

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::PolygonMesh MeshT;

// Used to indicate the type of the derive class (Point Cloud/ Mesh)
#define POINTCLOUD_TYPE "pointcloud_"
#define MESH_TYPE "mesh_"

class BaseVisualization
{
protected:
	// Unique ID needs to be assigned to different element
	string id;

	// Flag indicate if the element is shown in viewer
	bool isShown = false;

	// PCL viewer (binded to Qt::qvtkWidget, see void InitVtk())
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;

public:
	// Virtual function: show pointcloud/mesh
	virtual void Show() = 0;

	// Virtual function: hide pointcloud/mesh
	virtual void Hide() = 0;
	
	// Virtual function: save pointcloud/mesh
	virtual void Save(string filepath) = 0;

	// Virtual function: save pointcloud/mesh as specific type
	virtual void Save(string filepath, string type) = 0;
	
	// Virtual function: delete ave pointcloud/mesh
	virtual void Delete() = 0;

	// Virtual function: used to return the type of derive function
	virtual string GetType() = 0;

	// Return current element's id
	inline string GetId() { return id; };

	// Check if current element is shown
	inline bool IsShown() { return isShown; };

	BaseVisualization(boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer, string id);
	~BaseVisualization();
};

