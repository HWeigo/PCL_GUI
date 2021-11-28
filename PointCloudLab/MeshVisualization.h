#pragma once
#include <pcl/PolygonMesh.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "BaseVisualization.h"
class MeshVisualization :
	public BaseVisualization
{
private:
	// Pointer pointing to current point cloud 
	MeshT::Ptr meshPtr;
	
	// Mesh color
	int red = 255;
	int green = 255;
	int blue = 255;

public:
	MeshVisualization(boost::shared_ptr<pcl::visualization::PCLVisualizer>, pcl::PolygonMesh::Ptr, string);
	~MeshVisualization();

	// Show mesh
	void Show();

	// Hide mesh
	void Hide();
	
	// Save mesh to filepath (default)
	void Save(string filepath);

	// Save mesh to filepath as specific type
	// Support type: ".ply"/".obj"/".stl"
	void Save(string filepath, string type);
	
	// Delete point cloud (release memory)
	void Delete();

	// Return face number
	int GetFaceNum();

	// Return point(vertex) number
	int GetPointNum();
	
	// Return the pointer pointing to current mesh (meshPtr)
	MeshT::Ptr GetMeshPtr();

	// Set mesh color to red = r, green = g, blue = b
	void SetColor(const int r, const int g, const int b);

	// Return mesh's color
	vector<int> GetColor();
	
	// Used to indicate current visualization if for mesh (MESH_TYPE)
	inline string GetType() {return MESH_TYPE;};
};

