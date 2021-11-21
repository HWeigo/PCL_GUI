#pragma once
#include "PointCloudVisualization.h"
#include "MeshVisualization.h"


class PointCloudVector
{
private:
	// PCL viewer (binded to Qt::qvtkWidget, see void InitVtk())
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;

	// Vector stores all the visualization class (both pointcloud type and mesh type)
	vector<BaseVisualization*> visualVector;

	// Vector indicates whether an entity is vaild (not deleted)
	// Note: if an entity is deleted, it will not be removed from visualVector
	// but clear all the memory and set flase flag in isValid (see Delete Entity)
	vector<bool> isValid;

	// Size of visualVector
	// Always has size == visualVector.size() == isValid.size()
	int size = 0;

	// --- NOT USED ---
	int nextStartIdx = 0;
	vector<pair<int, int>> idxRange;

public:
	// Poinc cloud vector's constructor
	PointCloudVector(boost::shared_ptr<pcl::visualization::PCLVisualizer>);
	~PointCloudVector() {};

	// Add point cloud into visualVector (An unique id needs to be assigned)
	int AddPointCloud(PointCloudT::Ptr cloudPtr, string id);

	// Add mesh into visualVector (An unique id needs to be assigned)
	int AddMesh(MeshT::Ptr meshPtr, string id);

	// Delete entity
	void DeleteEntity(const int idx);

	// Check if idx th element is valid (deleted or not)
	bool IsValid(const int idx);
	
	// Check if idx th element is shown 
	bool IsShown(const int idx);

	// Return the size of visualVector
	int GetSize() const;

	// Return the type(string) of the idx th element
	// POINTCLOUD_TYPE: pointcloud, MESH_TYPE: mesh (Defined in BaseVisualization.h)
	string GetType(const int idx);

	// Return a point cloud pointer of the idx th element
	// (if the idx th element is not point cloud, a null pointer(PointCloudT::Ptr()) is returned) 
	PointCloudT::Ptr GetCloudPtrOfIdx(const int idx);
	
	// Return a mesh pointer of the idx th element
	// (if the idx th element is not mesh, a null pointer(MeshT::Ptr()) is returned) 
	MeshT::Ptr GetMeshPtrOfIdx(const int idx);

	// Return the pointcloud visualization class pointer of the idx th element
	// (if the idx th element is not point cloud, a null pointer(PointCloudT::Ptr()) is returned) 
	PointCloudVisualization* GetPCVofIdx(const int idx);

	// Return the mesh visualization class pointer of the idx the element
	MeshVisualization* GetMESHVofIdx(const int idx);

	// Return the ID of the idx the element
	string GetId(const int idx);

	// Save the idx th point cloud to filepath
	void SavePointCloudOfIdx(string filepath, const int idx);

	// Save the idx th mesh to filepath
	void SaveMeshOfIdx(string filepath, const int idx);

	// --- NOT USED ---
	int GetCompleteCloudSize();
	pair<int, int> GetRangeInCompleteCloud(const int idx);

};

