#include "PointCloudVector.h"



PointCloudVector::PointCloudVector(boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_in) :\
viewer(viewer_in)
{
}

int PointCloudVector::AddPointCloud(PointCloudT::Ptr cloudPtr, string id) {
	PointCloudVisualization* pcv = new PointCloudVisualization(viewer, cloudPtr, id);
	visualVector.push_back(pcv);
	isValid.push_back(true);
	idxRange.push_back({ nextStartIdx, nextStartIdx + pcv->GetPointNum() - 1 });
	nextStartIdx = nextStartIdx + pcv->GetPointNum();
	size = visualVector.size();
	assert((size == isValid.size()) && (size == idxRange.size()));
	return size - 1;
}

int PointCloudVector::AddMesh(pcl::PolygonMesh::Ptr meshPtr, string id) {
	MeshVisualization* meshv = new MeshVisualization(viewer, meshPtr, id);
	visualVector.push_back(meshv);
	isValid.push_back(true);
	//idxRange.push_back({ nextStartIdx, nextStartIdx + pcv->GetCloudSize() - 1 });
	//nextStartIdx = nextStartIdx + pcv->GetCloudSize();
	size = visualVector.size();
	assert(size == isValid.size());
	return size - 1;
}

void PointCloudVector::DeleteEntity(const int idx) {
	// Todo: release memory!!
	if (idx >= size)
		return;
	
	// Reset size
	visualVector[idx]->Delete();
	isValid[idx] = false;
}

void PointCloudVector::SavePointCloudOfIdx(string filepath, const int idx) {
	assert(idx >= 0 && idx < size);
	if (visualVector[idx]->GetType() != POINTCLOUD_TYPE)
		return;
	PointCloudVisualization* pcv = (PointCloudVisualization*)visualVector[idx];
	pcv->Save(filepath);
}

void PointCloudVector::SaveMeshOfIdx(string filepath, const int idx)
{
	assert(idx >= 0 && idx < size);
	if (visualVector[idx]->GetType() != MESH_TYPE)
		return;
	MeshVisualization* meshv = (MeshVisualization*)visualVector[idx];
	meshv->Save(filepath);
}

bool PointCloudVector::IsValid(const int idx) {
	return (idx >= 0) && (idx < size) && isValid[idx];
}

bool PointCloudVector::IsShown(const int idx) {
	return (idx >= 0) && (idx < size) && visualVector[idx]->IsShown();
}

int PointCloudVector::GetSize() const {
	return size;
}

int PointCloudVector::GetCompleteCloudSize() {
	// Todo: ERROR
	return nextStartIdx;
}

string PointCloudVector::GetType(const int idx)
{
	assert(idx >= 0 && idx < size);
	return visualVector[idx]->GetType();
}

pair<int, int> PointCloudVector::GetRangeInCompleteCloud(const int idx) {
	assert(idx >= 0 && idx < size);
	return idxRange[idx];
}

PointCloudT::Ptr PointCloudVector::GetCloudPtrOfIdx(const int idx) {
	assert(idx >= 0 && idx < size);
	if (visualVector[idx]->GetType() != POINTCLOUD_TYPE)
		return PointCloudT::Ptr();
	//return PointCloudT::Ptr(pcvVector[idx]->GetEntityPtr());
	return ((PointCloudVisualization*)(visualVector[idx]))->GetCloudPtr();
}

MeshT::Ptr PointCloudVector::GetMeshPtrOfIdx(const int idx)
{
	assert(idx >= 0 && idx < size);
	if (visualVector[idx]->GetType() != MESH_TYPE)
		return MeshT::Ptr();
	return ((MeshVisualization*)(visualVector[idx]))->GetMeshPtr();
}

PointCloudVisualization* PointCloudVector::GetPCVofIdx(const int idx) {
	assert(idx >= 0 && idx < size);
	return (PointCloudVisualization*)visualVector[idx];
}

MeshVisualization * PointCloudVector::GetMESHVofIdx(const int idx)
{
	assert(idx >= 0 && idx < size);
	return (MeshVisualization*)visualVector[idx];
}

string PointCloudVector::GetId(const int idx) {
	assert(idx >= 0 && idx < size);
	return visualVector[idx]->GetId();
}