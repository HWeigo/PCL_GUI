#include "PointCloudVector.h"



PointCloudVector::PointCloudVector(boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_in) :\
viewer(viewer_in)
{
}

int PointCloudVector::AddPointCloud(PointCloudT::Ptr cloud, string id) {
	PointCloudVisualization* pcv = new PointCloudVisualization(viewer, cloud, id);
	pcvVector.push_back(pcv);
	isValid.push_back(true);
	idxRange.push_back({ nextStartIdx, nextStartIdx + pcv->GetCloudSize() - 1 });
	nextStartIdx = nextStartIdx + pcv->GetCloudSize();
	size = pcvVector.size();
	assert((size == isValid.size()) && (size == idxRange.size()));
	return size - 1;
}

void PointCloudVector::DeletePointCloud(const int idx) {
	// Todo: release memory!!
	if (idx >= size)
		return;
	
	// Reset size
	pcvVector[idx]->Delete();
	isValid[idx] = false;
}

void PointCloudVector::SavePointCloudOfIdx(string filepath, const int idx) {
	//assert(idx >= size || idx < 0);
	assert(idx >= 0 && idx < size);

	PointCloudVisualization* pcv = pcvVector[idx];
	pcv->Save(filepath);
}

bool PointCloudVector::IsValid(const int idx) {
	return (idx >= 0) && (idx < size) && isValid[idx];
}

bool PointCloudVector::IsShown(const int idx) {
	return (idx >= 0) && (idx < size) && pcvVector[idx]->IsShown();
}

int PointCloudVector::GetSize() {
	return size;
}

int PointCloudVector::GetCompleteCloudSize() {
	// Todo: ERROR
	return nextStartIdx;
}

pair<int, int> PointCloudVector::GetRangeInCompleteCloud(const int idx) {
	assert(idx >= 0 && idx < size);
	return idxRange[idx];
}

PointCloudT::Ptr PointCloudVector::GetCloudPtrOfIdx(const int idx) {
	assert(idx >= 0 && idx < size);
	return pcvVector[idx]->GetCloudPtr();
}

PointCloudVisualization* PointCloudVector::GetPCVofIdx(const int idx) {
	assert(idx >= 0 && idx < size);
	return pcvVector[idx];
}

string PointCloudVector::GetId(const int idx) {
	assert(idx >= 0 && idx < size);
	return pcvVector[idx]->GetId();
}