#include "PointCloudVector.h"



PointCloudVector::PointCloudVector(boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_in) :\
viewer(viewer_in)
{
}

void PointCloudVector::AddPointCloud(PointCloudT::Ptr cloud, string id) {
	PointCloudVisualization* pcv = new PointCloudVisualization(viewer, cloud, id);
	pcvVector.push_back(pcv);
	isValid.push_back(true);
	idxRange.push_back({ nextStartIdx, nextStartIdx + pcv->GetCloudSize() - 1 });
	nextStartIdx = nextStartIdx + pcv->GetCloudSize();
	size = pcvVector.size();
	assert((size == isValid.size()) && (size == idxRange.size()));
}

void PointCloudVector::DeletePointCloud(const int idx) {
	// Todo: release memory!!

	isValid[idx] = false;
}

bool PointCloudVector::IsValid(const int idx) {
	return (idx < size) && isValid[idx];
}

int PointCloudVector::GetCompleteCloudSize() {
	return nextStartIdx;
}

pair<int, int> PointCloudVector::GetRangeInCompleteCloud(const int idx) {
	if (idx >= size)
		return{ -1, -1 };
	return idxRange[idx];
}

PointCloudT::Ptr PointCloudVector::GetCloudPtrOfIdx(const int idx) {
	if (idx >= size)
		return nullptr;
	return pcvVector[idx]->GetCloudPtr();
}

PointCloudVisualization* PointCloudVector::GetPCVofIdx(const int idx) {
	if (idx >= size)
		return nullptr;
	return pcvVector[idx];
}
