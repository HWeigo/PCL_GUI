#pragma once
#include "PointCloudVisualization.h"


class PointCloudVector
{
private:
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;

	int size = 0;
	int nextStartIdx = 0;
	vector<PointCloudVisualization*> pcvVector;
	vector<bool> isValid;
	vector<pair<int, int>> idxRange;

public:
	PointCloudVector(boost::shared_ptr<pcl::visualization::PCLVisualizer>);
	~PointCloudVector() {};

	int AddPointCloud(PointCloudT::Ptr, string);
	void DeletePointCloud(const int idx);
	bool IsValid(const int idx);
	bool IsShown(const int idx);
	int GetSize();
	int GetCompleteCloudSize();
	pair<int, int> GetRangeInCompleteCloud(const int idx);
	PointCloudT::Ptr GetCloudPtrOfIdx(const int idx);
	PointCloudVisualization* GetPCVofIdx(const int idx);
	string GetId(const int idx);
};

