#pragma once
#include "PointCloudVisualization.h"

class PointCloudVector
{
private:
	int size = 0;
	vector<PointCloudVisualization> pcvVector;
	vector<bool> isDeleted;
	vector<pair<int, int>> idxRange;

public:
	PointCloudVector();
	~PointCloudVector();

	void AddPointCloud();

	
};

