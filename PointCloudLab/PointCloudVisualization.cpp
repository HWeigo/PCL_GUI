#include "PointCloudVisualization.h"

PointCloudVisualization::PointCloudVisualization(boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_in, \
	PointCloudT::Ptr cloud_in, string id_in) : BaseVisualization(viewer_in, id_in), cloudPtr(cloud_in) {
	
	cloudPtr->height = (cloudPtr->height == 0) ? 1 :cloudPtr->height;
	cloudPtr->width = (cloudPtr->width == 0) ? cloudPtr->size() : cloudPtr->width;

	pointNum = cloudPtr->size();
}

void PointCloudVisualization::Show() {
	isShown = true;
	cout << "Display - " << id << endl;
	if (cloudPtr->points[10].r == 0 && cloudPtr->points[10].g == 0 && cloudPtr->points[10].b == 0)
	{
		SetColor(red, green, blue);
	}
	
	SetPointSize(pointSize);
	viewer->removePointCloud(id);
	viewer->addPointCloud(cloudPtr, id);
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pointSize, id);

	viewer->updatePointCloud(cloudPtr, id);
}

void PointCloudVisualization::Hide() {
	isShown = false;
	cout << "Hide - " << id << endl;


	viewer->removePointCloud(id);
}

void PointCloudVisualization::Delete() {
	PointCloudT().swap(*cloudPtr);
	viewer->removePointCloud(id);

	cout << cloudPtr->size() << endl;
	isShown = false;
}

void PointCloudVisualization::AddCloud(PointCloudT::Ptr newCloud) {
	*cloudPtr += *newCloud;
	pointNum = cloudPtr->size();
}

void PointCloudVisualization::DeletePointFromVector(const unordered_set<int> &setSelected) {
	PointCloudT::Ptr tempCloud(new PointCloudT());
	for (int i = 0; i < pointNum; ++i) {
		if (setSelected.count(i) == 0) {
			tempCloud->push_back(cloudPtr->points.at(i));
		}
	}
	cloudPtr.swap(tempCloud);
	Show();
	pointNum = cloudPtr->size();
}


void PointCloudVisualization::SetColor(const int r, const int g, const int b) {
	red = r;
	green = g;
	blue = b;

	for (int i = 0; i < cloudPtr->size(); ++i) {
		cloudPtr->points[i].r = red;
		cloudPtr->points[i].g = green;
		cloudPtr->points[i].b = blue;
	}

	viewer->updatePointCloud(cloudPtr, id);
}

void PointCloudVisualization::SetPointSize(const int sz) {
	pointSize = sz;
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pointSize, id);

	viewer->updatePointCloud(cloudPtr, id);
}

void PointCloudVisualization::Save(string filepath) {
	pcl::PCDWriter writer;
	writer.write(filepath, *cloudPtr);
}

vector<int> PointCloudVisualization::GetColor() {
	vector<int> color(3);
	color[0] = red;
	color[1] = green;
	color[2] = blue;

	return color;
}

int PointCloudVisualization::GetPointSize() {
	return pointSize;
}

int PointCloudVisualization::GetPointNum() {
	pointNum = cloudPtr->size();
	return pointNum;
}

PointCloudT::Ptr PointCloudVisualization::GetCloudPtr() {
	return cloudPtr;
}




