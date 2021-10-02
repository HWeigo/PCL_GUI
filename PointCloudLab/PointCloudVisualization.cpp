#include "PointCloudVisualization.h"

PointCloudVisualization::PointCloudVisualization(boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_in, \
	PointCloudT::Ptr cloud_in, string id_in) : viewer(viewer_in), cloud(cloud_in), id(id_in) {
	pointSize = cloud->size();
}

void PointCloudVisualization::Show() {
	isShown = true;
	cout << "Display - " << id << endl;
	SetColor(255, 255, 255);
	SetPointSize(3);

	viewer->addPointCloud(cloud, id);
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pointSize, id);

	viewer->updatePointCloud(cloud, id);
}

void PointCloudVisualization::Hide() {
	isShown = false;

	viewer->removePointCloud(id);
}

void PointCloudVisualization::SetColor(int r, int g, int b) {
	red = r;
	green = g;
	blue = b;

	for (int i = 0; i < cloud->size(); ++i) {
		cloud->points[i].r = red;
		cloud->points[i].g = green;
		cloud->points[i].b = blue;
	}

	viewer->updatePointCloud(cloud, id);
}

void PointCloudVisualization::SetPointSize(int sz) {
	pointSize = sz;
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pointSize, id);

	viewer->updatePointCloud(cloud, id);
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

int PointCloudVisualization::GetCloudSize() {
	return cloudSize;
}



