#include "PointCloudVisualization.h"

PointCloudVisualization::PointCloudVisualization(boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_in, \
	PointCloudT::Ptr cloud_in, string id_in) : viewer(viewer_in), cloud(cloud_in), id(id_in) {
	cloudSize = cloud->size();
}

void PointCloudVisualization::Show() {
	isShown = true;
	cout << "Display - " << id << endl;
	SetColor(red, green, blue);
	SetPointSize(pointSize);

	viewer->addPointCloud(cloud, id);
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pointSize, id);

	viewer->updatePointCloud(cloud, id);
}

void PointCloudVisualization::Hide() {
	isShown = false;
	cout << "Hide - " << id << endl;


	viewer->removePointCloud(id);
}

void PointCloudVisualization::Delete() {
	PointCloudT().swap(*cloud);
	viewer->removePointCloud(id);

	cout << cloud->size() << endl;
	isShown = false;
}

void PointCloudVisualization::AddCloud(PointCloudT::Ptr newCloud) {
	*cloud += *newCloud;
}

void PointCloudVisualization::SetColor(const int r, const int g, const int b) {
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

void PointCloudVisualization::SetPointSize(const int sz) {
	pointSize = sz;
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pointSize, id);

	viewer->updatePointCloud(cloud, id);
}

void PointCloudVisualization::Save(string filepath) {
	pcl::PCDWriter writer;
	writer.write(filepath, *cloud);
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
	cloudSize = cloud->size();
	return cloudSize;
}

string PointCloudVisualization::GetId() {
	return id;
}

PointCloudT::Ptr PointCloudVisualization::GetCloudPtr() {
	return cloud;
}




