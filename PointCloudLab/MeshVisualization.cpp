#include "MeshVisualization.h"

MeshVisualization::MeshVisualization(boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_in, pcl::PolygonMesh::Ptr mesh_in, string id_in) :
	BaseVisualization(viewer_in, id_in), meshPtr(mesh_in)
{
}

MeshVisualization::~MeshVisualization()
{
}


void MeshVisualization::Show() {
	isShown = true;
	cout << "Display - " << id << endl;

	viewer->addPolygonMesh(*meshPtr, id);
	viewer->updatePolygonMesh(*meshPtr, id);
}


void MeshVisualization::Hide() {
	isShown = false;
	cout << "Hide - " << id << endl;

	viewer->removePolygonMesh(id);
}

void MeshVisualization::Save(string filepath){
	Save(filepath, ".ply");
}

void MeshVisualization::Save(string filepath, string type) {
	if (type == ".ply") {
		pcl::io::savePLYFile(filepath, *meshPtr);
	} else if (type == ".stl") {
		pcl::io::savePolygonFileSTL(filepath, *meshPtr);
	} else if (type == ".obj") {
		pcl::io::saveOBJFile(filepath, *meshPtr);
	}
}

void MeshVisualization::Delete(){
	Hide();
	//pcl::PolygonMesh().
	pcl::PolygonMesh::Ptr().swap(meshPtr);
}

MeshT::Ptr MeshVisualization::GetMeshPtr()
{
	return meshPtr;
}

vector<int> MeshVisualization::GetColor()
{
	vector<int> color(3);
	color[0] = red;
	color[1] = green;
	color[2] = blue;

	return color;
}

void MeshVisualization::ChangeID(string newID) {
	Hide();
	BaseVisualization::ChangeID(newID);
	Show();
}

int MeshVisualization::GetFaceNum()
{
	return meshPtr->polygons.size();
}

int MeshVisualization::GetPointNum()
{
	return meshPtr->cloud.width;
}

void MeshVisualization::SetColor(const int r, const int g, const int b)
{
	Hide();
	red = r;
	green = g;
	blue = b;
	pcl::PCLPointCloud2 tempPd = meshPtr->cloud;
	PointCloudT cloudMesh;
	pcl::fromPCLPointCloud2(meshPtr->cloud, cloudMesh);
	for (int i = 0; i < cloudMesh.size(); ++i) {
		cloudMesh.points[i].r = r;
		cloudMesh.points[i].g = g;
		cloudMesh.points[i].b = b;
	}
	pcl::toPCLPointCloud2(cloudMesh, meshPtr->cloud);

	cout << "show" << endl;
	Show();
	//viewer->updatePolygonMesh(*mesh, id);
	//mesh->cloud
}


