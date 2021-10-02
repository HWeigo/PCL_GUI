#pragma once

#include <iostream>
#include <vector>
#include <string>

// Qt
#include <QMainWindow>
// Point Cloud Library
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

// Visualization Toolkit (VTK)
#include <vtkRenderWindow.h>

//#include <QtWidgets/QMainWindow>
#include "ui_PointCloudLab.h"

#include "PointCloudVisualization.h"
#include "PointTree.h"

using std::string;

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

namespace Ui
{
	class PCLViewer;
}

using namespace std;

class PointCloudLab : public QMainWindow
{
    Q_OBJECT

public:
    PointCloudLab(QWidget *parent = Q_NULLPTR);

private:
	PointCloudT::Ptr clicked_points_3d;

    Ui::PointCloudLabClass ui;
    void OpenFile();
    void PushMessage(string msg);
    void InitVtk();
    void InitPointTree();

    vector<string> split(const string& str, const string& delim);
	static void point_callback(const pcl::visualization::PointPickingEvent& event, void* args);


protected:
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
	vector<PointCloudVisualization> cloudVisualVector;
    vector<PointTree*> PointCloudTree;
    
public slots:
    void on_openFileAction_triggered(bool checked); 
                                  

};
