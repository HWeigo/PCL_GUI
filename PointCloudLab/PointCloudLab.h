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
	boost::mutex cloud_mutex;
	PointCloudT::Ptr clicked_points_3d;
	typedef enum {
		DRAG = 0,
		POINT_PICK = 1,
		AREA_PICK = 2
	} MOTION_STATE;

	int motionState = DRAG;

    Ui::PointCloudLabClass ui;
    void OpenFile();
    void PushMessage(string msg);
    void InitVtk();
    void InitPointTree();
	void PointPicking();
	void AreaPicking();

    vector<string> split(const string& str, const string& delim);
	static void point_callback(const pcl::visualization::PointPickingEvent& event, void* args);
	static void area_callback(const pcl::visualization::AreaPickingEvent& event, void *args);

protected:
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
	vector<PointCloudVisualization> cloudVisualVector;
    vector<PointTree*> PointCloudTree;
    
public slots:
    void on_openFileAction_triggered(bool checked); 
	void on_pushButton_pointPick_clicked();
	void on_pushButton_areaPick_clicked();
	void on_pushButton_drag_clicked();
                                  

};
