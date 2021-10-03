#pragma once

#include <iostream>
#include <vector>
#include <string>
#include <unordered_map>
#include <unordered_set>

// Qt
#include <QMainWindow>
#include <QMenu>
#include <QAction>
#include <QContextMenuEvent>
#include <QColorDialog>

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
#include "PointCloudVector.h"
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
    ~PointCloudLab();

private:
	boost::mutex cloud_mutex;
	PointCloudT::Ptr clicked_points_3d;
	int selectedCloudIdx = -1;
	vector<int> selectedPointIdxs;
	unordered_set<int> setSelected;

	typedef enum {
		DRAG = 0,
		POINT_PICK = 1,
		AREA_PICK = 2
	} MOTION_STATE;

	int motionState = DRAG;

    Ui::PointCloudLabClass ui;

	enum {
		SUCCESS,
		FAILED,
		CANCEL
	};

    QMenu *treeMenu;
    QAction *showAction;
    QAction *hideAction;
    QAction *deleteAction;
	QAction *setColorAction;
	QAction *saveCurPointAction;
    int curPointsId = -1;

	int pointViewerSize = 3;
	int backGroundColor[3] = { 0,0,0 };
	bool isCoordinateSystemShown = false;
	double coordinateSystemSize = 0.1;
	bool isCoordinateSystemSizeShown = false;

    void contextMenuEvent(QContextMenuEvent *event);

    void PushMessage(string msg);
    void InitVtk();
    void InitPointTree();
    void InitMenuAction();
    vector<string> split(const string& str, const string& delim);
    
    //打开文件相关
    int OpenFile(string filePath);
    int OpenPcdFile(string path);
    int OpenPlyFile(string path);
    int OpenObjFile(string path);
    int OpenStlFile(string path);
    int OpenMeshFile(string path);
    int OpenPngFile(string path);
	vector<int> GetValidPointsId();


	void PointPicking();
	void AreaPicking();
	static void point_callback(const pcl::visualization::PointPickingEvent& event, void* args);
	static void area_callback(const pcl::visualization::AreaPickingEvent& event, void *args);

protected:
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
	PointCloudVisualization* completeCloud = nullptr;
	PointCloudVisualization* selectedCloud = nullptr;
	PointCloudVector* pointCloudVector;
    vector<PointTree*> PointCloudTree;
    vector<bool> isDeleted;
    vector<bool> isShown;
    
public slots:
    void on_openFileAction_triggered(bool checked); 
    void on_saveFileAction_triggered(bool checked);
    void on_filterAction1_triggered(bool checked);//直通滤波
    void on_filterAction2_triggered(bool checked);//体素滤波
    void on_filterAction3_triggered(bool checked);//统计滤波
    void on_copyPointAction_triggered(bool checked);//复制点云
    void on_extractPointAction_triggered(bool checked);//提取点云
    void OnShowAction();
    void OnHideAction();
    void OnDeleteAction();
    void OnSetColorAction();
	void OnSaveCurPointAction();

	void on_pushButton_pointPick_clicked();
	void on_pushButton_areaPick_clicked();
	void on_pushButton_drag_clicked();
	void on_pushButton_clicked();
	void on_pushButton_allSelect_clicked();
	void on_pushButton_invertSelect_clicked();
	void on_pushButton_setting_clicked();
};
