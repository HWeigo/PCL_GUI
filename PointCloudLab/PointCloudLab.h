#pragma once

#include <iostream>
#include <vector>
#include <string>

// Qt
#include <QMainWindow>
#include <QMenu>
#include <QAction>
#include <QContextMenuEvent>
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
    Ui::PointCloudLabClass ui;

    QMenu *treeMenu;
    QAction *showAction;
    QAction *hideAction;
    QAction *deleteAction;
    QAction *setColorAction;
    int curPointsId = -1;


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


protected:
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
	vector<PointCloudVisualization*> cloudVisualVector;
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
};
