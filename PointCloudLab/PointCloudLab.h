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
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/project_inliers.h>

// Visualization Toolkit (VTK)
#include <vtkRenderWindow.h>

//#include <QtWidgets/QMainWindow>
#include "ui_PointCloudLab.h"

#include "PointCloudVisualization.h"
#include "EntityVector.h"
#include "EntityTree.h"

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

	// --- Point selection function ---
	// Store selected point cloud
	PointCloudT::Ptr clicked_points_3d;

	// Index of the selected point cloud to be processed picking
	int selectedCloudIdx = -1;
	// Index of the selected mesh to be processed picking
	int selectedMeshIdx = -1;

	// Point indexs in the selected point cloud (temporarily used in callback)
	vector<int> selectedPointIdxs;

	// Set to store point indexs in the selected point cloud
	unordered_set<int> setSelected;


	typedef enum {
		DRAG = 0,
		POINT_PICK = 1,
		AREA_PICK = 2
	} MOTION_STATE;

	// Indicate current motion mode
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

	// Set point size for all the point cloud
	int pointViewerSize = 3;

	// Background color
	int backGroundColor[3] = { 0,0,0 };

	// Coordinate system flag and size
	bool isCoordinateSystemShown = false;
	double coordinateSystemSize = 0.1;
	bool isCoordinateSystemSizeShown = false;

	// Initialization
    void InitVtk();
    void InitPointTree();
    void InitMenuAction();

	// Handle right click on the context menu
    void contextMenuEvent(QContextMenuEvent *event);

	// Used to push message into info widget
    void PushMessage(string msg);

	// Helper function used to split filename from filepath
	// Used in OpenFile(string) and SaveFile(string)
    vector<string> split(const string& str, const string& delim);
    
    // Read point cloud/mesh, called by void on_openFileAction_triggered(bool checked)
	int OpenFile(string filePath);
	// -- Subfunction used to read different pointcloud/mesh file --
    int OpenPcdFile(string path);
    int OpenPlyFile(string path);
    int OpenObjFile(string path);
    int OpenStlFile(string path);
    int OpenMeshFile(string path);
    int OpenPngFile(string path);
    int OpenTxtFile(string path);

	// Get the index of valid entities in visualVector
	vector<int> GetValidEntitiesId();

	// Get the index of valid entities of a specific type in visualVector
	// type == POINTCLOUD_TYPE or MESH_TYPE, defined in BaseVisualization.h
	vector<int> GetValidEntitiesId(string type);

	// --- EXAMPLE START ---
	// Assume two point (p1 p2) cloud and one mesh (m1) is sequentially opened or created 
	// +──────────+─────────────+─────────────+───────+
	// | name     | p1          | p2          | m1    |
	// +──────────+─────────────+─────────────+───────+
	// | type     | pointcloud  | pointcloud  | mesh  |
	// | index    | 0           | 1           | 2     |
	// | isValid  | true        | true        | true  |
	// +──────────+─────────────+─────────────+───────+
	// $ GetValidPointsId() -> 0, 1, 2
	// $ GetValidPointsId(POINTCLOUD_TYPE) -> 0, 1
	// $ GetValidPointsId(MESH_TYPE) -> 2
	//
	// Now if p2 is deleted
	// +──────────+─────────────+────────────────+───────+
	// | name     | p1          | p2 (released)  | m1    |
	// +──────────+─────────────+────────────────+───────+
	// | type     | pointcloud  | pointcloud     | mesh  |
	// | index    | 0           | 1              | 2     |
	// | isValid  | true        | false          | true  |
	// +──────────+─────────────+────────────────+───────+
	// $ GetValidPointsId() -> 0, 2
	// $ GetValidPointsId(POINTCLOUD_TYPE) -> 0
	// $ GetValidPointsId(MESH_TYPE) -> 2
	// --- EXAMPLE END ---


	// --- Point select function ---
	// Point pick helper function
	void PointPicking();
	// Area pick helper function
	void AreaPicking();

	// Callback function of point picking
	// Registed in void PointPicking()
	static void point_callback(const pcl::visualization::PointPickingEvent& event, void* args);
	
	// Callback function of area picking
	// Registed in void AreaPicking()
	static void area_callback(const pcl::visualization::AreaPickingEvent& event, void *args);


	//int on_acceptCallback();
	//int on_rejectCallback();

    // Save point cloud/mesh, called by void on_saveFileAction_triggered(bool checked)
	int SaveFiles(std::string filePath, PointCloudT::Ptr Cloud); // NOT USED
	// -- Subfunction used to save different pointcloud/mesh file -- 
	int SavePcdFile(string filePath, PointCloudT::Ptr Cloud);// NOT USED
	int SavePlyFile(string filePath, PointCloudT::Ptr Cloud);// NOT USED

protected:
	// PCL viewer (binded to Qt::qvtkWidget, see void InitVtk())
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;

	// Store poing cloud and mesh entity (see EntityVector.h)
	EntityVector* entityVector;

	// Stores context manu's information 
    vector<EntityTree*> entityTree;
    
	// Temporarily stores the selected points 
	PointCloudVisualization* selectedCloud = nullptr;

	// (NOT REALLY USED) Stores a complete point cloud which is an assemble of all the opened point cloud
	PointCloudVisualization* completeCloud = nullptr;

public slots:
	// Qt slot: open file 
    void on_openFileAction_triggered(bool checked); 

	// Qt slot: save file 
    void on_saveFileAction_triggered(bool checked);

	// Qt slot: pass through filter 
    void on_filterAction1_triggered(bool checked);//ֱͨ�˲�

	// Qt slot: voxel grid filter 
    void on_filterAction2_triggered(bool checked);//�����˲�

	// Qt slot: statistical filter 
    void on_filterAction3_triggered(bool checked);//ͳ���˲�

	// Qt slot: projection filter 
	void on_filterAction4_triggered(bool checked);//ͶӰ�˲�

	void on_linefitAction_triggered(bool checked);//ֱ�����
	void on_planefitAction_triggered(bool checked);//ƽ�����
	void on_ballfitAction_triggered(bool checked);//�������
	void on_pmdAction_triggered(bool checked);//ƽ��ȼ���
	void on_boundaryAction_triggered(bool checked);//��Ե��ȡ
	void on_match1Action_triggered(bool checked);//���ƴ���׼
	void on_match2Action_triggered(bool checked);//���ƾ���׼
	void on_match3Action_triggered(bool checked);//����ƴ��

	// Qt slot: copy selected point cloud 
    void on_copyPointAction_triggered(bool checked);//���Ƶ���

	// Qt slot: extract selected point cloud 
    void on_extractPointAction_triggered(bool checked);//��ȡ����

	// Qt slot: extract point cloud from mesh
	void on_meshToPointCloudAction_triggered(bool checked);

	// Qt slot: enable point picking mode
	void on_pushButton_pointPick_clicked();

	// Qt slot: enable area picking mode
	void on_pushButton_areaPick_clicked();

	// Qt slot: enable drag mode
	void on_pushButton_drag_clicked();

	// Qt slot: select all the point
	void on_pushButton_allSelect_clicked();

	// Qt slot: invert selected point
	void on_pushButton_invertSelect_clicked();

	// Qt slot: point/background/coordinate system setting
	void on_pushButton_setting_clicked();

	// --- Context manu function ---
	// Qt slot: show pointcloud/mesh
    void OnShowAction();

	// Qt slot: hide pointcloud/mesh
    void OnHideAction();

	// Qt slot: delete pointcloud/mesh
    void OnDeleteAction();

	// Qt slot: set pointcloud/mesh color
    void OnSetColorAction();

	// Qt slot: save pointcloud/mesh 
	void OnSaveCurPointAction();

	// For testing
	void on_pushButton_clicked();
	void on_newBtn_clicked();

};
