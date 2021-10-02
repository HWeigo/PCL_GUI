#include "PointCloudLab.h"
#include <QStandardItem>
#include <QDateTime>
#include <QFileDialog>
#include <QTextCodec>

#include <iostream>
using namespace std;

#pragma execution_character_set("utf-8")


PointCloudLab::PointCloudLab(QWidget *parent)
    : QMainWindow(parent)
{
    ui.setupUi(this);
    InitVtk();
    InitPointTree();

}


//functions:

void PointCloudLab::InitVtk()
{
    // Set up the QVTK window
    viewer.reset(new pcl::visualization::PCLVisualizer("viewer", false));
    	
	ui.qvtkWidget->SetRenderWindow(viewer->getRenderWindow());
    viewer->setupInteractor(ui.qvtkWidget->GetInteractor(), ui.qvtkWidget->GetRenderWindow());
    
	ui.qvtkWidget->update();

    viewer->resetCamera();
    ui.qvtkWidget->update();
}


void PointCloudLab::PointPicking() {
	clicked_points_3d.reset(new PointCloudT);
	cloud_mutex.lock();    // for not overwriting the point cloud 
	////viewer->registerPointPickingCallback(&PtPicking::PtActivePick_callback, *this);

	viewer->registerPointPickingCallback(point_callback, this);
	cout << "Shift+click on three floor points, then press 'Q'..." << endl;

	cloud_mutex.unlock();

}

void PointCloudLab::AreaPicking() {
	clicked_points_3d.reset(new PointCloudT);

	cloud_mutex.lock();    
	viewer->registerAreaPickingCallback(area_callback, this);
	std::cout << "press X to strat or ending picking, then press 'Q'..." << std::endl;

	cloud_mutex.unlock();

}

void PointCloudLab::InitPointTree()
{
    ui.treeWidget->setColumnCount(1); //设置列数
    ui.treeWidget->setHeaderHidden(true);
}

void PointCloudLab::OpenFile()
{
	cout << "clicked" << endl;
    QString fileName = QFileDialog::getOpenFileName(this,
        tr("Open PointCloud"), ".",
        tr("Open PCD files(*.pcd)"));
    if (fileName.isEmpty())
    {
        //std::string file_name = fileName.toStdString();
		cerr << "Failed to open file " << endl;
        PushMessage("ERROR: Cannot open file ");
    }

    QTextCodec *code = QTextCodec::codecForName("GB2312");//解决中文路径问题
    std::string filePath = code->fromUnicode(fileName).data();

	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>());

	if (pcl::io::loadPCDFile(filePath, *cloud))
	{
		cerr << "ERROR: Cannot open file " << filePath << "! Aborting..." << endl;
		return;
	}
    vector<string> tempId = PointCloudLab::split(filePath, "/");
    string id = tempId.back();
	PointCloudVisualization pcv(viewer, cloud, id);
	cloudVisualVector.push_back(pcv);
	pcv.Show();
	viewer->resetCamera();
	ui.qvtkWidget->update();
    
    PointTree * tempTree= new PointTree(&ui, id, 100, 100);
    PointCloudTree.push_back(tempTree);
}

void PointCloudLab::PushMessage(string msg)
{
    ui.textBrowser->moveCursor(QTextCursor::End);

    QDateTime current_date_time = QDateTime::currentDateTime();
    QString current_date = current_date_time.toString("MM.dd hh:mm:ss  ");
    ui.textBrowser->insertPlainText(current_date);

    if (msg.back() != '\n')
        msg += '\n';
    QString qmsg = QString::fromStdString(msg);
    ui.textBrowser->insertPlainText(qmsg);
}

vector<string> PointCloudLab::split(const string& str, const string& delim)
{
    vector<string> res;
    if ("" == str) return res;
    //先将要切割的字符串从string类型转换为char*类型  
    char * strs = new char[str.length() + 1]; //不要忘了  
    strcpy(strs, str.c_str());

    char * d = new char[delim.length() + 1];
    strcpy(d, delim.c_str());

    char *p = strtok(strs, d);
    while (p) {
        string s = p; //分割得到的字符串转换为string类型  
        res.push_back(s); //存入结果数组  
        p = strtok(NULL, d);
    }
    return res;
}

void PointCloudLab::point_callback(const pcl::visualization::PointPickingEvent& event, void* args) {
	//struct callback_args* data = (struct callback_args *)args;
	PointCloudLab *p = (PointCloudLab *)args;
	if (p->motionState == POINT_PICK) {

		PointT current_point;
		event.getPoint(current_point.x, current_point.y, current_point.z);
		p->clicked_points_3d->points.push_back(current_point);

		pcl::visualization::PointCloudColorHandlerCustom<PointT> red(p->clicked_points_3d, 255, 0, 0);

		p->viewer->removePointCloud("clicked_points");
		p->viewer->addPointCloud(p->clicked_points_3d, red, "clicked_points");
		p->viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "clicked_points");
	}
}
 
void PointCloudLab::area_callback(const pcl::visualization::AreaPickingEvent& event, void *args) {
	PointCloudLab *p = (PointCloudLab *)args;
	if (p->motionState == AREA_PICK) {
		vector<int > indices;
		if (event.getPointsIndices(indices) == false)
			return;
		cout << indices.size() << endl;
		//for (size_t i = 0; i < indices.size(); i++)
		//{
		//	p->clicked_points_3d->points.push_back(baseCloud->points.at(indices[i]));
		//}
		//pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> red(clicked_points_3d, 255, 0, 0);
		//viewer->removePointCloud("clicked_points");
		//viewer->addPointCloud(clicked_points_3d, red, "clicked_points");
		//viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "clicked_points");
		//for (int i = 0; i < clicked_points_3d->points.size(); i++)
		//	std::cout << clicked_points_3d->points[i].x << std::endl;
		//std::cout << "clicked_points_3d->points.size()" << clicked_points_3d->points.size() << std::endl;
		
		//if (event.getPointIndex() == -1)
		//	return;
	}
}
//slot:
void PointCloudLab::on_openFileAction_triggered(bool checked)
{
    OpenFile();
    PushMessage("打开成功");
}

void PointCloudLab::on_pushButton_pointPick_clicked() {
	//cout << "-- Point pick mode -- " << endl;
	motionState = POINT_PICK;
	PointPicking();
}

void PointCloudLab::on_pushButton_areaPick_clicked() {
	motionState = AREA_PICK;
	int n = cloudVisualVector.size();

	AreaPicking();
}

void PointCloudLab::on_pushButton_drag_clicked() {
	motionState = DRAG;

	int n = clicked_points_3d->size();
	cout << "Remove " << n << " selected points" << endl;
	//pcl::visualization::PointCloudColorHandlerCustom<PointT> red(p->clicked_points_3d, 255, 0, 0);
	clicked_points_3d.reset(new PointCloudT);
	viewer->removePointCloud("clicked_points");
	ui.qvtkWidget->update();

	//p->viewer->addPointCloud(p->clicked_points_3d, red, "clicked_points");
	//p->viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "clicked_points");
}
