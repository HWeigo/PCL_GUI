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
    
	viewer->registerPointPickingCallback(point_callback, this);
	PointCloudT::Ptr clicked_points_3d_(new PointCloudT);
	this->clicked_points_3d = clicked_points_3d_;
	
	ui.qvtkWidget->SetRenderWindow(viewer->getRenderWindow());
    viewer->setupInteractor(ui.qvtkWidget->GetInteractor(), ui.qvtkWidget->GetRenderWindow());
    ui.qvtkWidget->update();

    viewer->resetCamera();
    ui.qvtkWidget->update();
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


	PointT current_point;
	event.getPoint(current_point.x, current_point.y, current_point.z);
	p->clicked_points_3d->points.push_back(current_point);

	pcl::visualization::PointCloudColorHandlerCustom<PointT> red(p->clicked_points_3d, 255, 0, 0);

	p->viewer->removePointCloud("clicked_points");
	p->viewer->addPointCloud(p->clicked_points_3d, red, "clicked_points");
	p->viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "clicked_points");
}
 

//slot:
void PointCloudLab::on_openFileAction_triggered(bool checked)
{
    OpenFile();
    PushMessage("打开成功");
}
