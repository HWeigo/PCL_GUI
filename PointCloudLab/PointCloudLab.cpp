#include "PointCloudLab.h"
#include <QStandardItem>
#include <QDateTime>
#include <QFileDialog>
#include <QTextCodec>
#include <QFormLayout>
#include <QSpinBox>
#include <QDialogButtonBox>
#include <QLabel>
#include <QCheckBox>
#include <QLineEdit>
#include <QColorDialog>
#include <QMessageBox>
#include <QComboBox>
#include <QDebug>

#include <iostream>
#include "fun.hpp"
# include <stdlib.h>
#include "colorwidget.h"


#pragma execution_character_set("utf-8")


PointCloudLab::PointCloudLab(QWidget *parent)
    : QMainWindow(parent)
{
    ui.setupUi(this);
    InitVtk();
    InitPointTree();
    InitMenuAction();

	// Init PointCloudVector
	entityVector = new PointCloudVector(viewer);

	// Init complete cloud and selected cloud
	PointCloudT::Ptr tempSelectedCloud(new PointCloudT());
	selectedCloud = new PointCloudVisualization(viewer, tempSelectedCloud,"_SELECTED_CLOUD_");
	PointCloudT::Ptr tempCompleteCloud(new PointCloudT());
	completeCloud = new PointCloudVisualization(viewer, tempCompleteCloud, "_COMPLETE_CLOUD_");
}
PointCloudLab::~PointCloudLab()
{
    delete treeMenu;
    delete showAction;
    delete hideAction;
    delete deleteAction;
    delete setColorAction;
	delete saveCurPointAction;

    for (int i = 0; i < entityTree.size(); ++i)
    {
        if (entityTree[i] != nullptr) {
            delete entityTree[i];
        }
    }
}

//functions:

void PointCloudLab::contextMenuEvent(QContextMenuEvent *event)
{
    QTreeWidgetItem *item = ui.treeWidget->currentItem();
    if (item != nullptr)
    {
        for (int i = 0; i < entityTree.size(); ++i)
        {
            if (entityTree[i] != nullptr&&entityTree[i]->cloudName == item)
            {
                curPointsId = i;
                treeMenu->clear();
                treeMenu->addAction(showAction);
                treeMenu->addAction(hideAction);
                treeMenu->addAction(deleteAction);
                treeMenu->addAction(setColorAction);
				treeMenu->addAction(saveCurPointAction);
                treeMenu->exec(QCursor::pos());   //菜单弹出位置为鼠标点击位置
               
                break;
            }
        }
    }
    event->accept();
}


void PointCloudLab::InitMenuAction()
{
	treeMenu = new QMenu(ui.treeWidget);
	showAction = new QAction("显示", ui.treeWidget);
	hideAction = new QAction("隐藏", ui.treeWidget);
	deleteAction = new QAction("删除点云", ui.treeWidget);
	setColorAction = new QAction("设置颜色", ui.treeWidget);
	saveCurPointAction = new QAction("保存点云", ui.treeWidget);

	connect(showAction, SIGNAL(triggered(bool)), this, SLOT(OnShowAction()));
	connect(hideAction, SIGNAL(triggered(bool)), this, SLOT(OnHideAction()));
	connect(deleteAction, SIGNAL(triggered(bool)), this, SLOT(OnDeleteAction()));
	connect(setColorAction, SIGNAL(triggered(bool)), this, SLOT(OnSetColorAction()));
	connect(saveCurPointAction, SIGNAL(triggered(bool)), this, SLOT(OnSaveCurPointAction()));
}

void PointCloudLab::InitVtk()
{
    // Set up the QVTK window
    viewer.reset(new pcl::visualization::PCLVisualizer("viewer", false));
    ui.qvtkWidget->SetRenderWindow(viewer->getRenderWindow());
    viewer->setupInteractor(ui.qvtkWidget->GetInteractor(), ui.qvtkWidget->GetRenderWindow());
    ui.qvtkWidget->update();

    viewer->resetCamera();
	if(isCoordinateSystemShown) viewer->addCoordinateSystem(coordinateSystemSize);
    ui.qvtkWidget->update();
	
}

void PointCloudLab::InitPointTree()
{
    ui.treeWidget->setColumnCount(2); //设置列数
    ui.treeWidget->setHeaderHidden(false);
	QStringList header;
	header.push_back(QString("名称"));
	header.push_back(QString("属性"));
	ui.treeWidget->setHeaderLabels(header);
	ui.treeWidget->header()->setSectionResizeMode(QHeaderView::Interactive);
}


void PointCloudLab::PushMessage(std::string msg)
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

vector<std::string> PointCloudLab::split(const std::string& str, const std::string& delim)
{
    vector<std::string> res;
    if ("" == str) return res;
    //先将要切割的字符串从string类型转换为char*类型  
    char * strs = new char[str.length() + 1]; //不要忘了  
    strcpy(strs, str.c_str());

    char * d = new char[delim.length() + 1];
    strcpy(d, delim.c_str());

    char *p = strtok(strs, d);
    while (p) {
        std::string s = p; //分割得到的字符串转换为string类型  
        res.push_back(s); //存入结果数组  
        p = strtok(NULL, d);
    }
    return res;
}

int PointCloudLab::OpenFile(std::string filePath)
{
    vector<std::string> temp = split(filePath, ".");
    if (temp.back() == "pcd") {
        return OpenPcdFile(filePath);
    }
    else if (temp.back() == "ply") {
        return OpenPlyFile(filePath);
    }
    else if (temp.back() == "obj") {
        return OpenObjFile(filePath);
    }
    else if (temp.back() == "stl") {
        return OpenStlFile(filePath);
    }
    else if (temp.back() == "png") {
        return OpenPngFile(filePath);
    }
}

int PointCloudLab::OpenPcdFile(std::string path)
{
	PointCloudT::Ptr cloud(new PointCloudT());


    if (pcl::io::loadPCDFile(path, *cloud))
    {
        cerr << "ERROR: Cannot open file " << path << "! Aborting..." << endl;
        return FAILED;
    }
    vector<std::string> tempId = PointCloudLab::split(path, "/");
    std::string id = tempId.back();
	int idx = entityVector->AddPointCloud(cloud, id);
	PointCloudVisualization *pcv = entityVector->GetPCVofIdx(idx);
    pcv->Show();
    viewer->resetCamera();
    ui.qvtkWidget->update();

	completeCloud->AddCloud(cloud);

	// Todo: edit type, point num, face num
	PointTree * tempTree = new PointTree(&ui, id, "PointXYZ", pcv->GetPointNum(), 0);
	entityTree.push_back(tempTree);
    //isDeleted.push_back(false);
    //isShown.push_back(true);
    return SUCCESS;
}
int PointCloudLab::OpenPlyFile(std::string path)
{
	//PointCloudT::Ptr cloud(new PointCloudT());
	
	pcl::PolygonMesh::Ptr mesh(new pcl::PolygonMesh());
	if (pcl::io::loadPLYFile(path, *mesh))
	{
		cerr << "ERROR: Cannot open file " << path << "! Aborting..." << endl;
		return FAILED;
	}

	vector<std::string> tempId = PointCloudLab::split(path, "/");
	std::string id = tempId.back();
	int idx = entityVector->AddMesh(mesh, id);

	//int idx = pointCloudVector->AddPointCloud(cloud, id);
	MeshVisualization *meshv = entityVector->GetMESHVofIdx(idx);
	meshv->Show();
	viewer->resetCamera();
	ui.qvtkWidget->update();

	// Todo: edit type, point num, face num
	PointTree * tempTree = new PointTree(&ui, id, "Mesh", meshv->GetPointNum(), meshv->GetFaceNum());
	entityTree.push_back(tempTree);


	//if (pcl::io::loadPLYFile(path, *cloud))
	//{
	//	cerr << "ERROR: Cannot open file " << path << "! Aborting..." << endl;
	//	return FAILED;
	//}

	//vector<std::string> tempId = PointCloudLab::split(path, "/");
	//std::string id = tempId.back();
	//int idx = pointCloudVector->AddPointCloud(cloud, id);
	//PointCloudVisualization *pcv = pointCloudVector->GetPCVofIdx(idx);
	//pcv->Show();
	//viewer->resetCamera();
	//ui.qvtkWidget->update();

	//completeCloud->AddCloud(cloud);

	//// Todo: edit type, point num, face num
	//PointTree * tempTree = new PointTree(&ui, id, "PointCloud", pcv->GetCloudSize(), 0);
	//PointCloudTree.push_back(tempTree);
	//isDeleted.push_back(false);
	//isShown.push_back(true);
	return SUCCESS;
}
int PointCloudLab::OpenObjFile(std::string path)
{
	pcl::PolygonMesh mesh;
	PointCloudT::Ptr cloud(new PointCloudT());


	if (pcl::io::loadPolygonFileOBJ(path, mesh) == -1)//*打开点云文件  path为obj文件名
	{
		cerr << "ERROR: Cannot open file " << path << "! Aborting..." << endl;
		return FAILED;
	}
	pcl::fromPCLPointCloud2(mesh.cloud, *cloud);

	vector<std::string> tempId = PointCloudLab::split(path, "/");
	std::string id = tempId.back();
	int idx = entityVector->AddPointCloud(cloud, id);
	PointCloudVisualization *pcv = entityVector->GetPCVofIdx(idx);
	pcv->Show();
	viewer->resetCamera();
	ui.qvtkWidget->update();

	completeCloud->AddCloud(cloud);

	// Todo: edit type, point num, face num
	PointTree * tempTree = new PointTree(&ui, id, "PointCloud", pcv->GetPointNum(), 0);
	entityTree.push_back(tempTree);
	//isDeleted.push_back(false);
	//isShown.push_back(true);
	return SUCCESS;

}

//stl,mesh,png 不能读成pointcloudxyz格式
int PointCloudLab::OpenStlFile(std::string path)
{
	return CANCEL;
}
int PointCloudLab::OpenMeshFile(std::string path)
{
	return CANCEL;
}



//png格式 可以读成矩阵

int PointCloudLab::OpenPngFile(std::string path)
{
	QDialog dialog(this);
	//dialog.setFixedSize(200, 200);
	dialog.setFixedSize(200, 200);
	dialog.setWindowTitle("设置深度图参数");
	QFormLayout form(&dialog);


	QLineEdit fx, fy, cx, cy, s;
	QDoubleValidator aDoubleValidator;
	fx.setValidator(&aDoubleValidator);
	fy.setValidator(&aDoubleValidator);
	cx.setValidator(&aDoubleValidator);
	cy.setValidator(&aDoubleValidator);
	s.setValidator(&aDoubleValidator);
	fx.setText("616.3767");
	fy.setText("616.3767");
	cx.setText("326.9487");
	cy.setText("232.3900");
	s.setText("1000");
	form.addRow(QString("X轴焦距= "), &fx);
	form.addRow(QString("Y轴焦距= "), &fy);
	form.addRow(QString("相机中心cx= "), &cx);
	form.addRow(QString("相机中心cy= "), &cy);
	form.addRow(QString("深度缩放因子s= "), &s);
	

	QString fxStr = fx.text();
	QString fyStr = fy.text();
	QString cxStr = cx.text();
	QString cyStr = cy.text();
	QString sStr = s.text();

	double camera_fx = fxStr.toDouble();
	double camera_fy = fyStr.toDouble();
	double camera_cx = cxStr.toDouble();
	double camera_cy = cyStr.toDouble();
	double camera_factor = sStr.toDouble();

	QHBoxLayout  tempLayout1;
	QCheckBox checkBoxX("是否添加颜色");
	tempLayout1.addWidget(&checkBoxX);
	form.addRow(&tempLayout1);

	QDialogButtonBox buttonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel,Qt::Horizontal, &dialog);
	form.addRow(&buttonBox);
	QObject::connect(&buttonBox, SIGNAL(accepted()), &dialog, SLOT(accept()));
	QObject::connect(&buttonBox, SIGNAL(rejected()), &dialog, SLOT(reject()));

	if (dialog.exec() == QDialog::Rejected) {
		
		return CANCEL;
	}


	if (checkBoxX.isChecked()) 
	{
		QString curPath = "C:/Users/sjtuzhy/Desktop/point cloud doc";
		QString dlgTitle = "打开文件"; //对话框标题
		QString filter = "png文件(*.png)"; //文件过滤器
		QString fileName = QFileDialog::getOpenFileName(this, dlgTitle, curPath, filter);

		/*if (fileName.isEmpty())
		{
		return;
		}*/

		QTextCodec *code = QTextCodec::codecForName("GB2312");//解决中文路径问题
		std::string filePath = code->fromUnicode(fileName).data();


		cv::Mat depth, rgb;
		rgb = cv::imread(filePath, -1);
		depth = cv::imread(path, -1);
		PointCloudT::Ptr cloud(new PointCloudT());

		for (int m = 0; m < depth.rows; m++)
			for (int n = 0; n < depth.cols; n++)
			{
				// 获取深度图中(m,n)处的值
				ushort d = depth.ptr<ushort>(m)[n];
				// d 可能没有值，若如此，跳过此点
				if (d == 0)
					continue;
				// d 存在值，则向点云增加一个点
				pcl::PointXYZRGBA p;

				// 计算这个点的空间坐标
				p.z = double(d) / camera_factor;
				p.x = (n - camera_cx) * p.z / camera_fx;
				p.y = (m - camera_cy) * p.z / camera_fy;

				// 从rgb图像中获取它的颜色
				// rgb是三通道的BGR格式图，所以按下面的顺序获取颜色
				p.b = rgb.ptr<uchar>(m)[n * 3];
				p.g = rgb.ptr<uchar>(m)[n * 3 + 1];
				p.r = rgb.ptr<uchar>(m)[n * 3 + 2];

				// 把p加入到点云中
				cloud->points.push_back(p);
				//cout << cloud->points.size() << endl;
			}
		cloud->height = 1;
		cloud->width = cloud->points.size();
		//cout << "point cloud size = " << cloud->points.size() << endl;
		cloud->is_dense = false;


		vector<std::string> tempId = PointCloudLab::split(path, "/");
		std::string id = tempId.back();
		int idx = entityVector->AddPointCloud(cloud, id);
		PointCloudVisualization *pcv = entityVector->GetPCVofIdx(idx);
		pcv->Show();
		viewer->resetCamera();
		ui.qvtkWidget->update();

		completeCloud->AddCloud(cloud);

		// Todo: edit type, point num, face num
		PointTree * tempTree = new PointTree(&ui, id, "PointCloud", pcv->GetPointNum(), 0);
		entityTree.push_back(tempTree);
		//isDeleted.push_back(false);
		//isShown.push_back(true);
		return SUCCESS;
	}
	else
	{
		cv::Mat depth;
		
		depth = cv::imread(path, -1);
		PointCloudT::Ptr cloud(new PointCloudT());

		for (int m = 0; m < depth.rows; m++)
			for (int n = 0; n < depth.cols; n++)
			{
				// 获取深度图中(m,n)处的值
				ushort d = depth.ptr<ushort>(m)[n];
				// d 可能没有值，若如此，跳过此点
				if (d == 0)
					continue;
				// d 存在值，则向点云增加一个点
				pcl::PointXYZRGBA p;

				// 计算这个点的空间坐标
				p.z = double(d) / camera_factor;
				p.x = (n - camera_cx) * p.z / camera_fx;
				p.y = (m - camera_cy) * p.z / camera_fy;


				// 把p加入到点云中
				cloud->points.push_back(p);
				//cout << cloud->points.size() << endl;
			}
		cloud->height = 1;
		cloud->width = cloud->points.size();
		//cout << "point cloud size = " << cloud->points.size() << endl;
		cloud->is_dense = false;


		vector<std::string> tempId = PointCloudLab::split(path, "/");
		std::string id = tempId.back();
		int idx = entityVector->AddPointCloud(cloud, id);
		PointCloudVisualization *pcv = entityVector->GetPCVofIdx(idx);
		pcv->Show();
		viewer->resetCamera();
		ui.qvtkWidget->update();

		completeCloud->AddCloud(cloud);

		// Todo: edit type, point num, face num
		PointTree * tempTree = new PointTree(&ui, id, "PointCloud", pcv->GetPointNum(), 0);
		entityTree.push_back(tempTree);
		//isDeleted.push_back(false);
		//isShown.push_back(true);
		return SUCCESS;
	}
	
	
	
}




//slot:
void PointCloudLab::on_openFileAction_triggered(bool checked)
{
    //QString curPath = QDir::currentPath();
	QString curPath = "C:/Users/sjtuzhy/Desktop/point cloud doc";
    QString dlgTitle = "打开文件"; //对话框标题
    QString filter = "pcd文件(*.pcd);;ply文件(*.ply);;obj文件(*.obj);;stl文件(*.stl);;png文件(*.png);;所有文件(*.*)"; //文件过滤器
    QString fileName = QFileDialog::getOpenFileName(this, dlgTitle, curPath, filter);

    if (fileName.isEmpty())
    {
        return;
    }

    QTextCodec *code = QTextCodec::codecForName("GB2312");//解决中文路径问题
    std::string filePath = code->fromUnicode(fileName).data();

	int ret = OpenFile(filePath);
	if (ret == SUCCESS)
	{
		PushMessage("打开成功");
	}
	else if (ret == FAILED)
	{
		PushMessage("打开失败");
	}
}

int PointCloudLab::SaveFiles(std::string filePath, PointCloudT::Ptr Cloud)
{
	vector<std::string> temp = split(filePath, ".");
	if (temp.back() == "pcd") {
		return SavePcdFile(filePath, Cloud);
	}
	else if (temp.back() == "ply") {
		return SavePlyFile(filePath, Cloud);
	}
	/*
	else if (temp.back() == "obj") {
	return OpenObjFile(filePath);
	}
	else if (temp.back() == "stl") {
	return OpenStlFile(filePath);
	}
	else if (temp.back() == "png") {
	return OpenPngFile(filePath);
	}*/
}


int  PointCloudLab::SavePcdFile(std::string filepath, PointCloudT::Ptr Cloud)
{
	if (pcl::io::savePCDFile(filepath, *Cloud))
	{
		cerr << "ERROR: Cannot save file " << filepath << "! Aborting..." << endl;
		return FAILED;
	}
	return SUCCESS;
}

int  PointCloudLab::SavePlyFile(std::string filepath, PointCloudT::Ptr Cloud)
{
	if (pcl::io::savePLYFile(filepath, *Cloud))
	{
		cerr << "ERROR: Cannot save file " << filepath << "! Aborting..." << endl;
		return FAILED;
	}
	return SUCCESS;
}

//点云保存
void PointCloudLab::on_saveFileAction_triggered(bool checked)
{
	// Todo: check valid point
	vector<int> validPoints = GetValidPointsId();
	if (validPoints.size() == 0) {
		QString dlgTitle = "提示";
		QString strInfo = "当前没有打开的点云";
		QMessageBox::information(this, dlgTitle, strInfo, QMessageBox::Ok, QMessageBox::NoButton);
		return;
	}

	QDialog dialog(this);
	dialog.setFixedSize(300, 100);
	dialog.setWindowTitle("选择要保存的点云");
	QFormLayout form(&dialog);
	QComboBox curComboBox;
	for (int i = 0; i < validPoints.size(); ++i) {
		curComboBox.addItem(QString::fromStdString(entityVector->GetId(validPoints[i])));//改成点云的名字
	}
	form.addRow(QString("选择点云："), &curComboBox);

	QDialogButtonBox buttonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel,
		Qt::Horizontal, &dialog);
	form.addRow(&buttonBox);
	QObject::connect(&buttonBox, SIGNAL(accepted()), &dialog, SLOT(accept()));
	QObject::connect(&buttonBox, SIGNAL(rejected()), &dialog, SLOT(reje ct()));
	
	if (dialog.exec() == QDialog::Rejected) {
		return;
	}

	selectedCloudIdx = validPoints[curComboBox.currentIndex()];
	PointCloudT::Ptr selectedCloud = entityVector->GetCloudPtrOfIdx(selectedCloudIdx);


	
	//QString curPath = QCoreApplication::applicationDirPath();
	QString curPath = "C:/Users/sjtuzhy/Desktop/point cloud doc";
	QString dlgTitle = "保存点云"; //对话框标题
	QString filter = "pcd文件(*.pcd);;ply文件(*.ply);;obj文件(*.obj);;stl文件(*.stl);;所有文件(*.*)"; //文件过滤器 mesh文件(*.mesh);;png文件(*.png);;
	QString fileName = QFileDialog::getSaveFileName(this, dlgTitle, curPath, filter);
	
	if (fileName.isEmpty()) {
		return;
	}

	QFileInfo file(fileName);
	if (file.exists()) {
		QFile::remove(fileName);
	}

	QTextCodec *code = QTextCodec::codecForName("GB2312");//解决中文路径问题
	std::string filePath = code->fromUnicode(fileName).data();
	vector<std::string> temp = split(filePath, ".");
	std::string fileType = temp.back();

	//selectedCloud->height = (selectedCloud->height == 0) ? 1 : selectedCloud->height;
	//selectedCloud->width = (selectedCloud->width == 0) ? selectedCloud->size() : selectedCloud->width;
	//selectedCloud->height = 1; selectedCloud->width = selectedCloud->size();
	//std::cout << selectedCloud <<'\n'<<selectedCloud->size()<<'\n'<<selectedCloud->height<<'\n'<<selectedCloud->width<<'\n';
	//std::cout << filePath << '\n';

	int ret = SaveFiles(filePath,selectedCloud);
	if (ret == SUCCESS)
	{
		std::cout << "成功" << '\n';
		PushMessage("保存成功");
	}
	else if (ret == FAILED)
	{
		std::cout << "失败" << '\n';
		PushMessage("保存失败");
	}
	/*
	if (fileType == "pcd" )       //|| fileType == "ply" || fileType == "obj" || fileType == "stl" || fileType == "mesh"
	{
		std::cout << selectedCloud << ' ' << selectedCloud->size()<<'  ' << filePath<<'\n';
		
		pcl::io::savePCDFile(filePath, *selectedCloud);
		return;

	}
	else if (fileType == "ply")
	{
		pcl::io::savePLYFile(filePath,*selectedCloud);
		return;
	}
	PushMessage("保存成功");
	/*else if (fileType == "png")
	{
		//弹出来晚了
		QDialog dialog(this);
		dialog.setFixedSize(220, 200);
		dialog.setWindowTitle("设置深度图参数");
		QFormLayout form(&dialog);


		QLineEdit fx, fy, cx, cy, s;
		QDoubleValidator aDoubleValidator;
		fx.setValidator(&aDoubleValidator);
		fy.setValidator(&aDoubleValidator);
		cx.setValidator(&aDoubleValidator);
		cy.setValidator(&aDoubleValidator);
		s.setValidator(&aDoubleValidator);
		fx.setText("0");
		fy.setText("0");
		cx.setText("0");
		cy.setText("0");
		s.setText("1");
		form.addRow(QString("X轴焦距= "), &fx);
		form.addRow(QString("Y轴焦距= "), &fy);
		form.addRow(QString("相机中心cx= "), &cx);
		form.addRow(QString("相机中心cy= "), &cy);
		form.addRow(QString("深度缩放因子s= "), &s);


		QDialogButtonBox buttonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel,
			Qt::Horizontal, &dialog);
		form.addRow(&buttonBox);
		QObject::connect(&buttonBox, SIGNAL(accepted()), &dialog, SLOT(accept()));
		QObject::connect(&buttonBox, SIGNAL(rejected()), &dialog, SLOT(reject()));

		if (dialog.exec() == QDialog::Rejected) {
			return;
		}

		//To do
	}*/
	
}


//直通滤波
void PointCloudLab::on_filterAction1_triggered(bool checked)
{
	// Get valid point cloud index (note: point cloud only)
	vector<int> validPoints = GetValidPointsId(POINTCLOUD_TYPE);
	if (validPoints.size() == 0) {
		QString dlgTitle = "提示";
		QString strInfo = "当前没有打开的点云";
		QMessageBox::information(this, dlgTitle, strInfo, QMessageBox::Ok, QMessageBox::NoButton);
		return;
	}

	QDialog dialog(this);
	dialog.setFixedSize(300, 200);
	dialog.setWindowTitle("直通滤波参数设置");
	QFormLayout form(&dialog);
	QComboBox curComboBox;
	for (int i = 0; i < validPoints.size(); ++i) {
		// Show valid point cloud's id on combo box
		curComboBox.addItem(QString::fromStdString(entityVector->GetId(validPoints[i])));//改成点云的名字
	}
	// Here curComboBox.currentIndex() is the index of combo box
	// Use validPoints[curComboBox.currentIndex()] to get the actual point cloud index in entityVector and entityTree 
	form.addRow(QString("选择滤波点云："), &curComboBox);

	QHBoxLayout  tempLayout1;
	QCheckBox checkBoxX("X");
	QCheckBox checkBoxY("Y");
	QCheckBox checkBoxZ("Z");
	tempLayout1.addWidget(&checkBoxX);
	tempLayout1.addWidget(&checkBoxY);
	tempLayout1.addWidget(&checkBoxZ);
	form.addRow(QString("滤波区域: "), &tempLayout1);
	QLabel tempLabel("滤波范围:");
	form.addRow(&tempLabel);
	QLineEdit textMin;
	QLineEdit textMax;
	QDoubleValidator aDoubleValidator;
	textMin.setValidator(&aDoubleValidator);
	textMax.setValidator(&aDoubleValidator);
	textMin.setText("0");
	textMax.setText("0");
	form.addRow(QString("    最小值="), &textMin);
	form.addRow(QString("    最大值= "), &textMax);

	QDialogButtonBox buttonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel,
		Qt::Horizontal, &dialog);
	form.addRow(&buttonBox);
	QObject::connect(&buttonBox, SIGNAL(accepted()), &dialog, SLOT(accept()));
	QObject::connect(&buttonBox, SIGNAL(rejected()), &dialog, SLOT(reject()));

	if (dialog.exec() == QDialog::Rejected) {
		return;
	}

	cout << "curIndx=" << curComboBox.currentIndex() << endl;
	cout << "validPoints[0]=" << validPoints[0] << endl;
	cout << "select  point :" << validPoints[curComboBox.currentIndex()] << endl;

	QString minStr = textMin.text();
	QString maxStr = textMax.text();
	double minVal = minStr.toDouble();
	double maxVal = maxStr.toDouble();
	cout << "min =" << minVal << endl;
	cout << "max =" << maxVal << endl;

	// Get selected point cloud's index
	selectedCloudIdx = validPoints[curComboBox.currentIndex()];

	// Get a pointer pointing to selected point cloud
	PointCloudT::Ptr selectedCloud = entityVector->GetCloudPtrOfIdx(selectedCloudIdx);

	// --- Start point cloud processing ---
	// YOUR CODE HERE
	if (checkBoxX.isChecked()) {
		cout << "X is checked\n";
		pcl::PassThrough<PointT> pass;//设置滤波器对象
		pass.setFilterFieldName("x");
		pass.setFilterLimits(minVal, maxVal);
		pass.setInputCloud(selectedCloud);
		pass.filter(*selectedCloud);
	}
	if (checkBoxY.isChecked()) {
		cout << "Y is checked\n";
		pcl::PassThrough<PointT> pass;//设置滤波器对象
		pass.setFilterFieldName("y");
		pass.setFilterLimits(minVal, maxVal);
		pass.setInputCloud(selectedCloud);
		pass.filter(*selectedCloud);
	}
	if (checkBoxZ.isChecked()) {
		cout << "Z is checked\n";
		pcl::PassThrough<PointT> pass;//设置滤波器对象
		pass.setFilterFieldName("y");
		pass.setFilterLimits(minVal, maxVal);
		pass.setInputCloud(selectedCloud);
		pass.filter(*selectedCloud);
	}
	// --- End point cloud processing ---

	// Update context manu
	entityTree[selectedCloudIdx]->pointsSize->setText(1, QString::number(selectedCloud->size()));

	// Update qvtkWidget display
	PointCloudVisualization *pcv = entityVector->GetPCVofIdx(selectedCloudIdx);
	pcv->Show();
	ui.qvtkWidget->update();
	
	// Push message to info widget
	PushMessage("直通滤波");
}

//体素滤波
void PointCloudLab::on_filterAction2_triggered(bool checked)
{
	cout << "体素滤波\n";
	// MARK
	vector<int> validPoints = GetValidPointsId(POINTCLOUD_TYPE);
	if (validPoints.size() == 0) {
		QString dlgTitle = "提示";
		QString strInfo = "当前没有打开的点云";
		QMessageBox::information(this, dlgTitle, strInfo, QMessageBox::Ok, QMessageBox::NoButton);
		return;
	}

	QDialog dialog(this);
	dialog.setFixedSize(300, 200);
	dialog.setWindowTitle("体素滤波参数设置");
	QFormLayout form(&dialog);
	QComboBox curComboBox;
	for (int i = 0; i < validPoints.size(); ++i) {
		curComboBox.addItem(QString::fromStdString(entityVector->GetId(validPoints[i])));//改成点云的名字
	}
	form.addRow(QString("选择滤波点云："), &curComboBox);
	QLabel tempLabel("体素大小设置:");
	form.addRow(&tempLabel);
	QLineEdit xQLine, yQLine, zQLine;
	QDoubleValidator aDoubleValidator;
	xQLine.setValidator(&aDoubleValidator);
	yQLine.setValidator(&aDoubleValidator);
	zQLine.setValidator(&aDoubleValidator);
	xQLine.setText("0");
	yQLine.setText("0");
	zQLine.setText("0");
	form.addRow(QString("    x= "), &xQLine);
	form.addRow(QString("    y="), &yQLine);
	form.addRow(QString("    z="), &zQLine);
	QDialogButtonBox buttonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel,
		Qt::Horizontal, &dialog);
	form.addRow(&buttonBox);
	QObject::connect(&buttonBox, SIGNAL(accepted()), &dialog, SLOT(accept()));
	QObject::connect(&buttonBox, SIGNAL(rejected()), &dialog, SLOT(reject()));


	if (dialog.exec() == QDialog::Rejected) {
		return;
	}

	QString xStr = xQLine.text();
	QString yStr = yQLine.text();
	QString zStr = zQLine.text();
	double x = xStr.toDouble();
	double y = yStr.toDouble();
	double z = zStr.toDouble();

	// MARK
	selectedCloudIdx = validPoints[curComboBox.currentIndex()];
	PointCloudT::Ptr selectedCloud = entityVector->GetCloudPtrOfIdx(selectedCloudIdx);

	pcl::VoxelGrid<PointT> sor;
	sor.setInputCloud(selectedCloud);
	sor.setLeafSize(x, y, z); 
	sor.filter(*selectedCloud);

	// MARK
	PointCloudVisualization *pcv = entityVector->GetPCVofIdx(selectedCloudIdx);
	pcv->Show();
	ui.qvtkWidget->update();

	PushMessage("体素滤波");
}//体素滤波

//统计滤波
void PointCloudLab::on_filterAction3_triggered(bool checked)
{
	vector<int> validPoints = GetValidPointsId(POINTCLOUD_TYPE);
	if (validPoints.size() == 0) {
		QString dlgTitle = "提示";
		QString strInfo = "当前没有打开的点云";
		QMessageBox::information(this, dlgTitle, strInfo, QMessageBox::Ok, QMessageBox::NoButton);
		return;
	}

	QDialog dialog(this);
	dialog.setFixedSize(300, 200);
	dialog.setWindowTitle("统计滤波参数设置");
	QFormLayout form(&dialog);
	QComboBox curComboBox;
	for (int i = 0; i < validPoints.size(); ++i) {
		curComboBox.addItem(QString::fromStdString(entityVector->GetId(validPoints[i])));//改成点云的名字
	}
	form.addRow(QString("选择滤波点云："), &curComboBox);
	QLineEdit neighbourNum, standerDif;

	QDoubleValidator aDoubleValidator;
	QIntValidator aIntValidator;
	neighbourNum.setValidator(&aIntValidator);
	standerDif.setValidator(&aDoubleValidator);

	neighbourNum.setText("0");
	standerDif.setText("1");

	form.addRow(QString("邻居数="), &neighbourNum);
	form.addRow(QString("标准差乘数="), &standerDif);

	QDialogButtonBox buttonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel,
		Qt::Horizontal, &dialog);
	form.addRow(&buttonBox);
	QObject::connect(&buttonBox, SIGNAL(accepted()), &dialog, SLOT(accept()));
	QObject::connect(&buttonBox, SIGNAL(rejected()), &dialog, SLOT(reject()));


	if (dialog.exec() == QDialog::Rejected) {
		return;
	}

	QString meanKStr = neighbourNum.text();
	QString stdStr = standerDif.text();
	double meanK = meanKStr.toDouble();
	double thrd = stdStr.toDouble();

	selectedCloudIdx = validPoints[curComboBox.currentIndex()];
	PointCloudT::Ptr selectedCloud = entityVector->GetCloudPtrOfIdx(selectedCloudIdx);
	pcl::StatisticalOutlierRemoval<PointT> Static;   
	Static.setInputCloud(selectedCloud);   
	Static.setMeanK(meanK);
	Static.setStddevMulThresh(thrd);
	Static.filter(*selectedCloud);                    

	PointCloudVisualization *pcv = entityVector->GetPCVofIdx(selectedCloudIdx);
	pcv->Show();
	ui.qvtkWidget->update();

	PushMessage("统计滤波");

}//统计滤波

//投影滤波
void PointCloudLab::on_filterAction4_triggered(bool checked)
{
	cout << "投影滤波\n";
	// MARK
	vector<int> validPoints = GetValidPointsId(POINTCLOUD_TYPE);
	if (validPoints.size() == 0) {
		QString dlgTitle = "提示";
		QString strInfo = "当前没有打开的点云";
		QMessageBox::information(this, dlgTitle, strInfo, QMessageBox::Ok, QMessageBox::NoButton);
		return;
	}

	QDialog dialog(this);
	dialog.setFixedSize(300, 200);
	dialog.setWindowTitle("投影滤波参数设置");
	QFormLayout form(&dialog);
	QComboBox curComboBox;
	for (int i = 0; i < validPoints.size(); ++i) {
		curComboBox.addItem(QString::fromStdString(entityVector->GetId(validPoints[i])));//改成点云的名字
	}
	form.addRow(QString("选择滤波点云："), &curComboBox);



	QLabel tempLabel("投影参数设置:ax+by+cz+d=0");
	form.addRow(&tempLabel);

	QLineEdit aQLine, bQLine, cQLine,dQLine;
	QDoubleValidator aDoubleValidator;
	aQLine.setValidator(&aDoubleValidator);
	bQLine.setValidator(&aDoubleValidator);
	cQLine.setValidator(&aDoubleValidator);
	dQLine.setValidator(&aDoubleValidator);
	aQLine.setText("0");
	bQLine.setText("0");
	cQLine.setText("0");
	dQLine.setText("0");
	form.addRow(QString("    a= "), &aQLine);
	form.addRow(QString("    b="), &bQLine);
	form.addRow(QString("    c="), &cQLine);
	form.addRow(QString("    d="), &dQLine);
	
	//按键
	QDialogButtonBox buttonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel,
		Qt::Horizontal, &dialog);
	form.addRow(&buttonBox);
	QObject::connect(&buttonBox, SIGNAL(accepted()), &dialog, SLOT(accept()));
	QObject::connect(&buttonBox, SIGNAL(rejected()), &dialog, SLOT(reject()));


	if (dialog.exec() == QDialog::Rejected) {
		return;
	}

	QString xStr = aQLine.text();
	QString yStr = bQLine.text();
	QString zStr = cQLine.text();
	QString dStr = dQLine.text();
	double a = xStr.toDouble();
	double b = yStr.toDouble();
	double c = zStr.toDouble();
	double d = dStr.toDouble();

	// MARK
	selectedCloudIdx = validPoints[curComboBox.currentIndex()];
	PointCloudT::Ptr selectedCloud = entityVector->GetCloudPtrOfIdx(selectedCloudIdx);

	projecting(selectedCloud, selectedCloud, a, b, c, d);
	/*pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
	coefficients->values.resize(4);
	coefficients->values[0] = x;
	coefficients->values[1] = y;
	coefficients->values[2] = z;
	coefficients->values[3] = d;
	pcl::ProjectInliers<PointT> proj;
	proj.setModelType(pcl::SACMODEL_PLANE);
	proj.setInputCloud(selectedCloud);
	proj.setModelCoefficients(coefficients);
	proj.filter(*selectedCloud);*/

	// MARK
	PointCloudVisualization *pcv = entityVector->GetPCVofIdx(selectedCloudIdx);
	pcv->Show();
	ui.qvtkWidget->update();

	entityTree[selectedCloudIdx]->pointsSize->setText(1, QString::number(selectedCloud->size()));

	PushMessage("投影滤波");
}

//直线拟合
void PointCloudLab::on_linefitAction_triggered(bool checked)
{
	vector<int> validPoints = GetValidPointsId();
	if (validPoints.size() == 0) {
		QString dlgTitle = "提示";
		QString strInfo = "当前没有打开的点云";
		QMessageBox::information(this, dlgTitle, strInfo, QMessageBox::Ok, QMessageBox::NoButton);
		return;
	}

	QDialog dialog(this);
	dialog.setFixedSize(300, 150);
	dialog.setWindowTitle("直线拟合参数设置");
	QFormLayout form(&dialog);
	QComboBox curComboBox;
	for (int i = 0; i < validPoints.size(); ++i) {
		curComboBox.addItem(QString::fromStdString(entityVector->GetId(validPoints[i])));//改成点云的名字
	}
	form.addRow(QString("选择拟合直线的点云："), &curComboBox);

	QHBoxLayout  tempLayout1;
	
	
	QLineEdit lineThreshold;
	
	QDoubleValidator aDoubleValidator;
	lineThreshold.setValidator(&aDoubleValidator);
	
	lineThreshold.setText("0.01");
	form.addRow(QString("直线拟合阈值="), &lineThreshold);
	

	QDialogButtonBox buttonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel,
		Qt::Horizontal, &dialog);
	form.addRow(&buttonBox);
	QObject::connect(&buttonBox, SIGNAL(accepted()), &dialog, SLOT(accept()));
	QObject::connect(&buttonBox, SIGNAL(rejected()), &dialog, SLOT(reject()));


	if (dialog.exec() == QDialog::Rejected) {
		return;
	}


	selectedCloudIdx = validPoints[curComboBox.currentIndex()];
	PointCloudT::Ptr selectedCloud = entityVector->GetCloudPtrOfIdx(selectedCloudIdx);
	
	QString linStr =lineThreshold.text();
	double linthrld = linStr.toDouble();
	PointCloudT::Ptr linecloud(new PointCloudT);

	pcl::ModelCoefficients::Ptr lineCoefficients(new pcl::ModelCoefficients);
	lineCoefficients=linefitting(selectedCloud, linecloud, linthrld);

	float a = lineCoefficients->values[0];
	float b = lineCoefficients->values[1];
	float c = lineCoefficients->values[2];
	float d = lineCoefficients->values[3];
	float e = lineCoefficients->values[4];
	float f = lineCoefficients->values[5];

	std::string id = "line";
	int idx = entityVector->AddPointCloud(linecloud, id);
	PointCloudVisualization *pcv = entityVector->GetPCVofIdx(idx);
	pcv->Show();
	viewer->resetCamera();
	ui.qvtkWidget->update();

	completeCloud->AddCloud(linecloud);

	// Todo: edit type, point num, face num
	PointTree * tempTree = new PointTree(&ui, id, "PointCloud", pcv->GetPointNum(), 0);
	entityTree.push_back(tempTree);
	stringstream aa,bb,cc,dd,ee,ff;
	aa << a; bb << b; cc << c; dd << d; ee << e; ff << f;
	std::string lineEquation = "(x - "+aa.str()+")/"+dd.str()+"=(y-"+bb.str()+")/"+ee.str()+"=(z-"+cc.str()+")/"+ff.str();
	
	PushMessage("拟合直线方程：（--为 + ）");
	PushMessage(lineEquation);
}

//平面拟合
void PointCloudLab::on_planefitAction_triggered(bool checked)
{
	vector<int> validPoints = GetValidPointsId();
	if (validPoints.size() == 0) {
		QString dlgTitle = "提示";
		QString strInfo = "当前没有打开的点云";
		QMessageBox::information(this, dlgTitle, strInfo, QMessageBox::Ok, QMessageBox::NoButton);
		return;
	}

	QDialog dialog(this);
	dialog.setFixedSize(300, 150);
	dialog.setWindowTitle("平面拟合参数设置");
	QFormLayout form(&dialog);
	QComboBox curComboBox;
	for (int i = 0; i < validPoints.size(); ++i) {
		curComboBox.addItem(QString::fromStdString(entityVector->GetId(validPoints[i])));//改成点云的名字
	}
	form.addRow(QString("选择拟合平面的点云："), &curComboBox);

	QHBoxLayout  tempLayout1;


	QLineEdit planeThreshold;

	QDoubleValidator aDoubleValidator;
	planeThreshold.setValidator(&aDoubleValidator);

	planeThreshold.setText("0.01");
	form.addRow(QString("平面拟合阈值="), &planeThreshold);


	QDialogButtonBox buttonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel,
		Qt::Horizontal, &dialog);
	form.addRow(&buttonBox);
	QObject::connect(&buttonBox, SIGNAL(accepted()), &dialog, SLOT(accept()));
	QObject::connect(&buttonBox, SIGNAL(rejected()), &dialog, SLOT(reject()));


	if (dialog.exec() == QDialog::Rejected) {
		return;
	}


	selectedCloudIdx = validPoints[curComboBox.currentIndex()];
	PointCloudT::Ptr selectedCloud = entityVector->GetCloudPtrOfIdx(selectedCloudIdx);

	QString plnStr = planeThreshold.text();
	double plnthrld = plnStr.toDouble();
	PointCloudT::Ptr planecloud(new PointCloudT);

	pcl::ModelCoefficients::Ptr planeCoefficients(new pcl::ModelCoefficients);
	planeCoefficients = planefitting(selectedCloud, planecloud, plnthrld);

	float a = planeCoefficients->values[0];
	float b = planeCoefficients->values[1];
	float c = planeCoefficients->values[2];
	float d = planeCoefficients->values[3];
	

	std::string id = "plane";
	int idx = entityVector->AddPointCloud(planecloud, id);
	PointCloudVisualization *pcv = entityVector->GetPCVofIdx(idx);
	pcv->Show();
	viewer->resetCamera();
	ui.qvtkWidget->update();

	completeCloud->AddCloud(planecloud);

	PointTree * tempTree = new PointTree(&ui, id, "PointCloud", pcv->GetPointNum(), 0);
	entityTree.push_back(tempTree);

	//pcl::io::savePCDFile("C:/Users/sjtuzhy/Desktop/plane.pcd", *planecloud);
	stringstream aa, bb, cc, dd;
	aa << a; bb << b; cc << c; dd << d; 
	std::string planeEquation =aa.str() + "*x+ " +  bb.str() + "*y+" + cc.str() + "*z+" + dd.str()+  "=0" ;

	PushMessage("拟合平面方程：（--为 + ）");
	PushMessage(planeEquation);
}

//球面拟合
void PointCloudLab::on_ballfitAction_triggered(bool checked)
{
	vector<int> validPoints = GetValidPointsId();
	if (validPoints.size() == 0) {
		QString dlgTitle = "提示";
		QString strInfo = "当前没有打开的点云";
		QMessageBox::information(this, dlgTitle, strInfo, QMessageBox::Ok, QMessageBox::NoButton);
		return;
	}

	QDialog dialog(this);
	dialog.setFixedSize(300, 150);
	dialog.setWindowTitle("球面拟合参数设置");
	QFormLayout form(&dialog);
	QComboBox curComboBox;
	for (int i = 0; i < validPoints.size(); ++i) {
		curComboBox.addItem(QString::fromStdString(entityVector->GetId(validPoints[i])));//改成点云的名字
	}
	form.addRow(QString("选择拟合球面的点云："), &curComboBox);

	QHBoxLayout  tempLayout1;


	QLineEdit planeThreshold;

	QDoubleValidator aDoubleValidator;
	planeThreshold.setValidator(&aDoubleValidator);

	planeThreshold.setText("0.01");
	form.addRow(QString("球面拟合阈值="), &planeThreshold);


	QDialogButtonBox buttonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel,
		Qt::Horizontal, &dialog);
	form.addRow(&buttonBox);
	QObject::connect(&buttonBox, SIGNAL(accepted()), &dialog, SLOT(accept()));
	QObject::connect(&buttonBox, SIGNAL(rejected()), &dialog, SLOT(reject()));


	if (dialog.exec() == QDialog::Rejected) {
		return;
	}


	selectedCloudIdx = validPoints[curComboBox.currentIndex()];
	PointCloudT::Ptr selectedCloud = entityVector->GetCloudPtrOfIdx(selectedCloudIdx);

	QString plnStr = planeThreshold.text();
	double plnthrld = plnStr.toDouble();
	PointCloudT::Ptr ballcloud(new PointCloudT);

	pcl::ModelCoefficients::Ptr ballCoefficients(new pcl::ModelCoefficients);
	ballCoefficients = ballfitting(selectedCloud, ballcloud, plnthrld);

	float a = ballCoefficients->values[0];
	float b = ballCoefficients->values[1];
	float c = ballCoefficients->values[2];
	float d = ballCoefficients->values[3];


	std::string id = "ball";
	int idx = entityVector->AddPointCloud(ballcloud, id);
	PointCloudVisualization *pcv = entityVector->GetPCVofIdx(idx);
	pcv->Show();
	viewer->resetCamera();
	ui.qvtkWidget->update();

	completeCloud->AddCloud(ballcloud);

	PointTree * tempTree = new PointTree(&ui, id, "PointCloud", pcv->GetPointNum(), 0);
	entityTree.push_back(tempTree);

	stringstream aa, bb, cc, dd;
	aa << a; bb << b; cc << c; dd << d;
	std::string ballEquation = "拟合球面圆心为：("+aa.str() + ", " + bb.str() + "," + cc.str() + ")\n 半径为：" + dd.str() + "mm";

	
	PushMessage(ballEquation);
}

//平面度计算
void PointCloudLab::on_pmdAction_triggered(bool checked)
{
	vector<int> validPoints = GetValidPointsId();
	if (validPoints.size() == 0) {
		QString dlgTitle = "提示";
		QString strInfo = "当前没有打开的点云";
		QMessageBox::information(this, dlgTitle, strInfo, QMessageBox::Ok, QMessageBox::NoButton);
		return;
	}

	QDialog dialog(this);
	dialog.setFixedSize(300, 100);
	dialog.setWindowTitle("平面度计算");
	QFormLayout form(&dialog);
	QComboBox curComboBox;
	for (int i = 0; i < validPoints.size(); ++i) {
		curComboBox.addItem(QString::fromStdString(entityVector->GetId(validPoints[i])));//改成点云的名字
	}
	form.addRow(QString("选择平面点云："), &curComboBox);


	QDialogButtonBox buttonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel,
		Qt::Horizontal, &dialog);
	form.addRow(&buttonBox);
	QObject::connect(&buttonBox, SIGNAL(accepted()), &dialog, SLOT(accept()));
	QObject::connect(&buttonBox, SIGNAL(rejected()), &dialog, SLOT(reject()));


	if (dialog.exec() == QDialog::Rejected) {
		return;
	}


	selectedCloudIdx = validPoints[curComboBox.currentIndex()];
	PointCloudT::Ptr selectedCloud = entityVector->GetCloudPtrOfIdx(selectedCloudIdx);


	
	float p;
	p = pmd(selectedCloud);
	stringstream pp;
	pp << p;
	std::string pm="平面度为："+pp.str();

	PushMessage(pm);
}


//边缘提取
void PointCloudLab::on_boundaryAction_triggered(bool checked)
{
	vector<int> validPoints = GetValidPointsId();
	if (validPoints.size() == 0) {
		QString dlgTitle = "提示";
		QString strInfo = "当前没有打开的点云";
		QMessageBox::information(this, dlgTitle, strInfo, QMessageBox::Ok, QMessageBox::NoButton);
		return;
	}

	QDialog dialog(this);
	dialog.setFixedSize(300, 100);
	dialog.setWindowTitle("边缘提取");
	QFormLayout form(&dialog);
	QComboBox curComboBox;
	for (int i = 0; i < validPoints.size(); ++i) {
		curComboBox.addItem(QString::fromStdString(entityVector->GetId(validPoints[i])));//改成点云的名字
	}
	form.addRow(QString("选择边缘提取的点云："), &curComboBox);


	QDialogButtonBox buttonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel,
		Qt::Horizontal, &dialog);
	form.addRow(&buttonBox);
	QObject::connect(&buttonBox, SIGNAL(accepted()), &dialog, SLOT(accept()));
	QObject::connect(&buttonBox, SIGNAL(rejected()), &dialog, SLOT(reject()));


	if (dialog.exec() == QDialog::Rejected) {
		return;
	}


	selectedCloudIdx = validPoints[curComboBox.currentIndex()];
	PointCloudT::Ptr selectedCloud = entityVector->GetCloudPtrOfIdx(selectedCloudIdx);

	
	PointCloudT::Ptr boundarycloud(new PointCloudT);

	boundary(selectedCloud, boundarycloud);

	


	std::string id = "boundary";
	int idx = entityVector->AddPointCloud(boundarycloud, id);
	PointCloudVisualization *pcv = entityVector->GetPCVofIdx(idx);
	pcv->Show();
	viewer->resetCamera();
	ui.qvtkWidget->update();

	completeCloud->AddCloud(boundarycloud);

	// Todo: edit type, point num, face num
	PointTree * tempTree = new PointTree(&ui, id, "PointCloud", pcv->GetPointNum(), 0);
	entityTree.push_back(tempTree);
	

	PushMessage("边缘提取成功");

}


//点云粗配准
void PointCloudLab::on_match1Action_triggered(bool checked)
{
	vector<int> validPoints = GetValidPointsId();
	if (validPoints.size() < 2) {
		QString dlgTitle = "提示";
		QString strInfo = "当前打开的点云数量小于2";
		QMessageBox::information(this, dlgTitle, strInfo, QMessageBox::Ok, QMessageBox::NoButton);
		return;
	}

	QDialog dialog(this);
	dialog.setFixedSize(300, 100);
	dialog.setWindowTitle("点云粗配准");
	QFormLayout form(&dialog);
	QComboBox curComboBox;
	for (int i = 0; i < validPoints.size(); ++i) {
		curComboBox.addItem(QString::fromStdString(entityVector->GetId(validPoints[i])));//改成点云的名字
	}
	form.addRow(QString("选择源点云："), &curComboBox);

	QComboBox curComboBox2;
	for (int i = 0; i < validPoints.size(); ++i) {
		curComboBox2.addItem(QString::fromStdString(entityVector->GetId(validPoints[i])));//改成点云的名字
	}
	form.addRow(QString("选择目标点云："), &curComboBox2);


	QDialogButtonBox buttonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel,
		Qt::Horizontal, &dialog);
	form.addRow(&buttonBox);
	QObject::connect(&buttonBox, SIGNAL(accepted()), &dialog, SLOT(accept()));
	QObject::connect(&buttonBox, SIGNAL(rejected()), &dialog, SLOT(reject()));


	if (dialog.exec() == QDialog::Rejected) {
		return;
	}


	selectedCloudIdx = validPoints[curComboBox.currentIndex()];
	PointCloudT::Ptr selectedCloud = entityVector->GetCloudPtrOfIdx(selectedCloudIdx);
	selectedCloudIdx = validPoints[curComboBox2.currentIndex()];
	PointCloudT::Ptr selectedCloud2 = entityVector->GetCloudPtrOfIdx(selectedCloudIdx);

	PointCloudT::Ptr sac_result(new PointCloudT);
	Eigen::Matrix4f sac_trans = Eigen::Matrix4f::Identity();
	sac_ia(selectedCloud, selectedCloud2, sac_result,sac_trans);
	
	std::cout << sac_result->size();
	std::string id = "sac_result cloud";
	int idx = entityVector->AddPointCloud(sac_result, id);
	PointCloudVisualization *pcv = entityVector->GetPCVofIdx(idx);
	pcv->Show();
	viewer->resetCamera();
	ui.qvtkWidget->update();

	completeCloud->AddCloud(sac_result);
	PointTree * tempTree = new PointTree(&ui, id, "PointCloud", pcv->GetPointNum(), 0);
	entityTree.push_back(tempTree);

	stringstream pp;
	pp << sac_trans;
	std::string trans = "粗配准结果，变换矩阵为：" + pp.str();

	PushMessage(trans);

}


//点云精配准
void PointCloudLab::on_match2Action_triggered(bool checked)
{
	vector<int> validPoints = GetValidPointsId();
	if (validPoints.size() < 2) {
		QString dlgTitle = "提示";
		QString strInfo = "当前打开的点云数量小于2";
		QMessageBox::information(this, dlgTitle, strInfo, QMessageBox::Ok, QMessageBox::NoButton);
		return;
	}

	QDialog dialog(this);
	dialog.setFixedSize(300, 100);
	dialog.setWindowTitle("点云精配准");
	QFormLayout form(&dialog);
	QComboBox curComboBox;
	for (int i = 0; i < validPoints.size(); ++i) {
		curComboBox.addItem(QString::fromStdString(entityVector->GetId(validPoints[i])));//改成点云的名字
	}
	form.addRow(QString("选择源点云："), &curComboBox);

	QComboBox curComboBox2;
	for (int i = 0; i < validPoints.size(); ++i) {
		curComboBox2.addItem(QString::fromStdString(entityVector->GetId(validPoints[i])));//改成点云的名字
	}
	form.addRow(QString("选择目标点云："), &curComboBox2);


	QDialogButtonBox buttonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel,
		Qt::Horizontal, &dialog);
	form.addRow(&buttonBox);
	QObject::connect(&buttonBox, SIGNAL(accepted()), &dialog, SLOT(accept()));
	QObject::connect(&buttonBox, SIGNAL(rejected()), &dialog, SLOT(reject()));


	if (dialog.exec() == QDialog::Rejected) {
		return;
	}


	selectedCloudIdx = validPoints[curComboBox.currentIndex()];
	PointCloudT::Ptr selectedCloud = entityVector->GetCloudPtrOfIdx(selectedCloudIdx);
	selectedCloudIdx = validPoints[curComboBox2.currentIndex()];
	PointCloudT::Ptr selectedCloud2 = entityVector->GetCloudPtrOfIdx(selectedCloudIdx);

	PointCloudT::Ptr icp_result(new PointCloudT);
	Eigen::Matrix4f icp_trans = Eigen::Matrix4f::Identity();
	icp(selectedCloud, selectedCloud2, icp_result, icp_trans);

	std::cout << icp_result->size();
	std::string id = "icp_result cloud";
	int idx = entityVector->AddPointCloud(icp_result, id);
	PointCloudVisualization *pcv = entityVector->GetPCVofIdx(idx);
	pcv->Show();
	viewer->resetCamera();
	ui.qvtkWidget->update();

	completeCloud->AddCloud(icp_result);
	PointTree * tempTree = new PointTree(&ui, id, "PointCloud", pcv->GetPointNum(), 0);
	entityTree.push_back(tempTree);

	stringstream pp;
	pp << icp_trans;
	std::string trans = "精配准结果，变换矩阵为：" + pp.str();

	PushMessage(trans);
}

//点云拼接
void PointCloudLab::on_match3Action_triggered(bool checked)
{
	vector<int> validPoints = GetValidPointsId();
	if (validPoints.size() < 2) {
		QString dlgTitle = "提示";
		QString strInfo = "当前打开的点云数量小于2";
		QMessageBox::information(this, dlgTitle, strInfo, QMessageBox::Ok, QMessageBox::NoButton);
		return;
	}

	QDialog dialog0(this);
	dialog0.setFixedSize(300, 100);
	dialog0.setWindowTitle("点云拼接数量");
	QFormLayout form(&dialog0);

	QHBoxLayout  tempLayout1;

	QLineEdit cloudnum;

	QIntValidator aIntValidator;
	cloudnum.setValidator(&aIntValidator);

	cloudnum.setText("2");
	form.addRow(QString("拼接数量为2或3："), &cloudnum);

	QDialogButtonBox buttonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel,Qt::Horizontal, &dialog0);
	form.addRow(&buttonBox);
	QObject::connect(&buttonBox, SIGNAL(accepted()), &dialog0, SLOT(accept()));
	QObject::connect(&buttonBox, SIGNAL(rejected()), &dialog0, SLOT(reject()));
	if (dialog0.exec() == QDialog::Rejected) {
		return;
	}

	QString numStr = cloudnum.text();
	int cloudn = numStr.toInt();

	if (cloudn == 2)
	{
		QDialog dialog(this);
		dialog.setFixedSize(300, 100);
		dialog.setWindowTitle("点云拼接");
		QFormLayout form(&dialog);

		QComboBox curComboBox;
		for (int i = 0; i < validPoints.size(); ++i) {
			curComboBox.addItem(QString::fromStdString(entityVector->GetId(validPoints[i])));//改成点云的名字
		}
		form.addRow(QString("选择第一个点云："), &curComboBox);

		QComboBox curComboBox2;
		for (int i = 0; i < validPoints.size(); ++i) {
			curComboBox2.addItem(QString::fromStdString(entityVector->GetId(validPoints[i])));//改成点云的名字
		}
		form.addRow(QString("选择第二个点云："), &curComboBox2);

		QDialogButtonBox buttonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel,
			Qt::Horizontal, &dialog);
		form.addRow(&buttonBox);
		QObject::connect(&buttonBox, SIGNAL(accepted()), &dialog, SLOT(accept()));
		QObject::connect(&buttonBox, SIGNAL(rejected()), &dialog, SLOT(reject()));


		if (dialog.exec() == QDialog::Rejected) {
			return;
		}

		selectedCloudIdx = validPoints[curComboBox.currentIndex()];
		PointCloudT::Ptr selectedCloud = entityVector->GetCloudPtrOfIdx(selectedCloudIdx);
		selectedCloudIdx = validPoints[curComboBox2.currentIndex()];
		PointCloudT::Ptr selectedCloud2 = entityVector->GetCloudPtrOfIdx(selectedCloudIdx);

		PointCloudT::Ptr cloudadd2(new PointCloudT);

		*cloudadd2 = *selectedCloud + *selectedCloud2;
		std::string id = "cloud_splice2";
		int idx = entityVector->AddPointCloud(cloudadd2, id);
		PointCloudVisualization *pcv = entityVector->GetPCVofIdx(idx);
		pcv->Show();
		viewer->resetCamera();
		ui.qvtkWidget->update();

		completeCloud->AddCloud(cloudadd2);
		PointTree * tempTree = new PointTree(&ui, id, "PointCloud", pcv->GetPointNum(), 0);
		entityTree.push_back(tempTree);

		std::string msg = "两块点云拼接完成";
		PushMessage(msg);
	}

	else if (cloudn == 3)
	{
		QDialog dialog(this);
		dialog.setFixedSize(300, 100);
		dialog.setWindowTitle("点云拼接");
		QFormLayout form(&dialog);

		QComboBox curComboBox;
		for (int i = 0; i < validPoints.size(); ++i) {
			curComboBox.addItem(QString::fromStdString(entityVector->GetId(validPoints[i])));//改成点云的名字
		}
		form.addRow(QString("选择第一个点云："), &curComboBox);

		QComboBox curComboBox2;
		for (int i = 0; i < validPoints.size(); ++i) {
			curComboBox2.addItem(QString::fromStdString(entityVector->GetId(validPoints[i])));//改成点云的名字
		}
		form.addRow(QString("选择第二个点云："), &curComboBox2);

		QComboBox curComboBox3;
		for (int i = 0; i < validPoints.size(); ++i) {
			curComboBox3.addItem(QString::fromStdString(entityVector->GetId(validPoints[i])));//改成点云的名字
		}
		form.addRow(QString("选择第三个点云："), &curComboBox3);

		QDialogButtonBox buttonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel,
			Qt::Horizontal, &dialog);
		form.addRow(&buttonBox);
		QObject::connect(&buttonBox, SIGNAL(accepted()), &dialog, SLOT(accept()));
		QObject::connect(&buttonBox, SIGNAL(rejected()), &dialog, SLOT(reject()));


		if (dialog.exec() == QDialog::Rejected) {
			return;
		}

		selectedCloudIdx = validPoints[curComboBox.currentIndex()];
		PointCloudT::Ptr selectedCloud = entityVector->GetCloudPtrOfIdx(selectedCloudIdx);
		selectedCloudIdx = validPoints[curComboBox2.currentIndex()];
		PointCloudT::Ptr selectedCloud2 = entityVector->GetCloudPtrOfIdx(selectedCloudIdx);
		selectedCloudIdx = validPoints[curComboBox3.currentIndex()];
		PointCloudT::Ptr selectedCloud3 = entityVector->GetCloudPtrOfIdx(selectedCloudIdx);

		PointCloudT::Ptr cloudadd3(new PointCloudT);

		*cloudadd3 = *selectedCloud + *selectedCloud2;
		*cloudadd3 = *cloudadd3 + *selectedCloud3;
		std::string id = "cloud_splice3";
		int idx = entityVector->AddPointCloud(cloudadd3, id);
		PointCloudVisualization *pcv = entityVector->GetPCVofIdx(idx);
		pcv->Show();
		viewer->resetCamera();
		ui.qvtkWidget->update();

		completeCloud->AddCloud(cloudadd3);
		PointTree * tempTree = new PointTree(&ui, id, "PointCloud", pcv->GetPointNum(), 0);
		entityTree.push_back(tempTree);

		std::string msg = "三块点云拼接完成";
		PushMessage(msg);
	}
	else
	{
		QString dlgTitle = "提示";
		QString strInfo = "拼接点云数量只能为2或3";
		QMessageBox::information(this, dlgTitle, strInfo, QMessageBox::Ok, QMessageBox::NoButton);
		return;
	}

}




 //ROI复制点云
void PointCloudLab::on_copyPointAction_triggered(bool checked)
{
    cout << "复制点云\n";
	if (setSelected.empty() || selectedCloudIdx == -1)
		return;

	std::string id = "copy_" + entityVector->GetId(selectedCloudIdx);
	PointCloudT::Ptr tempPtr(new PointCloudT);
	*tempPtr = *clicked_points_3d;
	int idx = entityVector->AddPointCloud(tempPtr, id);

	PointTree * tempTree = new PointTree(&ui, id, "PointXYZ", tempPtr->size(), 0);
	entityTree.push_back(tempTree);

	entityTree[idx]->cloudName->setTextColor(0, QColor(150, 150, 150));
	entityTree[idx]->pointsSize->setTextColor(0, QColor(150, 150, 150));
	entityTree[idx]->faceSize->setTextColor(0, QColor(150, 150, 150));
	entityTree[idx]->cloudName->setTextColor(1, QColor(150, 150, 150));
	entityTree[idx]->pointsSize->setTextColor(1, QColor(150, 150, 150));
	entityTree[idx]->faceSize->setTextColor(1, QColor(150, 150, 150));
	
	motionState = DRAG;
	ui.qvtkWidget->setFocus();
	ui.qvtkWidget->show();
	QKeyEvent keyPress(QEvent::KeyPress, Qt::Key_X, Qt::NoModifier, "x");
	QKeyEvent keyRelease(QEvent::KeyRelease, Qt::Key_X, Qt::NoModifier);
	QCoreApplication::sendEvent(ui.qvtkWidget, &keyPress);
	QCoreApplication::sendEvent(ui.qvtkWidget, &keyRelease);
	PushMessage("复制点云完成，按 Drag 键结束");
}


//ROI提取点云
void PointCloudLab::on_extractPointAction_triggered(bool checked)
{
    cout << "提取点云\n";
	if (setSelected.empty() || selectedCloudIdx == -1)
		return;

	std::string id = "extract_" + entityVector->GetId(selectedCloudIdx);
	PointCloudT::Ptr tempPtr(new PointCloudT);
	*tempPtr = *clicked_points_3d;
	int idx = entityVector->AddPointCloud(tempPtr, id);

	PointTree * tempTree = new PointTree(&ui, id, "PointXYZ", tempPtr->size(), 0);
	entityTree.push_back(tempTree);

	entityTree[idx]->cloudName->setTextColor(0, QColor(150, 150, 150));
	entityTree[idx]->pointsSize->setTextColor(0, QColor(150, 150, 150));
	entityTree[idx]->faceSize->setTextColor(0, QColor(150, 150, 150));
	entityTree[idx]->cloudName->setTextColor(1, QColor(150, 150, 150));
	entityTree[idx]->pointsSize->setTextColor(1, QColor(150, 150, 150));
	entityTree[idx]->faceSize->setTextColor(1, QColor(150, 150, 150));

	motionState = DRAG;
	ui.qvtkWidget->setFocus();
	ui.qvtkWidget->show();
	QKeyEvent keyPress(QEvent::KeyPress, Qt::Key_X, Qt::NoModifier, "x");
	QKeyEvent keyRelease(QEvent::KeyRelease, Qt::Key_X, Qt::NoModifier);
	QCoreApplication::sendEvent(ui.qvtkWidget, &keyPress);
	QCoreApplication::sendEvent(ui.qvtkWidget, &keyRelease);

	//Point oriCloudPtr = pointCloudVector->GetCloudPtrOfIdx(selectedCloudIdx);
	PointCloudVisualization *pcv = entityVector->GetPCVofIdx(selectedCloudIdx);
	pcv->DeletePointFromVector(setSelected);
	ui.qvtkWidget->update();

	entityTree[selectedCloudIdx]->pointsSize->setText(0, QString("点数: ") + QString::number(pcv->GetPointNum()));
	PushMessage("提取点云完成，按 Drag 键结束");
}


void PointCloudLab::OnShowAction()
{
    cout << "show points  " << curPointsId << endl;
	if (!entityVector->IsValid(curPointsId))
		return;
    if (entityVector->IsShown(curPointsId))
        return;
    //todo 显示点云
	PointCloudVisualization *pcv = entityVector->GetPCVofIdx(curPointsId);
	assert(pcv != nullptr);
	pcv->Show();
	ui.qvtkWidget->update();


    entityTree[curPointsId]->cloudName->setTextColor(0, QColor(0, 0, 0));
	entityTree[curPointsId]->pointsSize->setTextColor(0, QColor(0, 0, 0));
	entityTree[curPointsId]->faceSize->setTextColor(0, QColor(0, 0, 0));
	entityTree[curPointsId]->cloudName->setTextColor(1, QColor(0, 0, 0));
	entityTree[curPointsId]->pointsSize->setTextColor(1, QColor(0, 0, 0));
    entityTree[curPointsId]->faceSize->setTextColor(1, QColor(0, 0, 0));
}
void PointCloudLab::OnHideAction()
{
    cout << "hide points " << curPointsId << endl;
	if (!entityVector->IsValid(curPointsId))
		return;
	if (!entityVector->IsShown(curPointsId))
		return;

	PointCloudVisualization *pcv = entityVector->GetPCVofIdx(curPointsId);
	assert(pcv != nullptr);
	pcv->Hide();
	ui.qvtkWidget->update();
   
    entityTree[curPointsId]->cloudName->setTextColor(0, QColor(150, 150, 150));
	entityTree[curPointsId]->pointsSize->setTextColor(0, QColor(150, 150, 150));
	entityTree[curPointsId]->faceSize->setTextColor(0, QColor(150, 150, 150));
	entityTree[curPointsId]->cloudName->setTextColor(1, QColor(150, 150, 150));
	entityTree[curPointsId]->pointsSize->setTextColor(1, QColor(150, 150, 150));
    entityTree[curPointsId]->faceSize->setTextColor(1, QColor(150, 150, 150));
}
void PointCloudLab::OnDeleteAction()
{
	QDialog dialog(this);
	dialog.setFixedSize(250, 50);
	dialog.setWindowTitle("是否确认删除");
	QFormLayout form(&dialog);

	QDialogButtonBox buttonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel,Qt::Horizontal, &dialog);
	form.addRow(&buttonBox);
	QObject::connect(&buttonBox, SIGNAL(accepted()), &dialog, SLOT(accept()));
	QObject::connect(&buttonBox, SIGNAL(rejected()), &dialog, SLOT(reject()));


	if (dialog.exec() == QDialog::Rejected) {
		return;
	}


	// Todo: Release memory in point cloud vector
    if (entityTree[curPointsId] != nullptr) {
        delete entityTree[curPointsId];
        entityTree[curPointsId] = nullptr;
		entityVector->DeleteEntity(curPointsId);
		ui.qvtkWidget->update();
		PushMessage("删除成功");
    }
    cout << "delete points " << curPointsId << endl;

}

void PointCloudLab::OnSetColorAction()
{
    cout << "set color " << curPointsId << endl;
	if (entityTree[curPointsId]->type == "Mesh") {
		MeshVisualization *meshv = entityVector->GetMESHVofIdx(curPointsId);
		vector<int> currColor = meshv->GetColor();
		QColor  iniColor = QColor(currColor[0], currColor[1], currColor[2]); //现有的文字颜色
		QColor color = QColorDialog::getColor(iniColor, this, "选择颜色");
		if (color.isValid()) //选择有效
		{
			cout << "set color =R:" << color.red() << endl;
			cout << "set color =G:" << color.green() << endl;
			cout << "set color =B:" << color.blue() << endl;

			meshv->SetColor(color.red(), color.green(), color.blue());
		}

	}
	else {
		PointCloudVisualization *pcv = entityVector->GetPCVofIdx(curPointsId);
		vector<int> currColor = pcv->GetColor();
		QColor  iniColor = QColor(currColor[0], currColor[1], currColor[2]); //现有的文字颜色
		QColor color = QColorDialog::getColor(iniColor, this, "选择颜色");
		if (color.isValid()) //选择有效
		{
			cout << "set color =R:" << color.red() << endl;
			cout << "set color =G:" << color.green() << endl;
			cout << "set color =B:" << color.blue() << endl;

			pcv->SetColor(color.red(), color.green(), color.blue());
		}
	}

}

void PointCloudLab::OnSaveCurPointAction()
{
	cout << "save points " << curPointsId << endl;
	QString curPath = QCoreApplication::applicationDirPath();
	QString dlgTitle = "保存点云"; //对话框标题
	QString filter = "pcd文件(*.pcd);;ply文件(*.ply);;obj文件(*.obj);;stl文件(*.stl);;mesh文件(*.mesh);;png文件(*.png);;所有文件(*.*)"; //文件过滤器
	QString fileName = QFileDialog::getSaveFileName(this, dlgTitle, curPath, filter);
	if (fileName.isEmpty()) {
		return;
	}

	QFileInfo file(fileName);
	if (file.exists()) {
		QFile::remove(fileName);
	}

	QTextCodec *code = QTextCodec::codecForName("GB2312");//解决中文路径问题
	std::string filePath = code->fromUnicode(fileName).data();
	vector<std::string> temp = split(filePath, ".");
	std::string fileType = temp.back();

	if (fileType == "pcd" ||  fileType == "obj" || fileType == "stl" || fileType == "mesh")
	{
		entityVector->SavePointCloudOfIdx(filePath, curPointsId);
	}
	else if (fileType == "ply") {
		entityVector->SaveMeshOfIdx(filePath, curPointsId);
	}
	else if (fileType == "png")
	{
		QDialog dialog(this);
		dialog.setFixedSize(220, 200);
		dialog.setWindowTitle("设置深度图参数");
		QFormLayout form(&dialog);


		QLineEdit fx, fy, cx, cy, s;
		QDoubleValidator aDoubleValidator;
		fx.setValidator(&aDoubleValidator);
		fy.setValidator(&aDoubleValidator);
		cx.setValidator(&aDoubleValidator);
		cy.setValidator(&aDoubleValidator);
		s.setValidator(&aDoubleValidator);
		fx.setText("0");
		fy.setText("0");
		cx.setText("0");
		cy.setText("0");
		s.setText("1");
		form.addRow(QString("X轴焦距= "), &fx);
		form.addRow(QString("Y轴焦距= "), &fy);
		form.addRow(QString("相机中心cx= "), &cx);
		form.addRow(QString("相机中心cy= "), &cy);
		form.addRow(QString("深度缩放因子s= "), &s);


		QDialogButtonBox buttonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel,
			Qt::Horizontal, &dialog);
		form.addRow(&buttonBox);
		QObject::connect(&buttonBox, SIGNAL(accepted()), &dialog, SLOT(accept()));
		QObject::connect(&buttonBox, SIGNAL(rejected()), &dialog, SLOT(reject()));

		if (dialog.exec() == QDialog::Rejected) {
			return;
		}

		//To do
	}


	PushMessage("保存点云");

}

// Interation:
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
	cout << "pre  ss X to strat or ending picking, then press 'Q'..." << endl;
	
	// Simulate a keypress "X" to start area picking
	ui.qvtkWidget->setFocus();
	ui.qvtkWidget->show();
	QKeyEvent keyPress(QEvent::KeyPress, Qt::Key_X, Qt::NoModifier, "x");
	QKeyEvent keyRelease(QEvent::KeyRelease, Qt::Key_X, Qt::NoModifier);
	QCoreApplication::sendEvent(ui.qvtkWidget, &keyPress);
	QCoreApplication::sendEvent(ui.qvtkWidget, &keyRelease);

	cloud_mutex.unlock();
}

void PointCloudLab::on_pushButton_allSelect_clicked() {
	if (selectedCloudIdx == -1) {
		// Todo: Select all pointcloud
		//*clicked_points_3d = *completeCloud->GetCloudPtr();
		return;
	}
	else {
		PointCloudT::Ptr currCloud = entityVector->GetCloudPtrOfIdx(selectedCloudIdx);
		*clicked_points_3d = *currCloud;
		unordered_set<int>().swap(setSelected);
		for (int i = 0; i < currCloud->size(); ++i) {
				setSelected.insert(i);
		}
	}

	pcl::visualization::PointCloudColorHandlerCustom<PointT> red(clicked_points_3d, 255, 0, 0);
	viewer->removePointCloud("clicked_points");
	viewer->addPointCloud(clicked_points_3d, red, "clicked_points");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "clicked_points");
	cout << "clicked_points_3d->points.size()" << clicked_points_3d->points.size() << endl;
	ui.qvtkWidget->update();
}

void PointCloudLab::on_pushButton_invertSelect_clicked() {
	cout << "clicked" << endl;
	if (selectedCloudIdx == -1) {
		return;
	}
	PointCloudT::Ptr currCloud = entityVector->GetCloudPtrOfIdx(selectedCloudIdx);
	clicked_points_3d.reset(new PointCloudT);
	unordered_set<int> setTemp;
	for (int i = 0; i < currCloud->size(); ++i) {
		if (setSelected.find(i) == setSelected.end()) {
			clicked_points_3d->points.push_back(currCloud->points.at(i));
			setTemp.insert(i);
		}
	}
	setSelected.swap(setTemp);

	pcl::visualization::PointCloudColorHandlerCustom<PointT> red(clicked_points_3d, 255, 0, 0);
	viewer->removePointCloud("clicked_points");
	viewer->addPointCloud(clicked_points_3d, red, "clicked_points");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "clicked_points");
	cout << "clicked_points_3d->points.size()" << clicked_points_3d->points.size() << endl;
	ui.qvtkWidget->update();
}

void PointCloudLab::point_callback(const pcl::visualization::PointPickingEvent& event, void* args) {
	//struct callback_args* data = (struct callback_args *)args;
	PointCloudLab *p = (PointCloudLab *)args;
	if (p->motionState != POINT_PICK)
		return;

	PointT current_point;
	//event.getPoint(current_point.x, current_point.y, current_point.z);
	int idx = event.getPointIndex();
	if (idx == -1)
		return;
	PointCloudT::Ptr currCloud = p->entityVector->GetCloudPtrOfIdx(p->selectedCloudIdx);
	//p->clicked_points_3d->points.push_back(current_point);
	p->clicked_points_3d->points.push_back(currCloud->points.at(idx));

	pcl::visualization::PointCloudColorHandlerCustom<PointT> red(p->clicked_points_3d, 255, 0, 0);

	p->viewer->removePointCloud("clicked_points");
	p->viewer->addPointCloud(p->clicked_points_3d, red, "clicked_points");
	p->viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "clicked_points");
	p->ui.qvtkWidget->update();	
}

void PointCloudLab::area_callback(const pcl::visualization::AreaPickingEvent& event, void *args) {
	PointCloudLab *p = (PointCloudLab *)args;

	if (p->motionState != AREA_PICK)
		return;
	if (!p->entityVector->IsValid(p->selectedCloudIdx))
		return;
	
	PointCloudT::Ptr currCloud = p->entityVector->GetCloudPtrOfIdx(p->selectedCloudIdx);

	
	if (!p->ui.checkBox_undo->isChecked()) {
		vector<int>().swap(p->selectedPointIdxs);
		if (event.getPointsIndices(p->selectedPointIdxs) == false)
			return;

		for (size_t i = 0; i < p->selectedPointIdxs.size(); i++)
		{
			//p->clicked_points_3d->points.push_back(baseCloud->points.at(indices[i]));
			int idx = p->selectedPointIdxs[i];
			p->setSelected.insert(idx);
		}
	}
	else {
		vector<int>().swap(p->selectedPointIdxs);
		if (event.getPointsIndices(p->selectedPointIdxs) == false)
			return;
		for (size_t i = 0; i < p->selectedPointIdxs.size(); i++)
		{
			int idx = p->selectedPointIdxs[i];
			if (p->setSelected.find(idx) == p->setSelected.end())
				continue;
			p->setSelected.erase(idx);
		}
	}
	p->clicked_points_3d.reset(new PointCloudT);
	if (!p->setSelected.size()) {
		p->viewer->removePointCloud("clicked_points");
		p->ui.qvtkWidget->update();
		return;
	}
	for (auto i = p->setSelected.begin(); i != p->setSelected.end(); ++i)
		p->clicked_points_3d->points.push_back(currCloud->points.at(*i));

	pcl::visualization::PointCloudColorHandlerCustom<PointT> red(p->clicked_points_3d, 255, 0, 0);
	p->viewer->removePointCloud("clicked_points");
	p->viewer->addPointCloud(p->clicked_points_3d, red, "clicked_points");
	p->viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "clicked_points");
	cout << "clicked_points_3d->points.size()" << p->clicked_points_3d->points.size() << endl;
	p->ui.qvtkWidget->update();
	//if (event.getPointIndex() == -1)
	//	return;
}
void PointCloudLab::on_pushButton_pointPick_clicked() {
	cout << "-- Point pick mode -- " << endl;
	vector<int> validPoints = GetValidPointsId(POINTCLOUD_TYPE);
	if (validPoints.size() == 0) {
		QString dlgTitle = "提示";
		QString strInfo = "当前没有打开的点云";
		QMessageBox::information(this, dlgTitle, strInfo, QMessageBox::Ok, QMessageBox::NoButton);
		return;
	}

	QDialog dialog(this);
	dialog.setFixedSize(300, 100);
	dialog.setWindowTitle("点选点云选择");
	QFormLayout form(&dialog);
	QComboBox curComboBox;
	for (int i = 0; i < validPoints.size(); ++i) {
		curComboBox.addItem(QString::fromStdString(entityVector->GetId(validPoints[i])));//改成点云的名字
	}
	form.addRow(QString("选择点选点云："), &curComboBox);

	QDialogButtonBox buttonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel,
		Qt::Horizontal, &dialog);
	form.addRow(&buttonBox);
	QObject::connect(&buttonBox, SIGNAL(accepted()), &dialog, SLOT(accept()));
	QObject::connect(&buttonBox, SIGNAL(rejected()), &dialog, SLOT(reject()));


	if (dialog.exec() == QDialog::Rejected) {
		return;
	}

	selectedCloudIdx = validPoints[curComboBox.currentIndex()];
	int n = entityVector->GetSize();
	for (int i = 0; i < n; ++i) {
		if (!entityVector->IsValid(i)) {
			continue;
		}
		if (i == selectedCloudIdx) {
			PointCloudVisualization* pcv = entityVector->GetPCVofIdx(i);
			assert(pcv != nullptr);
			pcv->Show();
			ui.qvtkWidget->update();

			entityTree[i]->cloudName->setTextColor(0, QColor(0, 0, 0));
			entityTree[i]->pointsSize->setTextColor(0, QColor(0, 0, 0));
			entityTree[i]->faceSize->setTextColor(0, QColor(0, 0, 0));
			entityTree[i]->cloudName->setTextColor(1, QColor(0, 0, 0));
			entityTree[i]->pointsSize->setTextColor(1, QColor(0, 0, 0));
			entityTree[i]->faceSize->setTextColor(1, QColor(0, 0, 0));
			continue;
		}
		if (entityVector->IsShown(i)) {
			PointCloudVisualization* pcv = entityVector->GetPCVofIdx(i);
			assert(pcv != nullptr);
			pcv->Hide();
			ui.qvtkWidget->update();

			entityTree[i]->cloudName->setTextColor(0, QColor(150, 150, 150));
			entityTree[i]->pointsSize->setTextColor(0, QColor(150, 150, 150));
			entityTree[i]->faceSize->setTextColor(0, QColor(150, 150, 150));
			entityTree[i]->cloudName->setTextColor(1, QColor(150, 150, 150));
			entityTree[i]->pointsSize->setTextColor(1, QColor(150, 150, 150));
			entityTree[i]->faceSize->setTextColor(1, QColor(150, 150, 150));
		}
	}
	motionState = POINT_PICK;
	if (selectedCloud != nullptr) {
		delete selectedCloud;
		selectedCloud = nullptr;
	}
	PointPicking();
}

void PointCloudLab::on_pushButton_areaPick_clicked() {

	cout << "-- Area pick mode -- " << endl;
	vector<int> validPoints = GetValidPointsId(POINTCLOUD_TYPE);
	if (validPoints.size() == 0) {
		QString dlgTitle = "提示";
		QString strInfo = "当前没有打开的点云";
		QMessageBox::information(this, dlgTitle, strInfo, QMessageBox::Ok, QMessageBox::NoButton);
		return;
	}

	QDialog dialog(this);
	dialog.setFixedSize(300, 100);
	dialog.setWindowTitle("框选点云选择");
	QFormLayout form(&dialog);
	QComboBox curComboBox;
	for (int i = 0; i < validPoints.size(); ++i) {
		curComboBox.addItem(QString::fromStdString(entityVector->GetId(validPoints[i])));//改成点云的名字
	}
	form.addRow(QString("选择框选点云："), &curComboBox);

	QDialogButtonBox buttonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel,
		Qt::Horizontal, &dialog);
	form.addRow(&buttonBox);
	QObject::connect(&buttonBox, SIGNAL(accepted()), &dialog, SLOT(accept()));
	QObject::connect(&buttonBox, SIGNAL(rejected()), &dialog, SLOT(reject()));


	if (dialog.exec() == QDialog::Rejected) {
		return;
	}

	PushMessage("按“X”键 开始/结束 框选点云");
	if (validPoints[curComboBox.currentIndex()] != selectedCloudIdx) {
		clicked_points_3d.reset(new PointCloudT);
		vector<int>().swap(selectedPointIdxs);
		unordered_set<int>().swap(setSelected);
		viewer->removePointCloud("clicked_points");
		ui.qvtkWidget->update();
	}
	selectedCloudIdx = validPoints[curComboBox.currentIndex()];
	int n = entityVector->GetSize();
	for (int i = 0; i < n; ++i) {
		if (!entityVector->IsValid(i)) {
			continue;
		}
		if (i == selectedCloudIdx) {
			PointCloudVisualization* pcv = entityVector->GetPCVofIdx(i);
			assert(pcv != nullptr);
			pcv->Show();
			ui.qvtkWidget->update();

			entityTree[i]->cloudName->setTextColor(0, QColor(0, 0, 0));
			entityTree[i]->pointsSize->setTextColor(0, QColor(0, 0, 0));
			entityTree[i]->faceSize->setTextColor(0, QColor(0, 0, 0));
			entityTree[i]->cloudName->setTextColor(1, QColor(0, 0, 0));
			entityTree[i]->pointsSize->setTextColor(1, QColor(0, 0, 0));
			entityTree[i]->faceSize->setTextColor(1, QColor(0, 0, 0));
			continue;
		}
		if (entityVector->IsShown(i)) {
			PointCloudVisualization* pcv = entityVector->GetPCVofIdx(i);
			assert(pcv != nullptr);
			pcv->Hide();
			ui.qvtkWidget->update();

			entityTree[i]->cloudName->setTextColor(0, QColor(150, 150, 150));
			entityTree[i]->pointsSize->setTextColor(0, QColor(150, 150, 150));
			entityTree[i]->faceSize->setTextColor(0, QColor(150, 150, 150));
			entityTree[i]->cloudName->setTextColor(1, QColor(150, 150, 150));
			entityTree[i]->pointsSize->setTextColor(1, QColor(150, 150, 150));
			entityTree[i]->faceSize->setTextColor(1, QColor(150, 150, 150));
		}
	}

	motionState = AREA_PICK;
	AreaPicking();
	//ui.qvtkWidget->setFocus();
	//QKeyEvent keyPress(QEvent::KeyPress, Qt::Key_X, Qt::NoModifier);
	//QKeyEvent keyRelease(QEvent::KeyReleas e, Qt::Key_X, Qt::NoModifier);

	//QApplication::sendEvent(ui.qvtkWidget, &keyPress);
	//QApplication::sendEvent(ui.qvtkWidget, &keyRelease);

}

void PointCloudLab::on_pushButton_drag_clicked() {
	motionState = DRAG;
	selectedCloudIdx = -1;
	if (clicked_points_3d == nullptr)
		return;
	int n = clicked_points_3d->size();
	cout << "Remove " << n << " selected points" << endl;
	clicked_points_3d.reset(new PointCloudT);
	vector<int>().swap(selectedPointIdxs);
	unordered_set<int>().swap(setSelected);
	viewer->removePointCloud("clicked_points");
	//viewer->registerAreaPickingCallback(nullptr, nullptr);
	//viewer->registerPointPickingCallback(nullptr, nullptr);
	ui.qvtkWidget->update();

}

void PointCloudLab::on_pushButton_setting_clicked() {
	QDialog dialog(this);
	dialog.setFixedSize(320, 240);
	dialog.setWindowTitle("显示设置");
	QFormLayout form(&dialog);

	QHBoxLayout  tempLayout1;
	QSpinBox r, g, b;
	r.setRange(0, 255);
	g.setRange(0, 255);
	b.setRange(0, 255);
	r.setValue(backGroundColor[0]);
	g.setValue(backGroundColor[1]);
	b.setValue(backGroundColor[2]);
	tempLayout1.addWidget(&r);
	tempLayout1.addWidget(&g);
	tempLayout1.addWidget(&b);
	form.addRow(QString("背景颜色: "), &tempLayout1);

	QSpinBox pointSizeBox;
	pointSizeBox.setValue(pointViewerSize);
	pointSizeBox.setRange(1, 10);

	form.addRow(QString("点大小: "), &pointSizeBox);

	QCheckBox checkBox1("显示坐标轴");
	checkBox1.setChecked(isCoordinateSystemShown);
	form.addRow(&checkBox1);

	QDoubleSpinBox coordSizeBox;
	coordSizeBox.setValue(coordinateSystemSize);
	coordSizeBox.setRange(0.0001, 100000);

	form.addRow(QString("坐标轴大小: "), &coordSizeBox);

	QCheckBox checkBox2("显示尺寸(mm)");
	checkBox2.setChecked(isCoordinateSystemSizeShown);
	form.addRow(&checkBox2);

	QDialogButtonBox buttonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel,
		Qt::Horizontal, &dialog);
	form.addRow(&buttonBox);
	QObject::connect(&buttonBox, SIGNAL(accepted()), &dialog, SLOT(accept()));
	QObject::connect(&buttonBox, SIGNAL(rejected()), &dialog, SLOT(reject()));


	if (dialog.exec() == QDialog::Rejected) {
		return;
	}

	pointViewerSize = pointSizeBox.value();
	backGroundColor[0] = r.value();
	backGroundColor[1] = g.value();
	backGroundColor[2] = b.value();
	isCoordinateSystemShown = checkBox1.isChecked();
	isCoordinateSystemSizeShown = checkBox2.isChecked();
	coordinateSystemSize = coordSizeBox.value();

	// Set point size
	vector<int> validPoints = GetValidPointsId(POINTCLOUD_TYPE);
	for (int i : validPoints) {
		PointCloudVisualization* pcv = entityVector->GetPCVofIdx(i);
		pcv->SetPointSize(pointViewerSize);
	}

	viewer->setBackgroundColor(backGroundColor[0] / 255.0, backGroundColor[1] / 255.0, backGroundColor[2] / 255.0);
	if (isCoordinateSystemShown) {
		viewer->addCoordinateSystem(coordinateSystemSize);
	}
	else {
		viewer->removeCoordinateSystem();
	}

}

void PointCloudLab::on_pushButton_clicked() {
	cout << "complete cloud size: " << completeCloud->GetPointNum() << endl;
	if (completeCloud->IsShown()) {
		completeCloud->Hide();
	}
	else {
		completeCloud->Show();
	}
	ui.qvtkWidget->update();
}

vector<int> PointCloudLab::GetValidPointsId()
{
	vector<int> validPoints;
	for (int i = 0; i < entityVector->GetSize(); ++i) {
		if (entityVector->IsValid(i)) {
			validPoints.push_back(i);
		}
	}

	return validPoints;
}

vector<int> PointCloudLab::GetValidPointsId(string type)
{
	vector<int> validPoints;
	for (int i = 0; i < entityVector->GetSize(); ++i) {
		if (entityVector->IsValid(i) && entityVector->GetType(i) == type) {
			validPoints.push_back(i);
		}
	}

	return validPoints;
}

void PointCloudLab::on_newBtn_clicked()
{
	PushMessage("clicked newBtn");
	//ui.newBtn->setText("11");
}