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
                treeMenu->exec(QCursor::pos());   //�˵�����λ��Ϊ�����λ��
               
                break;
            }
        }
    }
    event->accept();
}


void PointCloudLab::InitMenuAction()
{
	treeMenu = new QMenu(ui.treeWidget);
	showAction = new QAction("��ʾ", ui.treeWidget);
	hideAction = new QAction("����", ui.treeWidget);
	deleteAction = new QAction("ɾ������", ui.treeWidget);
	setColorAction = new QAction("������ɫ", ui.treeWidget);
	saveCurPointAction = new QAction("�������", ui.treeWidget);

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
    ui.treeWidget->setColumnCount(2); //��������
    ui.treeWidget->setHeaderHidden(false);
	QStringList header;
	header.push_back(QString("����"));
	header.push_back(QString("����"));
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
    //�Ƚ�Ҫ�и���ַ�����string����ת��Ϊchar*����  
    char * strs = new char[str.length() + 1]; //��Ҫ����  
    strcpy(strs, str.c_str());

    char * d = new char[delim.length() + 1];
    strcpy(d, delim.c_str());

    char *p = strtok(strs, d);
    while (p) {
        std::string s = p; //�ָ�õ����ַ���ת��Ϊstring����  
        res.push_back(s); //����������  
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


	if (pcl::io::loadPolygonFileOBJ(path, mesh) == -1)//*�򿪵����ļ�  pathΪobj�ļ���
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

//stl,mesh,png ���ܶ���pointcloudxyz��ʽ
int PointCloudLab::OpenStlFile(std::string path)
{
	return CANCEL;
}
int PointCloudLab::OpenMeshFile(std::string path)
{
	return CANCEL;
}



//png��ʽ ���Զ��ɾ���

int PointCloudLab::OpenPngFile(std::string path)
{
	QDialog dialog(this);
	//dialog.setFixedSize(200, 200);
	dialog.setFixedSize(200, 200);
	dialog.setWindowTitle("�������ͼ����");
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
	form.addRow(QString("X�ό��= "), &fx);
	form.addRow(QString("Y�ό��= "), &fy);
	form.addRow(QString("�������cx= "), &cx);
	form.addRow(QString("�������cy= "), &cy);
	form.addRow(QString("�����������s= "), &s);
	

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
	QCheckBox checkBoxX("�Ƿ������ɫ");
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
		QString dlgTitle = "���ļ�"; //�Ի������
		QString filter = "png�ļ�(*.png)"; //�ļ�������
		QString fileName = QFileDialog::getOpenFileName(this, dlgTitle, curPath, filter);

		/*if (fileName.isEmpty())
		{
		return;
		}*/

		QTextCodec *code = QTextCodec::codecForName("GB2312");//�������·������
		std::string filePath = code->fromUnicode(fileName).data();


		cv::Mat depth, rgb;
		rgb = cv::imread(filePath, -1);
		depth = cv::imread(path, -1);
		PointCloudT::Ptr cloud(new PointCloudT());

		for (int m = 0; m < depth.rows; m++)
			for (int n = 0; n < depth.cols; n++)
			{
				// ��ȡ���ͼ��(m,n)����ֵ
				ushort d = depth.ptr<ushort>(m)[n];
				// d ����û��ֵ������ˣ������˵�
				if (d == 0)
					continue;
				// d ����ֵ�������������һ����
				pcl::PointXYZRGBA p;

				// ���������Ŀռ�����
				p.z = double(d) / camera_factor;
				p.x = (n - camera_cx) * p.z / camera_fx;
				p.y = (m - camera_cy) * p.z / camera_fy;

				// ��rgbͼ���л�ȡ������ɫ
				// rgb����ͨ����BGR��ʽͼ�����԰������˳���ȡ��ɫ
				p.b = rgb.ptr<uchar>(m)[n * 3];
				p.g = rgb.ptr<uchar>(m)[n * 3 + 1];
				p.r = rgb.ptr<uchar>(m)[n * 3 + 2];

				// ��p���뵽������
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
				// ��ȡ���ͼ��(m,n)����ֵ
				ushort d = depth.ptr<ushort>(m)[n];
				// d ����û��ֵ������ˣ������˵�
				if (d == 0)
					continue;
				// d ����ֵ�������������һ����
				pcl::PointXYZRGBA p;

				// ���������Ŀռ�����
				p.z = double(d) / camera_factor;
				p.x = (n - camera_cx) * p.z / camera_fx;
				p.y = (m - camera_cy) * p.z / camera_fy;


				// ��p���뵽������
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
    QString dlgTitle = "���ļ�"; //�Ի������
    QString filter = "pcd�ļ�(*.pcd);;ply�ļ�(*.ply);;obj�ļ�(*.obj);;stl�ļ�(*.stl);;png�ļ�(*.png);;�����ļ�(*.*)"; //�ļ�������
    QString fileName = QFileDialog::getOpenFileName(this, dlgTitle, curPath, filter);

    if (fileName.isEmpty())
    {
        return;
    }

    QTextCodec *code = QTextCodec::codecForName("GB2312");//�������·������
    std::string filePath = code->fromUnicode(fileName).data();

	int ret = OpenFile(filePath);
	if (ret == SUCCESS)
	{
		PushMessage("�򿪳ɹ�");
	}
	else if (ret == FAILED)
	{
		PushMessage("��ʧ��");
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

//���Ʊ���
void PointCloudLab::on_saveFileAction_triggered(bool checked)
{
	// Todo: check valid point
	vector<int> validPoints = GetValidPointsId();
	if (validPoints.size() == 0) {
		QString dlgTitle = "��ʾ";
		QString strInfo = "��ǰû�д򿪵ĵ���";
		QMessageBox::information(this, dlgTitle, strInfo, QMessageBox::Ok, QMessageBox::NoButton);
		return;
	}

	QDialog dialog(this);
	dialog.setFixedSize(300, 100);
	dialog.setWindowTitle("ѡ��Ҫ����ĵ���");
	QFormLayout form(&dialog);
	QComboBox curComboBox;
	for (int i = 0; i < validPoints.size(); ++i) {
		curComboBox.addItem(QString::fromStdString(entityVector->GetId(validPoints[i])));//�ĳɵ��Ƶ�����
	}
	form.addRow(QString("ѡ����ƣ�"), &curComboBox);

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
	QString dlgTitle = "�������"; //�Ի������
	QString filter = "pcd�ļ�(*.pcd);;ply�ļ�(*.ply);;obj�ļ�(*.obj);;stl�ļ�(*.stl);;�����ļ�(*.*)"; //�ļ������� mesh�ļ�(*.mesh);;png�ļ�(*.png);;
	QString fileName = QFileDialog::getSaveFileName(this, dlgTitle, curPath, filter);
	
	if (fileName.isEmpty()) {
		return;
	}

	QFileInfo file(fileName);
	if (file.exists()) {
		QFile::remove(fileName);
	}

	QTextCodec *code = QTextCodec::codecForName("GB2312");//�������·������
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
		std::cout << "�ɹ�" << '\n';
		PushMessage("����ɹ�");
	}
	else if (ret == FAILED)
	{
		std::cout << "ʧ��" << '\n';
		PushMessage("����ʧ��");
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
	PushMessage("����ɹ�");
	/*else if (fileType == "png")
	{
		//����������
		QDialog dialog(this);
		dialog.setFixedSize(220, 200);
		dialog.setWindowTitle("�������ͼ����");
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
		form.addRow(QString("X�ό��= "), &fx);
		form.addRow(QString("Y�ό��= "), &fy);
		form.addRow(QString("�������cx= "), &cx);
		form.addRow(QString("�������cy= "), &cy);
		form.addRow(QString("�����������s= "), &s);


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


//ֱͨ�˲�
void PointCloudLab::on_filterAction1_triggered(bool checked)
{
	// Get valid point cloud index (note: point cloud only)
	vector<int> validPoints = GetValidPointsId(POINTCLOUD_TYPE);
	if (validPoints.size() == 0) {
		QString dlgTitle = "��ʾ";
		QString strInfo = "��ǰû�д򿪵ĵ���";
		QMessageBox::information(this, dlgTitle, strInfo, QMessageBox::Ok, QMessageBox::NoButton);
		return;
	}

	QDialog dialog(this);
	dialog.setFixedSize(300, 200);
	dialog.setWindowTitle("ֱͨ�˲���������");
	QFormLayout form(&dialog);
	QComboBox curComboBox;
	for (int i = 0; i < validPoints.size(); ++i) {
		// Show valid point cloud's id on combo box
		curComboBox.addItem(QString::fromStdString(entityVector->GetId(validPoints[i])));//�ĳɵ��Ƶ�����
	}
	// Here curComboBox.currentIndex() is the index of combo box
	// Use validPoints[curComboBox.currentIndex()] to get the actual point cloud index in entityVector and entityTree 
	form.addRow(QString("ѡ���˲����ƣ�"), &curComboBox);

	QHBoxLayout  tempLayout1;
	QCheckBox checkBoxX("X");
	QCheckBox checkBoxY("Y");
	QCheckBox checkBoxZ("Z");
	tempLayout1.addWidget(&checkBoxX);
	tempLayout1.addWidget(&checkBoxY);
	tempLayout1.addWidget(&checkBoxZ);
	form.addRow(QString("�˲�����: "), &tempLayout1);
	QLabel tempLabel("�˲���Χ:");
	form.addRow(&tempLabel);
	QLineEdit textMin;
	QLineEdit textMax;
	QDoubleValidator aDoubleValidator;
	textMin.setValidator(&aDoubleValidator);
	textMax.setValidator(&aDoubleValidator);
	textMin.setText("0");
	textMax.setText("0");
	form.addRow(QString("    ��Сֵ="), &textMin);
	form.addRow(QString("    ���ֵ= "), &textMax);

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
		pcl::PassThrough<PointT> pass;//�����˲�������
		pass.setFilterFieldName("x");
		pass.setFilterLimits(minVal, maxVal);
		pass.setInputCloud(selectedCloud);
		pass.filter(*selectedCloud);
	}
	if (checkBoxY.isChecked()) {
		cout << "Y is checked\n";
		pcl::PassThrough<PointT> pass;//�����˲�������
		pass.setFilterFieldName("y");
		pass.setFilterLimits(minVal, maxVal);
		pass.setInputCloud(selectedCloud);
		pass.filter(*selectedCloud);
	}
	if (checkBoxZ.isChecked()) {
		cout << "Z is checked\n";
		pcl::PassThrough<PointT> pass;//�����˲�������
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
	PushMessage("ֱͨ�˲�");
}

//�����˲�
void PointCloudLab::on_filterAction2_triggered(bool checked)
{
	cout << "�����˲�\n";
	// MARK
	vector<int> validPoints = GetValidPointsId(POINTCLOUD_TYPE);
	if (validPoints.size() == 0) {
		QString dlgTitle = "��ʾ";
		QString strInfo = "��ǰû�д򿪵ĵ���";
		QMessageBox::information(this, dlgTitle, strInfo, QMessageBox::Ok, QMessageBox::NoButton);
		return;
	}

	QDialog dialog(this);
	dialog.setFixedSize(300, 200);
	dialog.setWindowTitle("�����˲���������");
	QFormLayout form(&dialog);
	QComboBox curComboBox;
	for (int i = 0; i < validPoints.size(); ++i) {
		curComboBox.addItem(QString::fromStdString(entityVector->GetId(validPoints[i])));//�ĳɵ��Ƶ�����
	}
	form.addRow(QString("ѡ���˲����ƣ�"), &curComboBox);
	QLabel tempLabel("���ش�С����:");
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

	PushMessage("�����˲�");
}//�����˲�

//ͳ���˲�
void PointCloudLab::on_filterAction3_triggered(bool checked)
{
	vector<int> validPoints = GetValidPointsId(POINTCLOUD_TYPE);
	if (validPoints.size() == 0) {
		QString dlgTitle = "��ʾ";
		QString strInfo = "��ǰû�д򿪵ĵ���";
		QMessageBox::information(this, dlgTitle, strInfo, QMessageBox::Ok, QMessageBox::NoButton);
		return;
	}

	QDialog dialog(this);
	dialog.setFixedSize(300, 200);
	dialog.setWindowTitle("ͳ���˲���������");
	QFormLayout form(&dialog);
	QComboBox curComboBox;
	for (int i = 0; i < validPoints.size(); ++i) {
		curComboBox.addItem(QString::fromStdString(entityVector->GetId(validPoints[i])));//�ĳɵ��Ƶ�����
	}
	form.addRow(QString("ѡ���˲����ƣ�"), &curComboBox);
	QLineEdit neighbourNum, standerDif;

	QDoubleValidator aDoubleValidator;
	QIntValidator aIntValidator;
	neighbourNum.setValidator(&aIntValidator);
	standerDif.setValidator(&aDoubleValidator);

	neighbourNum.setText("0");
	standerDif.setText("1");

	form.addRow(QString("�ھ���="), &neighbourNum);
	form.addRow(QString("��׼�����="), &standerDif);

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

	PushMessage("ͳ���˲�");

}//ͳ���˲�

//ͶӰ�˲�
void PointCloudLab::on_filterAction4_triggered(bool checked)
{
	cout << "ͶӰ�˲�\n";
	// MARK
	vector<int> validPoints = GetValidPointsId(POINTCLOUD_TYPE);
	if (validPoints.size() == 0) {
		QString dlgTitle = "��ʾ";
		QString strInfo = "��ǰû�д򿪵ĵ���";
		QMessageBox::information(this, dlgTitle, strInfo, QMessageBox::Ok, QMessageBox::NoButton);
		return;
	}

	QDialog dialog(this);
	dialog.setFixedSize(300, 200);
	dialog.setWindowTitle("ͶӰ�˲���������");
	QFormLayout form(&dialog);
	QComboBox curComboBox;
	for (int i = 0; i < validPoints.size(); ++i) {
		curComboBox.addItem(QString::fromStdString(entityVector->GetId(validPoints[i])));//�ĳɵ��Ƶ�����
	}
	form.addRow(QString("ѡ���˲����ƣ�"), &curComboBox);



	QLabel tempLabel("ͶӰ��������:ax+by+cz+d=0");
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
	
	//����
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

	PushMessage("ͶӰ�˲�");
}

//ֱ�����
void PointCloudLab::on_linefitAction_triggered(bool checked)
{
	vector<int> validPoints = GetValidPointsId();
	if (validPoints.size() == 0) {
		QString dlgTitle = "��ʾ";
		QString strInfo = "��ǰû�д򿪵ĵ���";
		QMessageBox::information(this, dlgTitle, strInfo, QMessageBox::Ok, QMessageBox::NoButton);
		return;
	}

	QDialog dialog(this);
	dialog.setFixedSize(300, 150);
	dialog.setWindowTitle("ֱ����ϲ�������");
	QFormLayout form(&dialog);
	QComboBox curComboBox;
	for (int i = 0; i < validPoints.size(); ++i) {
		curComboBox.addItem(QString::fromStdString(entityVector->GetId(validPoints[i])));//�ĳɵ��Ƶ�����
	}
	form.addRow(QString("ѡ�����ֱ�ߵĵ��ƣ�"), &curComboBox);

	QHBoxLayout  tempLayout1;
	
	
	QLineEdit lineThreshold;
	
	QDoubleValidator aDoubleValidator;
	lineThreshold.setValidator(&aDoubleValidator);
	
	lineThreshold.setText("0.01");
	form.addRow(QString("ֱ�������ֵ="), &lineThreshold);
	

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
	
	PushMessage("���ֱ�߷��̣���--Ϊ + ��");
	PushMessage(lineEquation);
}

//ƽ�����
void PointCloudLab::on_planefitAction_triggered(bool checked)
{
	vector<int> validPoints = GetValidPointsId();
	if (validPoints.size() == 0) {
		QString dlgTitle = "��ʾ";
		QString strInfo = "��ǰû�д򿪵ĵ���";
		QMessageBox::information(this, dlgTitle, strInfo, QMessageBox::Ok, QMessageBox::NoButton);
		return;
	}

	QDialog dialog(this);
	dialog.setFixedSize(300, 150);
	dialog.setWindowTitle("ƽ����ϲ�������");
	QFormLayout form(&dialog);
	QComboBox curComboBox;
	for (int i = 0; i < validPoints.size(); ++i) {
		curComboBox.addItem(QString::fromStdString(entityVector->GetId(validPoints[i])));//�ĳɵ��Ƶ�����
	}
	form.addRow(QString("ѡ�����ƽ��ĵ��ƣ�"), &curComboBox);

	QHBoxLayout  tempLayout1;


	QLineEdit planeThreshold;

	QDoubleValidator aDoubleValidator;
	planeThreshold.setValidator(&aDoubleValidator);

	planeThreshold.setText("0.01");
	form.addRow(QString("ƽ�������ֵ="), &planeThreshold);


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

	PushMessage("���ƽ�淽�̣���--Ϊ + ��");
	PushMessage(planeEquation);
}

//�������
void PointCloudLab::on_ballfitAction_triggered(bool checked)
{
	vector<int> validPoints = GetValidPointsId();
	if (validPoints.size() == 0) {
		QString dlgTitle = "��ʾ";
		QString strInfo = "��ǰû�д򿪵ĵ���";
		QMessageBox::information(this, dlgTitle, strInfo, QMessageBox::Ok, QMessageBox::NoButton);
		return;
	}

	QDialog dialog(this);
	dialog.setFixedSize(300, 150);
	dialog.setWindowTitle("������ϲ�������");
	QFormLayout form(&dialog);
	QComboBox curComboBox;
	for (int i = 0; i < validPoints.size(); ++i) {
		curComboBox.addItem(QString::fromStdString(entityVector->GetId(validPoints[i])));//�ĳɵ��Ƶ�����
	}
	form.addRow(QString("ѡ���������ĵ��ƣ�"), &curComboBox);

	QHBoxLayout  tempLayout1;


	QLineEdit planeThreshold;

	QDoubleValidator aDoubleValidator;
	planeThreshold.setValidator(&aDoubleValidator);

	planeThreshold.setText("0.01");
	form.addRow(QString("���������ֵ="), &planeThreshold);


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
	std::string ballEquation = "�������Բ��Ϊ��("+aa.str() + ", " + bb.str() + "," + cc.str() + ")\n �뾶Ϊ��" + dd.str() + "mm";

	
	PushMessage(ballEquation);
}

//ƽ��ȼ���
void PointCloudLab::on_pmdAction_triggered(bool checked)
{
	vector<int> validPoints = GetValidPointsId();
	if (validPoints.size() == 0) {
		QString dlgTitle = "��ʾ";
		QString strInfo = "��ǰû�д򿪵ĵ���";
		QMessageBox::information(this, dlgTitle, strInfo, QMessageBox::Ok, QMessageBox::NoButton);
		return;
	}

	QDialog dialog(this);
	dialog.setFixedSize(300, 100);
	dialog.setWindowTitle("ƽ��ȼ���");
	QFormLayout form(&dialog);
	QComboBox curComboBox;
	for (int i = 0; i < validPoints.size(); ++i) {
		curComboBox.addItem(QString::fromStdString(entityVector->GetId(validPoints[i])));//�ĳɵ��Ƶ�����
	}
	form.addRow(QString("ѡ��ƽ����ƣ�"), &curComboBox);


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
	std::string pm="ƽ���Ϊ��"+pp.str();

	PushMessage(pm);
}


//��Ե��ȡ
void PointCloudLab::on_boundaryAction_triggered(bool checked)
{
	vector<int> validPoints = GetValidPointsId();
	if (validPoints.size() == 0) {
		QString dlgTitle = "��ʾ";
		QString strInfo = "��ǰû�д򿪵ĵ���";
		QMessageBox::information(this, dlgTitle, strInfo, QMessageBox::Ok, QMessageBox::NoButton);
		return;
	}

	QDialog dialog(this);
	dialog.setFixedSize(300, 100);
	dialog.setWindowTitle("��Ե��ȡ");
	QFormLayout form(&dialog);
	QComboBox curComboBox;
	for (int i = 0; i < validPoints.size(); ++i) {
		curComboBox.addItem(QString::fromStdString(entityVector->GetId(validPoints[i])));//�ĳɵ��Ƶ�����
	}
	form.addRow(QString("ѡ���Ե��ȡ�ĵ��ƣ�"), &curComboBox);


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
	

	PushMessage("��Ե��ȡ�ɹ�");

}


//���ƴ���׼
void PointCloudLab::on_match1Action_triggered(bool checked)
{
	vector<int> validPoints = GetValidPointsId();
	if (validPoints.size() < 2) {
		QString dlgTitle = "��ʾ";
		QString strInfo = "��ǰ�򿪵ĵ�������С��2";
		QMessageBox::information(this, dlgTitle, strInfo, QMessageBox::Ok, QMessageBox::NoButton);
		return;
	}

	QDialog dialog(this);
	dialog.setFixedSize(300, 100);
	dialog.setWindowTitle("���ƴ���׼");
	QFormLayout form(&dialog);
	QComboBox curComboBox;
	for (int i = 0; i < validPoints.size(); ++i) {
		curComboBox.addItem(QString::fromStdString(entityVector->GetId(validPoints[i])));//�ĳɵ��Ƶ�����
	}
	form.addRow(QString("ѡ��Դ���ƣ�"), &curComboBox);

	QComboBox curComboBox2;
	for (int i = 0; i < validPoints.size(); ++i) {
		curComboBox2.addItem(QString::fromStdString(entityVector->GetId(validPoints[i])));//�ĳɵ��Ƶ�����
	}
	form.addRow(QString("ѡ��Ŀ����ƣ�"), &curComboBox2);


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
	std::string trans = "����׼������任����Ϊ��" + pp.str();

	PushMessage(trans);

}


//���ƾ���׼
void PointCloudLab::on_match2Action_triggered(bool checked)
{
	vector<int> validPoints = GetValidPointsId();
	if (validPoints.size() < 2) {
		QString dlgTitle = "��ʾ";
		QString strInfo = "��ǰ�򿪵ĵ�������С��2";
		QMessageBox::information(this, dlgTitle, strInfo, QMessageBox::Ok, QMessageBox::NoButton);
		return;
	}

	QDialog dialog(this);
	dialog.setFixedSize(300, 100);
	dialog.setWindowTitle("���ƾ���׼");
	QFormLayout form(&dialog);
	QComboBox curComboBox;
	for (int i = 0; i < validPoints.size(); ++i) {
		curComboBox.addItem(QString::fromStdString(entityVector->GetId(validPoints[i])));//�ĳɵ��Ƶ�����
	}
	form.addRow(QString("ѡ��Դ���ƣ�"), &curComboBox);

	QComboBox curComboBox2;
	for (int i = 0; i < validPoints.size(); ++i) {
		curComboBox2.addItem(QString::fromStdString(entityVector->GetId(validPoints[i])));//�ĳɵ��Ƶ�����
	}
	form.addRow(QString("ѡ��Ŀ����ƣ�"), &curComboBox2);


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
	std::string trans = "����׼������任����Ϊ��" + pp.str();

	PushMessage(trans);
}

//����ƴ��
void PointCloudLab::on_match3Action_triggered(bool checked)
{
	vector<int> validPoints = GetValidPointsId();
	if (validPoints.size() < 2) {
		QString dlgTitle = "��ʾ";
		QString strInfo = "��ǰ�򿪵ĵ�������С��2";
		QMessageBox::information(this, dlgTitle, strInfo, QMessageBox::Ok, QMessageBox::NoButton);
		return;
	}

	QDialog dialog0(this);
	dialog0.setFixedSize(300, 100);
	dialog0.setWindowTitle("����ƴ������");
	QFormLayout form(&dialog0);

	QHBoxLayout  tempLayout1;

	QLineEdit cloudnum;

	QIntValidator aIntValidator;
	cloudnum.setValidator(&aIntValidator);

	cloudnum.setText("2");
	form.addRow(QString("ƴ������Ϊ2��3��"), &cloudnum);

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
		dialog.setWindowTitle("����ƴ��");
		QFormLayout form(&dialog);

		QComboBox curComboBox;
		for (int i = 0; i < validPoints.size(); ++i) {
			curComboBox.addItem(QString::fromStdString(entityVector->GetId(validPoints[i])));//�ĳɵ��Ƶ�����
		}
		form.addRow(QString("ѡ���һ�����ƣ�"), &curComboBox);

		QComboBox curComboBox2;
		for (int i = 0; i < validPoints.size(); ++i) {
			curComboBox2.addItem(QString::fromStdString(entityVector->GetId(validPoints[i])));//�ĳɵ��Ƶ�����
		}
		form.addRow(QString("ѡ��ڶ������ƣ�"), &curComboBox2);

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

		std::string msg = "�������ƴ�����";
		PushMessage(msg);
	}

	else if (cloudn == 3)
	{
		QDialog dialog(this);
		dialog.setFixedSize(300, 100);
		dialog.setWindowTitle("����ƴ��");
		QFormLayout form(&dialog);

		QComboBox curComboBox;
		for (int i = 0; i < validPoints.size(); ++i) {
			curComboBox.addItem(QString::fromStdString(entityVector->GetId(validPoints[i])));//�ĳɵ��Ƶ�����
		}
		form.addRow(QString("ѡ���һ�����ƣ�"), &curComboBox);

		QComboBox curComboBox2;
		for (int i = 0; i < validPoints.size(); ++i) {
			curComboBox2.addItem(QString::fromStdString(entityVector->GetId(validPoints[i])));//�ĳɵ��Ƶ�����
		}
		form.addRow(QString("ѡ��ڶ������ƣ�"), &curComboBox2);

		QComboBox curComboBox3;
		for (int i = 0; i < validPoints.size(); ++i) {
			curComboBox3.addItem(QString::fromStdString(entityVector->GetId(validPoints[i])));//�ĳɵ��Ƶ�����
		}
		form.addRow(QString("ѡ����������ƣ�"), &curComboBox3);

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

		std::string msg = "�������ƴ�����";
		PushMessage(msg);
	}
	else
	{
		QString dlgTitle = "��ʾ";
		QString strInfo = "ƴ�ӵ�������ֻ��Ϊ2��3";
		QMessageBox::information(this, dlgTitle, strInfo, QMessageBox::Ok, QMessageBox::NoButton);
		return;
	}

}




 //ROI���Ƶ���
void PointCloudLab::on_copyPointAction_triggered(bool checked)
{
    cout << "���Ƶ���\n";
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
	PushMessage("���Ƶ�����ɣ��� Drag ������");
}


//ROI��ȡ����
void PointCloudLab::on_extractPointAction_triggered(bool checked)
{
    cout << "��ȡ����\n";
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

	entityTree[selectedCloudIdx]->pointsSize->setText(0, QString("����: ") + QString::number(pcv->GetPointNum()));
	PushMessage("��ȡ������ɣ��� Drag ������");
}


void PointCloudLab::OnShowAction()
{
    cout << "show points  " << curPointsId << endl;
	if (!entityVector->IsValid(curPointsId))
		return;
    if (entityVector->IsShown(curPointsId))
        return;
    //todo ��ʾ����
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
	dialog.setWindowTitle("�Ƿ�ȷ��ɾ��");
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
		PushMessage("ɾ���ɹ�");
    }
    cout << "delete points " << curPointsId << endl;

}

void PointCloudLab::OnSetColorAction()
{
    cout << "set color " << curPointsId << endl;
	if (entityTree[curPointsId]->type == "Mesh") {
		MeshVisualization *meshv = entityVector->GetMESHVofIdx(curPointsId);
		vector<int> currColor = meshv->GetColor();
		QColor  iniColor = QColor(currColor[0], currColor[1], currColor[2]); //���е�������ɫ
		QColor color = QColorDialog::getColor(iniColor, this, "ѡ����ɫ");
		if (color.isValid()) //ѡ����Ч
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
		QColor  iniColor = QColor(currColor[0], currColor[1], currColor[2]); //���е�������ɫ
		QColor color = QColorDialog::getColor(iniColor, this, "ѡ����ɫ");
		if (color.isValid()) //ѡ����Ч
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
	QString dlgTitle = "�������"; //�Ի������
	QString filter = "pcd�ļ�(*.pcd);;ply�ļ�(*.ply);;obj�ļ�(*.obj);;stl�ļ�(*.stl);;mesh�ļ�(*.mesh);;png�ļ�(*.png);;�����ļ�(*.*)"; //�ļ�������
	QString fileName = QFileDialog::getSaveFileName(this, dlgTitle, curPath, filter);
	if (fileName.isEmpty()) {
		return;
	}

	QFileInfo file(fileName);
	if (file.exists()) {
		QFile::remove(fileName);
	}

	QTextCodec *code = QTextCodec::codecForName("GB2312");//�������·������
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
		dialog.setWindowTitle("�������ͼ����");
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
		form.addRow(QString("X�ό��= "), &fx);
		form.addRow(QString("Y�ό��= "), &fy);
		form.addRow(QString("�������cx= "), &cx);
		form.addRow(QString("�������cy= "), &cy);
		form.addRow(QString("�����������s= "), &s);


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


	PushMessage("�������");

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
		QString dlgTitle = "��ʾ";
		QString strInfo = "��ǰû�д򿪵ĵ���";
		QMessageBox::information(this, dlgTitle, strInfo, QMessageBox::Ok, QMessageBox::NoButton);
		return;
	}

	QDialog dialog(this);
	dialog.setFixedSize(300, 100);
	dialog.setWindowTitle("��ѡ����ѡ��");
	QFormLayout form(&dialog);
	QComboBox curComboBox;
	for (int i = 0; i < validPoints.size(); ++i) {
		curComboBox.addItem(QString::fromStdString(entityVector->GetId(validPoints[i])));//�ĳɵ��Ƶ�����
	}
	form.addRow(QString("ѡ���ѡ���ƣ�"), &curComboBox);

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
		QString dlgTitle = "��ʾ";
		QString strInfo = "��ǰû�д򿪵ĵ���";
		QMessageBox::information(this, dlgTitle, strInfo, QMessageBox::Ok, QMessageBox::NoButton);
		return;
	}

	QDialog dialog(this);
	dialog.setFixedSize(300, 100);
	dialog.setWindowTitle("��ѡ����ѡ��");
	QFormLayout form(&dialog);
	QComboBox curComboBox;
	for (int i = 0; i < validPoints.size(); ++i) {
		curComboBox.addItem(QString::fromStdString(entityVector->GetId(validPoints[i])));//�ĳɵ��Ƶ�����
	}
	form.addRow(QString("ѡ���ѡ���ƣ�"), &curComboBox);

	QDialogButtonBox buttonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel,
		Qt::Horizontal, &dialog);
	form.addRow(&buttonBox);
	QObject::connect(&buttonBox, SIGNAL(accepted()), &dialog, SLOT(accept()));
	QObject::connect(&buttonBox, SIGNAL(rejected()), &dialog, SLOT(reject()));


	if (dialog.exec() == QDialog::Rejected) {
		return;
	}

	PushMessage("����X���� ��ʼ/���� ��ѡ����");
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
	dialog.setWindowTitle("��ʾ����");
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
	form.addRow(QString("������ɫ: "), &tempLayout1);

	QSpinBox pointSizeBox;
	pointSizeBox.setValue(pointViewerSize);
	pointSizeBox.setRange(1, 10);

	form.addRow(QString("���С: "), &pointSizeBox);

	QCheckBox checkBox1("��ʾ������");
	checkBox1.setChecked(isCoordinateSystemShown);
	form.addRow(&checkBox1);

	QDoubleSpinBox coordSizeBox;
	coordSizeBox.setValue(coordinateSystemSize);
	coordSizeBox.setRange(0.0001, 100000);

	form.addRow(QString("�������С: "), &coordSizeBox);

	QCheckBox checkBox2("��ʾ�ߴ�(mm)");
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