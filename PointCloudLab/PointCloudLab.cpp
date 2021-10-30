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

#include <iostream>
using namespace std;

#pragma execution_character_set("utf-8")


PointCloudLab::PointCloudLab(QWidget *parent)
    : QMainWindow(parent)
{
    ui.setupUi(this);
    InitVtk();
    InitPointTree();
    InitMenuAction();

	// Init PointCloudVector
	pointCloudVector = new PointCloudVector(viewer);

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

    for (int i = 0; i < PointCloudTree.size(); ++i)
    {
        if (PointCloudTree[i] != nullptr) {
            delete PointCloudTree[i];
        }
    }
}

//functions:

void PointCloudLab::contextMenuEvent(QContextMenuEvent *event)
{
    QTreeWidgetItem *item = ui.treeWidget->currentItem();
    if (item != nullptr)
    {
        for (int i = 0; i < PointCloudTree.size(); ++i)
        {
            if (PointCloudTree[i] != nullptr&&PointCloudTree[i]->cloudName == item)
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
	showAction = new QAction("Show", ui.treeWidget);
	hideAction = new QAction("Hide", ui.treeWidget);
	deleteAction = new QAction("Delete", ui.treeWidget);
	setColorAction = new QAction("Set color", ui.treeWidget);
	saveCurPointAction = new QAction("Save", ui.treeWidget);

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
	
	viewer->registerAreaPickingCallback(area_callback, this);

}

void PointCloudLab::InitPointTree()
{
    ui.treeWidget->setColumnCount(2); //设置列数
    ui.treeWidget->setHeaderHidden(false);
	QStringList header;
	header.push_back(QString("Name"));
	header.push_back(QString("Property"));
	ui.treeWidget->setHeaderLabels(header);
	ui.treeWidget->header()->setSectionResizeMode(QHeaderView::Interactive);
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

int PointCloudLab::OpenFile(string filePath)
{
    vector<string> temp = split(filePath, ".");
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
	else if(temp.back() == "txt") {
		return OpenTxtFile(filePath);
	}
}

int PointCloudLab::OpenPcdFile(string path)
{
	PointCloudT::Ptr cloud(new PointCloudT());

    if (pcl::io::loadPCDFile(path, *cloud))
    {
        cerr << "ERROR: Cannot open file " << path << "! Aborting..." << endl;
        return FAILED;
    }
    vector<string> tempId = PointCloudLab::split(path, "/");
    string id = tempId.back();
	int idx = pointCloudVector->AddPointCloud(cloud, id);
	PointCloudVisualization *pcv = pointCloudVector->GetPCVofIdx(idx);
    pcv->Show();
    viewer->resetCamera();
    ui.qvtkWidget->update();

	completeCloud->AddCloud(cloud);

	// Todo: edit type, point num, face num
	PointTree * tempTree = new PointTree(&ui, id, "PointXYZ", pcv->GetCloudSize(), 0);
	PointCloudTree.push_back(tempTree);
    //isDeleted.push_back(false);
    //isShown.push_back(true);
    return SUCCESS;
}
int PointCloudLab::OpenPlyFile(string path)
{
	return CANCEL;
}
int PointCloudLab::OpenObjFile(string path)
{
	return CANCEL;
}
int PointCloudLab::OpenStlFile(string path)
{
	return CANCEL;
}
int PointCloudLab::OpenMeshFile(string path)
{
	return CANCEL;
}

int PointCloudLab::OpenTxtFile(string path) {
	ifstream fin(path);
	//fin.open(path);
	string line;
	PointCloudT::Ptr cloud(new PointCloudT());
	while (getline(fin, line)) {
		stringstream ss; //输入流
		ss << line; //向流中传值
		if (!ss.eof()) {
			double temp;
			vector<double> res;
			
			while (ss >> temp) {
				res.push_back(temp); //保存到vector
			}
			PointT point;
			point.x = res[0];
			point.y = res[1];
			point.z = res[2];
			cloud->push_back(point);
			//cout << res.size() << endl;
		}
	}
	fin.close();

	string id = "pd";
	int idx = pointCloudVector->AddPointCloud(cloud, id);
	PointCloudVisualization *pcv = pointCloudVector->GetPCVofIdx(idx);
	pcv->Show();
	viewer->resetCamera();
	ui.qvtkWidget->update();

	PointTree * tempTree = new PointTree(&ui, id, "PointXYZ", pcv->GetCloudSize(), 0);
	PointCloudTree.push_back(tempTree);

	return SUCCESS;
}

int PointCloudLab::OpenPngFile(string path)
{
	QDialog dialog(this);
	dialog.setFixedSize(200, 150);
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
		return CANCEL;
	}


	return FAILED;

}



//slot:
void PointCloudLab::on_openFileAction_triggered(bool checked)
{
    QString curPath = QDir::currentPath();
    QString dlgTitle = "Open File"; //对话框标题
    QString filter = "pcd file(*.pcd);;ply file(*.ply);;obj file(*.obj);;stl file(*.stl);;txt file(*.txt);;mesh file(*.mesh);;png file(*.png);;all file(*.*)"; //文件过滤器
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
		PushMessage("point cloud opened");
	}
	else if (ret == FAILED)
	{
		PushMessage("fail to open point cloud");
	}
}
void PointCloudLab::on_saveFileAction_triggered(bool checked)
{
	// Todo: check valid point
	vector<int> validPoints = GetValidPointsId();
	if (validPoints.size() == 0) {
		QString dlgTitle = "Warning";
		QString strInfo = "No opened point cloud";
		QMessageBox::information(this, dlgTitle, strInfo, QMessageBox::Ok, QMessageBox::NoButton);
		return;
	}
	//保存所有点云
	QString curPath = QCoreApplication::applicationDirPath();
	QString dlgTitle = "Save point cloud"; //对话框标题
	QString filter = "pcd file(*.pcd);;ply file(*.ply);;obj file(*.obj);;stl file(*.stl);;mesh file(*.mesh);;png file(*.png);;all file(*.*)"; //文件过滤器
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
	vector<string> temp = split(filePath, ".");
	string fileType = temp.back();

	if (fileType == "pcd" || fileType == "ply" || fileType == "obj" || fileType == "stl" || fileType == "mesh")
	{
		//To do

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
	cout << "save file\n";
}

void PointCloudLab::on_filterAction1_triggered(bool checked)
{
	vector<int> validPoints = GetValidPointsId();
	if (validPoints.size() == 0) {
		QString dlgTitle = "Warning";
		QString strInfo = "No opened point cloud";
		QMessageBox::information(this, dlgTitle, strInfo, QMessageBox::Ok, QMessageBox::NoButton);
		return;
	}

	QDialog dialog(this);
	dialog.setFixedSize(350, 200);
	dialog.setWindowTitle("Pass through setting");
	QFormLayout form(&dialog);
	QComboBox curComboBox;
	for (int i = 0; i < validPoints.size(); ++i) {
		curComboBox.addItem(QString::fromStdString(pointCloudVector->GetId(validPoints[i])));//改成点云的名字
	}
	form.addRow(QString("Select point cloud："), &curComboBox);

	QHBoxLayout  tempLayout1;
	QCheckBox checkBoxX("X");
	QCheckBox checkBoxY("Y");
	QCheckBox checkBoxZ("Z");
	tempLayout1.addWidget(&checkBoxX);
	tempLayout1.addWidget(&checkBoxY);
	tempLayout1.addWidget(&checkBoxZ);
	form.addRow(QString("Filter area: "), &tempLayout1);
	QLabel tempLabel("Filter range:");
	form.addRow(&tempLabel);
	QLineEdit textMin;
	QLineEdit textMax;
	QDoubleValidator aDoubleValidator;
	textMin.setValidator(&aDoubleValidator);
	textMax.setValidator(&aDoubleValidator);
	textMin.setText("0");
	textMax.setText("0");
	form.addRow(QString("    min="), &textMin);
	form.addRow(QString("    max= "), &textMax);

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

	selectedCloudIdx = validPoints[curComboBox.currentIndex()];
	PointCloudT::Ptr selectedCloud = pointCloudVector->GetCloudPtrOfIdx(selectedCloudIdx);

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
	// Filter Here

	PointCloudVisualization *pcv = pointCloudVector->GetPCVofIdx(selectedCloudIdx);
	pcv->Show();
	ui.qvtkWidget->update();
	
	PushMessage("Pass through filter");
}//直通滤波

void PointCloudLab::on_filterAction2_triggered(bool checked)
{
	vector<int> validPoints = GetValidPointsId();
	if (validPoints.size() == 0) {
		QString dlgTitle = "Warning";
		QString strInfo = "No opened point cloud";
		QMessageBox::information(this, dlgTitle, strInfo, QMessageBox::Ok, QMessageBox::NoButton);
		return;
	}

	QDialog dialog(this);
	dialog.setFixedSize(350, 200);
	dialog.setWindowTitle("Voxel grid setting");
	QFormLayout form(&dialog);
	QComboBox curComboBox;
	for (int i = 0; i < validPoints.size(); ++i) {
		curComboBox.addItem(QString::fromStdString(pointCloudVector->GetId(validPoints[i])));//改成点云的名字
	}
	form.addRow(QString("Select point cloud："), &curComboBox);
	QLabel tempLabel("Voxel size:");
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

	selectedCloudIdx = validPoints[curComboBox.currentIndex()];
	PointCloudT::Ptr selectedCloud = pointCloudVector->GetCloudPtrOfIdx(selectedCloudIdx);
	pcl::VoxelGrid<PointT> sor;
	sor.setInputCloud(selectedCloud);
	sor.setLeafSize(x, y, z); 
	sor.filter(*selectedCloud);

	PointCloudVisualization *pcv = pointCloudVector->GetPCVofIdx(selectedCloudIdx);
	pcv->Show();
	ui.qvtkWidget->update();

	PushMessage("Voxel grid filter");
}//体素滤波

void PointCloudLab::on_filterAction3_triggered(bool checked)
{
	vector<int> validPoints = GetValidPointsId();
	if (validPoints.size() == 0) {
		QString dlgTitle = "Warning";
		QString strInfo = "No opened point cloud";
		QMessageBox::information(this, dlgTitle, strInfo, QMessageBox::Ok, QMessageBox::NoButton);
		return;
	}

	QDialog dialog(this);
	dialog.setFixedSize(350, 200);
	dialog.setWindowTitle("Statistical filter setting");
	QFormLayout form(&dialog);
	QComboBox curComboBox;
	for (int i = 0; i < validPoints.size(); ++i) {
		curComboBox.addItem(QString::fromStdString(pointCloudVector->GetId(validPoints[i])));//改成点云的名字
	}
	form.addRow(QString("Select point cloud："), &curComboBox);
	QLineEdit neighbourNum, standerDif;

	QDoubleValidator aDoubleValidator;
	QIntValidator aIntValidator;
	neighbourNum.setValidator(&aIntValidator);
	standerDif.setValidator(&aDoubleValidator);

	neighbourNum.setText("0");
	standerDif.setText("1");

	form.addRow(QString("meank ="), &neighbourNum);
	form.addRow(QString("std ="), &standerDif);

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
	double std = stdStr.toDouble();

	selectedCloudIdx = validPoints[curComboBox.currentIndex()];
	PointCloudT::Ptr selectedCloud = pointCloudVector->GetCloudPtrOfIdx(selectedCloudIdx);
	pcl::StatisticalOutlierRemoval<PointT> Static;   
	Static.setInputCloud(selectedCloud);   
	Static.setMeanK(150);                               
	Static.setStddevMulThresh(0.5);                      
	Static.filter(*selectedCloud);                    

	PointCloudVisualization *pcv = pointCloudVector->GetPCVofIdx(selectedCloudIdx);
	pcv->Show();
	ui.qvtkWidget->update();

	PushMessage("Statistical filter");

}//统计滤波


void PointCloudLab::on_copyPointAction_triggered(bool checked)
{
	if (setSelected.empty() || selectedCloudIdx == -1)
		return;

	string id = "copy_" + pointCloudVector->GetId(selectedCloudIdx);
	PointCloudT::Ptr tempPtr(new PointCloudT);
	*tempPtr = *clicked_points_3d;
	int idx = pointCloudVector->AddPointCloud(tempPtr, id);

	PointTree * tempTree = new PointTree(&ui, id, "PointXYZ", tempPtr->size(), 0);
	PointCloudTree.push_back(tempTree);

	PointCloudTree[idx]->cloudName->setTextColor(0, QColor(150, 150, 150));
	PointCloudTree[idx]->pointsSize->setTextColor(0, QColor(150, 150, 150));
	PointCloudTree[idx]->faceSize->setTextColor(0, QColor(150, 150, 150));
	PointCloudTree[idx]->cloudName->setTextColor(1, QColor(150, 150, 150));
	PointCloudTree[idx]->pointsSize->setTextColor(1, QColor(150, 150, 150));
	PointCloudTree[idx]->faceSize->setTextColor(1, QColor(150, 150, 150));
	
	motionState = DRAG;
	ui.qvtkWidget->setFocus();
	ui.qvtkWidget->show();
	QKeyEvent keyPress(QEvent::KeyPress, Qt::Key_X, Qt::NoModifier, "x");
	QKeyEvent keyRelease(QEvent::KeyRelease, Qt::Key_X, Qt::NoModifier);
	QCoreApplication::sendEvent(ui.qvtkWidget, &keyPress);
	QCoreApplication::sendEvent(ui.qvtkWidget, &keyRelease);
	PushMessage("selected point cloud copied");
}//复制点云

void PointCloudLab::on_extractPointAction_triggered(bool checked)
{
	if (setSelected.empty() || selectedCloudIdx == -1)
		return;

	string id = "copy_" + pointCloudVector->GetId(selectedCloudIdx);
	PointCloudT::Ptr tempPtr(new PointCloudT);
	*tempPtr = *clicked_points_3d;
	int idx = pointCloudVector->AddPointCloud(tempPtr, id);

	PointTree * tempTree = new PointTree(&ui, id, "PointXYZ", tempPtr->size(), 0);
	PointCloudTree.push_back(tempTree);

	PointCloudTree[idx]->cloudName->setTextColor(0, QColor(150, 150, 150));
	PointCloudTree[idx]->pointsSize->setTextColor(0, QColor(150, 150, 150));
	PointCloudTree[idx]->faceSize->setTextColor(0, QColor(150, 150, 150));
	PointCloudTree[idx]->cloudName->setTextColor(1, QColor(150, 150, 150));
	PointCloudTree[idx]->pointsSize->setTextColor(1, QColor(150, 150, 150));
	PointCloudTree[idx]->faceSize->setTextColor(1, QColor(150, 150, 150));

	motionState = DRAG;
	ui.qvtkWidget->setFocus();
	ui.qvtkWidget->show();
	QKeyEvent keyPress(QEvent::KeyPress, Qt::Key_X, Qt::NoModifier, "x");
	QKeyEvent keyRelease(QEvent::KeyRelease, Qt::Key_X, Qt::NoModifier);
	QCoreApplication::sendEvent(ui.qvtkWidget, &keyPress);
	QCoreApplication::sendEvent(ui.qvtkWidget, &keyRelease);

	//Point oriCloudPtr = pointCloudVector->GetCloudPtrOfIdx(selectedCloudIdx);
	PointCloudVisualization *pcv = pointCloudVector->GetPCVofIdx(selectedCloudIdx);
	pcv->DeletePointFromVector(setSelected);
	ui.qvtkWidget->update();

	PointCloudTree[selectedCloudIdx]->pointsSize->setText(0, QString("point num: ") + QString::number(pcv->GetCloudSize()));
	PushMessage("selected point cloud extracted");
}//提取点云


void PointCloudLab::OnShowAction()
{
    cout << "show points  " << curPointsId << endl;
	if (!pointCloudVector->IsValid(curPointsId))
		return;
    if (pointCloudVector->IsShown(curPointsId))
        return;
    //todo Show点云
	PointCloudVisualization *pcv = pointCloudVector->GetPCVofIdx(curPointsId);
	assert(pcv != nullptr);
	pcv->Show();
	ui.qvtkWidget->update();


    PointCloudTree[curPointsId]->cloudName->setTextColor(0, QColor(0, 0, 0));
	PointCloudTree[curPointsId]->pointsSize->setTextColor(0, QColor(0, 0, 0));
	PointCloudTree[curPointsId]->faceSize->setTextColor(0, QColor(0, 0, 0));
	PointCloudTree[curPointsId]->cloudName->setTextColor(1, QColor(0, 0, 0));
	PointCloudTree[curPointsId]->pointsSize->setTextColor(1, QColor(0, 0, 0));
    PointCloudTree[curPointsId]->faceSize->setTextColor(1, QColor(0, 0, 0));
}
void PointCloudLab::OnHideAction()
{
    cout << "hide points " << curPointsId << endl;
	if (!pointCloudVector->IsValid(curPointsId))
		return;
	if (!pointCloudVector->IsShown(curPointsId))
		return;

	PointCloudVisualization *pcv = pointCloudVector->GetPCVofIdx(curPointsId);
	assert(pcv != nullptr);
	pcv->Hide();
	ui.qvtkWidget->update();
   
    PointCloudTree[curPointsId]->cloudName->setTextColor(0, QColor(150, 150, 150));
	PointCloudTree[curPointsId]->pointsSize->setTextColor(0, QColor(150, 150, 150));
	PointCloudTree[curPointsId]->faceSize->setTextColor(0, QColor(150, 150, 150));
	PointCloudTree[curPointsId]->cloudName->setTextColor(1, QColor(150, 150, 150));
	PointCloudTree[curPointsId]->pointsSize->setTextColor(1, QColor(150, 150, 150));
    PointCloudTree[curPointsId]->faceSize->setTextColor(1, QColor(150, 150, 150));
}
void PointCloudLab::OnDeleteAction()
{
	// Todo: Release memory in point cloud vector
    if (PointCloudTree[curPointsId] != nullptr) {
        delete PointCloudTree[curPointsId];
        PointCloudTree[curPointsId] = nullptr;
		pointCloudVector->DeletePointCloud(curPointsId);
		ui.qvtkWidget->update();
		PushMessage("delete point cloud");
    }
    cout << "delete points " << curPointsId << endl;

}

void PointCloudLab::OnSetColorAction()
{
    cout << "set color " << curPointsId << endl;
	PointCloudVisualization *pcv = pointCloudVector->GetPCVofIdx(curPointsId);
	vector<int> currColor = pcv->GetColor();
	QColor  iniColor = QColor(currColor[0], currColor[1], currColor[2]); //现有的文字颜色
	QColor color = QColorDialog::getColor(iniColor, this, "Select color");
	if (color.isValid()) //选择有效
	{
		cout << "set color =R:" << color.red() << endl;
		cout << "set color =G:" << color.green() << endl;
		cout << "set color =B:" << color.blue() << endl;

		pcv->SetColor(color.red(), color.green(), color.blue());
	}

}

void PointCloudLab::OnSaveCurPointAction()
{
	cout << "save points " << curPointsId << endl;
	QString curPath = QCoreApplication::applicationDirPath();
	QString dlgTitle = "Save"; //对话框标题
	QString filter = "pcd file(*.pcd);;ply file(*.ply);;obj file(*.obj);;stl file(*.stl);;mesh file(*.mesh);;png file(*.png);;all file(*.*)"; // 文件过滤器
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
	vector<string> temp = split(filePath, ".");
	string fileType = temp.back();

	if (fileType == "pcd" || fileType == "ply" || fileType == "obj" || fileType == "stl" || fileType == "mesh")
	{
		//To do
		pointCloudVector->SavePointCloudOfIdx(filePath, curPointsId);

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


	PushMessage("Save");

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

	//viewer->registerAreaPickingCallback(area_callback, this);
	cout << "press X to strat or ending picking, then press 'Q'..." << endl;
	
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
		PointCloudT::Ptr currCloud = pointCloudVector->GetCloudPtrOfIdx(selectedCloudIdx);
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
	PointCloudT::Ptr currCloud = pointCloudVector->GetCloudPtrOfIdx(selectedCloudIdx);
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
	PointCloudT::Ptr currCloud = p->pointCloudVector->GetCloudPtrOfIdx(p->selectedCloudIdx);
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
	if (!p->pointCloudVector->IsValid(p->selectedCloudIdx))
		return;
	
	PointCloudT::Ptr currCloud = p->pointCloudVector->GetCloudPtrOfIdx(p->selectedCloudIdx);

	
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
	vector<int> validPoints = GetValidPointsId();
	if (validPoints.size() == 0) {
		QString dlgTitle = "Warning";
		QString strInfo = "No opened point cloud";
		QMessageBox::information(this, dlgTitle, strInfo, QMessageBox::Ok, QMessageBox::NoButton);
		return;
	}

	QDialog dialog(this);
	dialog.setFixedSize(300, 100);
	dialog.setWindowTitle("Selection");
	QFormLayout form(&dialog);
	QComboBox curComboBox;
	for (int i = 0; i < validPoints.size(); ++i) {
		curComboBox.addItem(QString::fromStdString(pointCloudVector->GetId(validPoints[i])));//改成点云的名字
	}
	form.addRow(QString("Select point cloud："), &curComboBox);

	QDialogButtonBox buttonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel,
		Qt::Horizontal, &dialog);
	form.addRow(&buttonBox);
	QObject::connect(&buttonBox, SIGNAL(accepted()), &dialog, SLOT(accept()));
	QObject::connect(&buttonBox, SIGNAL(rejected()), &dialog, SLOT(reject()));


	if (dialog.exec() == QDialog::Rejected) {
		return;
	}

	selectedCloudIdx = validPoints[curComboBox.currentIndex()];
	int n = pointCloudVector->GetSize();
	for (int i = 0; i < n; ++i) {
		if (!pointCloudVector->IsValid(i)) {
			continue;
		}
		if (i == selectedCloudIdx) {
			PointCloudVisualization* pcv = pointCloudVector->GetPCVofIdx(i);
			assert(pcv != nullptr);
			pcv->Show();
			ui.qvtkWidget->update();

			PointCloudTree[i]->cloudName->setTextColor(0, QColor(0, 0, 0));
			PointCloudTree[i]->pointsSize->setTextColor(0, QColor(0, 0, 0));
			PointCloudTree[i]->faceSize->setTextColor(0, QColor(0, 0, 0));
			PointCloudTree[i]->cloudName->setTextColor(1, QColor(0, 0, 0));
			PointCloudTree[i]->pointsSize->setTextColor(1, QColor(0, 0, 0));
			PointCloudTree[i]->faceSize->setTextColor(1, QColor(0, 0, 0));
			continue;
		}
		if (pointCloudVector->IsShown(i)) {
			PointCloudVisualization* pcv = pointCloudVector->GetPCVofIdx(i);
			assert(pcv != nullptr);
			pcv->Hide();
			ui.qvtkWidget->update();

			PointCloudTree[i]->cloudName->setTextColor(0, QColor(150, 150, 150));
			PointCloudTree[i]->pointsSize->setTextColor(0, QColor(150, 150, 150));
			PointCloudTree[i]->faceSize->setTextColor(0, QColor(150, 150, 150));
			PointCloudTree[i]->cloudName->setTextColor(1, QColor(150, 150, 150));
			PointCloudTree[i]->pointsSize->setTextColor(1, QColor(150, 150, 150));
			PointCloudTree[i]->faceSize->setTextColor(1, QColor(150, 150, 150));
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
	vector<int> validPoints = GetValidPointsId();
	if (validPoints.size() == 0) {
		QString dlgTitle = "Warning";
		QString strInfo = "No opened point cloud";
		QMessageBox::information(this, dlgTitle, strInfo, QMessageBox::Ok, QMessageBox::NoButton);
		return;
	}

	QDialog dialog(this);
	dialog.setFixedSize(300, 100);
	dialog.setWindowTitle("Selection");
	QFormLayout form(&dialog);
	QComboBox curComboBox;
	for (int i = 0; i < validPoints.size(); ++i) {
		curComboBox.addItem(QString::fromStdString(pointCloudVector->GetId(validPoints[i])));//改成点云的名字
	}
	form.addRow(QString("Select point cloud："), &curComboBox);

	QDialogButtonBox buttonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel,
		Qt::Horizontal, &dialog);
	form.addRow(&buttonBox);
	QObject::connect(&buttonBox, SIGNAL(accepted()), &dialog, SLOT(accept()));
	QObject::connect(&buttonBox, SIGNAL(rejected()), &dialog, SLOT(reject()));


	if (dialog.exec() == QDialog::Rejected) {
		return;
	}

	if (validPoints[curComboBox.currentIndex()] != selectedCloudIdx) {
		clicked_points_3d.reset(new PointCloudT);
		vector<int>().swap(selectedPointIdxs);
		unordered_set<int>().swap(setSelected);
		viewer->removePointCloud("clicked_points");
		ui.qvtkWidget->update();
	}
	selectedCloudIdx = validPoints[curComboBox.currentIndex()];
	int n = pointCloudVector->GetSize();
	for (int i = 0; i < n; ++i) {
		if (!pointCloudVector->IsValid(i)) {
			continue;
		}
		if (i == selectedCloudIdx) {
			PointCloudVisualization* pcv = pointCloudVector->GetPCVofIdx(i);
			assert(pcv != nullptr);
			pcv->Show();
			ui.qvtkWidget->update();

			PointCloudTree[i]->cloudName->setTextColor(0, QColor(0, 0, 0));
			PointCloudTree[i]->pointsSize->setTextColor(0, QColor(0, 0, 0));
			PointCloudTree[i]->faceSize->setTextColor(0, QColor(0, 0, 0));
			PointCloudTree[i]->cloudName->setTextColor(1, QColor(0, 0, 0));
			PointCloudTree[i]->pointsSize->setTextColor(1, QColor(0, 0, 0));
			PointCloudTree[i]->faceSize->setTextColor(1, QColor(0, 0, 0));
			continue;
		}
		if (pointCloudVector->IsShown(i)) {
			PointCloudVisualization* pcv = pointCloudVector->GetPCVofIdx(i);
			assert(pcv != nullptr);
			pcv->Hide();
			ui.qvtkWidget->update();

			PointCloudTree[i]->cloudName->setTextColor(0, QColor(150, 150, 150));
			PointCloudTree[i]->pointsSize->setTextColor(0, QColor(150, 150, 150));
			PointCloudTree[i]->faceSize->setTextColor(0, QColor(150, 150, 150));
			PointCloudTree[i]->cloudName->setTextColor(1, QColor(150, 150, 150));
			PointCloudTree[i]->pointsSize->setTextColor(1, QColor(150, 150, 150));
			PointCloudTree[i]->faceSize->setTextColor(1, QColor(150, 150, 150));
		}
	}

	motionState = AREA_PICK;
	AreaPicking();
	//ui.qvtkWidget->setFocus();
	//QKeyEvent keyPress(QEvent::KeyPress, Qt::Key_X, Qt::NoModifier);
	//QKeyEvent keyRelease(QEvent::KeyRelease, Qt::Key_X, Qt::NoModifier);

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
	ui.qvtkWidget->update();

}

void PointCloudLab::on_pushButton_setting_clicked() {
	QDialog dialog(this);
	dialog.setFixedSize(320, 240);
	dialog.setWindowTitle("Settings");
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
	form.addRow(QString("Background color: "), &tempLayout1);

	QSpinBox pointSizeBox;
	pointSizeBox.setValue(pointViewerSize);
	pointSizeBox.setRange(1, 10);

	form.addRow(QString("Point size: "), &pointSizeBox);

	QCheckBox checkBox1("Show axis");
	checkBox1.setChecked(isCoordinateSystemShown);
	form.addRow(&checkBox1);

	QDoubleSpinBox coordSizeBox;
	coordSizeBox.setValue(coordinateSystemSize);
	coordSizeBox.setRange(0.0001, 100000);

	form.addRow(QString("Axis thickness: "), &coordSizeBox);

	QCheckBox checkBox2("Show grid(mm)");
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
	int cloudSize = pointCloudVector->GetSize();
	for (int i = 0; i < cloudSize; ++i) {
		PointCloudVisualization* pcv = pointCloudVector->GetPCVofIdx(i);
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
	cout << "complete cloud size: " << completeCloud->GetCloudSize() << endl;
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
	for (int i = 0; i < pointCloudVector->GetSize(); ++i) {
		if (pointCloudVector->IsValid(i)) {
			validPoints.push_back(i);
		}
	}

	return validPoints;
}