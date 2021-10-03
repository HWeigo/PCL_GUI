#include "PointCloudLab.h"
#include <QStandardItem>
#include <QDateTime>
#include <QFileDialog>
#include <QTextCodec>
#include <QFormLayout>
#include <QSpinBox>
#include <QDialogButtonBox>
#include <QLabel>

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
}
PointCloudLab::~PointCloudLab()
{
    delete treeMenu;
    delete showAction;
    delete hideAction;
    delete deleteAction;
    delete setColorAction;
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

    connect(showAction, SIGNAL(triggered(bool)), this, SLOT(OnShowAction()));
    connect(hideAction, SIGNAL(triggered(bool)), this, SLOT(OnHideAction()));
    connect(deleteAction, SIGNAL(triggered(bool)), this, SLOT(OnDeleteAction()));
    connect(setColorAction, SIGNAL(triggered(bool)), this, SLOT(OnSetColorAction()));
}

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

void PointCloudLab::InitPointTree()
{
    ui.treeWidget->setColumnCount(1); //设置列数
    ui.treeWidget->setHeaderHidden(true);
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
}

int PointCloudLab::OpenPcdFile(string path)
{
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>());

    if (pcl::io::loadPCDFile(path, *cloud))
    {
        cerr << "ERROR: Cannot open file " << path << "! Aborting..." << endl;
        return -1;
    }
    vector<string> tempId = PointCloudLab::split(path, "/");
    string id = tempId.back();
	int idx = pointCloudVector->AddPointCloud(cloud, id);
	PointCloudVisualization *pcv = pointCloudVector->GetPCVofIdx(idx);
    //PointCloudVisualization *pcv = new PointCloudVisualization(viewer, cloud, id);
    //cloudVisualVector.push_back(pcv);
    pcv->Show();
    viewer->resetCamera();
    ui.qvtkWidget->update();

    PointTree * tempTree = new PointTree(&ui, id, 100, 100);
    PointCloudTree.push_back(tempTree);
    isDeleted.push_back(false);
    isShown.push_back(true);
    return 1;
}
int PointCloudLab::OpenPlyFile(string path)
{
    return 0;
}
int PointCloudLab::OpenObjFile(string path)
{
    return 0;
}
int PointCloudLab::OpenStlFile(string path)
{
    return 0;
}
int PointCloudLab::OpenMeshFile(string path)
{
    return 0;
}
int PointCloudLab::OpenPngFile(string path)
{
    QDialog dialog(this);
    QFormLayout form(&dialog);
    form.addRow(new QLabel("User input:"));
    // Value1
    QString value1 = QString("Value1: ");
    QSpinBox *spinbox1 = new QSpinBox(&dialog);
    form.addRow(value1, spinbox1);
    // Value2
    QString value2 = QString("Value2: ");
    QSpinBox *spinbox2 = new QSpinBox(&dialog);
    form.addRow(value2, spinbox2);
    // Add Cancel and OK button
    QDialogButtonBox buttonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel,
        Qt::Horizontal, &dialog);
    form.addRow(&buttonBox);
    QObject::connect(&buttonBox, SIGNAL(accepted()), &dialog, SLOT(accept()));
    QObject::connect(&buttonBox, SIGNAL(rejected()), &dialog, SLOT(reject()));

    // Process when OK button is clicked
    if (dialog.exec() == QDialog::Accepted) {
        // Do something here
    }
    return 0;
}



//slot:
void PointCloudLab::on_openFileAction_triggered(bool checked)
{
    QString curPath = QDir::currentPath();
    QString dlgTitle = "打开文件"; //对话框标题
    QString filter = "pcd文件(*.pcd);;ply文件(*.ply);;obj文件(*.obj);;stl文件(*.stl);;mesh文件(*.mesh);;png文件(*.png);;所有文件(*.*)"; //文件过滤器
    QString fileName = QFileDialog::getOpenFileName(this, dlgTitle, curPath, filter);

    if (fileName.isEmpty())
    {
        return;
    }

    QTextCodec *code = QTextCodec::codecForName("GB2312");//解决中文路径问题
    std::string filePath = code->fromUnicode(fileName).data();

    if (OpenFile(filePath) == 1)
    {
        PushMessage("打开成功");
    }
    else if(OpenFile(filePath) == -1)
    {
        PushMessage("打开失败");
    }
}
void PointCloudLab::on_saveFileAction_triggered(bool checked)
{
    cout << "save file\n";
}

void PointCloudLab::on_filterAction1_triggered(bool checked)
{
    cout << "直通滤波 \n";
}//直通滤波
void PointCloudLab::on_filterAction2_triggered(bool checked)
{
    cout << "体素滤波\n";
}
//体素滤波
void PointCloudLab::on_filterAction3_triggered(bool checked) 
{
    cout << "统计滤波\n";
}//统计滤波


void PointCloudLab::on_copyPointAction_triggered(bool checked)
{
    cout << "复制点云\n";
}//复制点云
void PointCloudLab::on_extractPointAction_triggered(bool checked)
{
    cout << "提取点云\n";
}//提取点云


void PointCloudLab::OnShowAction()
{
    cout << "show points  " << curPointsId << endl;
    if (isDeleted[curPointsId])
        return;
    if (isShown[curPointsId])
        return;
    //todo 显示点云
	//PointCloudVisualization *pcv = cloudVisualVector[curPointsId];
	PointCloudVisualization *pcv = pointCloudVector->GetPCVofIdx(curPointsId);
	assert(pcv != nullptr);
	pcv->Show();
	ui.qvtkWidget->update();


    PointCloudTree[curPointsId]->cloudName->setTextColor(0, QColor(0, 0, 0));
    PointCloudTree[curPointsId]->pointsSize->setTextColor(0, QColor(0, 0, 0));
    PointCloudTree[curPointsId]->faceSize->setTextColor(0, QColor(0, 0, 0));
    isShown[curPointsId] = true;
}
void PointCloudLab::OnHideAction()
{
    cout << "hide points " << curPointsId << endl;
    if (isDeleted[curPointsId])
        return;
    if (!isShown[curPointsId])
        return;

	//PointCloudVisualization *pcv = cloudVisualVector[curPointsId];
	PointCloudVisualization *pcv = pointCloudVector->GetPCVofIdx(curPointsId);
	assert(pcv != nullptr);
	pcv->Hide();
	ui.qvtkWidget->update();
   
    PointCloudTree[curPointsId]->cloudName->setTextColor(0, QColor(150, 150, 150));
    PointCloudTree[curPointsId]->pointsSize->setTextColor(0, QColor(150, 150, 150));
    PointCloudTree[curPointsId]->faceSize->setTextColor(0, QColor(150, 150, 150));
    isShown[curPointsId] = false;
}
void PointCloudLab::OnDeleteAction()
{
	// Todo: Release memory in point cloud vector
    if (PointCloudTree[curPointsId] != nullptr) {
        delete PointCloudTree[curPointsId];
        PointCloudTree[curPointsId] = nullptr;
        isDeleted[curPointsId] = true;
    }
    cout << "delete points " << curPointsId << endl;
}

void PointCloudLab::OnSetColorAction()
{
    cout << "set color " << curPointsId << endl;
	QDialog dialog(this);
	dialog.setFixedSize(300, 200);
	dialog.setWindowTitle("设置颜色");
	QFormLayout form(&dialog);


	QSpinBox rBox, gBox, bBox;
	rBox.setRange(0, 255);
	gBox.setRange(0, 255);
	bBox.setRange(0, 255);

	form.addRow(QString("R=: "), &rBox);
	form.addRow(QString("G=: "), &gBox);
	form.addRow(QString("B=: "), &bBox);

	QDialogButtonBox buttonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel,
		Qt::Horizontal, &dialog);
	form.addRow(&buttonBox);
	QObject::connect(&buttonBox, SIGNAL(accepted()), &dialog, SLOT(accept()));
	QObject::connect(&buttonBox, SIGNAL(rejected()), &dialog, SLOT(reject()));

	if (dialog.exec() == QDialog::Accepted) {
		cout << "set color =R:" << rBox.value() << endl;
		cout << "set color =G:" << gBox.value() << endl;
		cout << "set color =B:" << bBox.value() << endl;

		PointCloudVisualization *pcv = pointCloudVector->GetPCVofIdx(curPointsId);
		pcv->SetColor(rBox.value(), gBox.value(), bBox.value());
	}

}



