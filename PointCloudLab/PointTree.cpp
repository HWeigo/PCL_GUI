#include "PointTree.h"

#pragma execution_character_set("utf-8")

PointTree::PointTree(Ui::PointCloudLabClass *input_ui, string filePath, std::string pointType, int pointNum, int faceNum)
{
	ui = input_ui;

	QStringList tempList;
	tempList.push_back(QString::fromStdString(filePath));
	tempList.push_back(QString::fromStdString(pointType));

	cloudName = new QTreeWidgetItem(ui->treeWidget, tempList);
	cloudName->setFlags(cloudName->flags() &(~Qt::ItemIsEditable));
	
	QStringList tempList_sub1;
	tempList_sub1.push_back(QString("point num: "));
	tempList_sub1.push_back(QString::number(pointNum));
	pointsSize = new QTreeWidgetItem(cloudName, tempList_sub1);

	QStringList tempList_sub2;
	tempList_sub2.push_back(QString("face num: "));
	tempList_sub2.push_back(QString::number(faceNum));
	faceSize = new QTreeWidgetItem(cloudName, tempList_sub2);
}
PointTree::~PointTree()
{
	delete pointsSize;
	delete faceSize;
	delete cloudName;
	pointsSize = nullptr;
	faceSize = nullptr;
	cloudName = nullptr;
}




