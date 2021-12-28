#include "EntityTree.h"

#pragma execution_character_set("utf-8")

EntityTree::EntityTree(Ui::PointCloudLabClass *input_ui, string filePath, std::string pointType, int pointNum, int faceNum)
{
	ui = input_ui;

	QStringList tempList;
	tempList.push_back(QString::fromStdString(filePath));
	tempList.push_back(QString::fromStdString(pointType));

	cloudName = new QTreeWidgetItem(ui->treeWidget, tempList);
	cloudName->setFlags(cloudName->flags() &(~Qt::ItemIsEditable));

	QStringList tempList_sub1;
	tempList_sub1.push_back(QString("点数: "));
	tempList_sub1.push_back(QString::number(pointNum));
	pointsSize = new QTreeWidgetItem(cloudName, tempList_sub1);

	QStringList tempList_sub2;
	tempList_sub2.push_back(QString("面数: "));
	tempList_sub2.push_back(QString::number(faceNum));
	faceSize = new QTreeWidgetItem(cloudName, tempList_sub2);

	type = pointType;
}
EntityTree::~EntityTree()
{
	delete pointsSize;
	delete faceSize;
	delete cloudName;
	pointsSize = nullptr;
	faceSize = nullptr;
	cloudName = nullptr;
}




