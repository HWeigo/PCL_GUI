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
	pointsSize = new QTreeWidgetItem(cloudName, QStringList(QString("点数: ") + QString::number(pointNum)));
	faceSize = new QTreeWidgetItem(cloudName, QStringList(QString("面数: ") + QString::number(faceNum)));
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




