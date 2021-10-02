#include "PointTree.h"

#pragma execution_character_set("utf-8")

PointTree::PointTree(Ui::PointCloudLabClass *input_ui, string filePath, int pointNum, int faceNum)
{
    ui = input_ui;
    cloudName = new QTreeWidgetItem(ui->treeWidget, QStringList(QString::fromStdString(filePath)));
    pointsSize = new QTreeWidgetItem(cloudName, QStringList(QString("点数: ")+QString::number(pointNum)));
    faceSize = new QTreeWidgetItem(cloudName, QStringList(QString("面数: ") + QString::number(faceNum)));
}
PointTree::~PointTree()
{
    delete pointsSize;
    delete faceSize;
    delete cloudName;
}




