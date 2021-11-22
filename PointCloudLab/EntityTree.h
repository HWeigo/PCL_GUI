#pragma once
#include "ui_PointCloudLab.h"

#include <string>
using namespace std;

class EntityTree
{
public:
	EntityTree(Ui::PointCloudLabClass *input_ui, std::string filePath, std::string pointType, int pointNum, int faceNum);
	~EntityTree();

	QTreeWidgetItem *cloudName;
	QTreeWidgetItem *pointsSize;
	QTreeWidgetItem *faceSize;
	string type;

private:
	Ui::PointCloudLabClass *ui;

};

