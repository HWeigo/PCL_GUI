#pragma once
#include "ui_PointCloudLab.h"

#include <string>
using namespace std;

class PointTree
{
public:
	PointTree(Ui::PointCloudLabClass *input_ui, std::string filePath, std::string pointType, int pointNum, int faceNum);
	~PointTree();

	QTreeWidgetItem *cloudName;
	QTreeWidgetItem *pointsSize;
	QTreeWidgetItem *faceSize;
	string type;

private:
	Ui::PointCloudLabClass *ui;

};

