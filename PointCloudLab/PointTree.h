#pragma once
#include "ui_PointCloudLab.h"

#include <string>
using namespace std;

class PointTree
{
public:
    PointTree(Ui::PointCloudLabClass *input_ui, std::string filePath, int pointNum, int faceNum);
    ~PointTree();

    QTreeWidgetItem *cloudName;
    QTreeWidgetItem *pointsSize;
    QTreeWidgetItem *faceSize;
 
private:
    Ui::PointCloudLabClass *ui;
   
    


};

