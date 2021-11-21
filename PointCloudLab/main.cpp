#include "PointCloudLab.h"
#include <QtWidgets/QApplication>

#include <vtkAutoInit.h>
#pragma comment( linker, "/subsystem:windows /entry:mainCRTStartup" )//不显示控制台
VTK_MODULE_INIT(vtkRenderingOpenGL2);
VTK_MODULE_INIT(vtkInteractionStyle);

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    PointCloudLab w;
    w.show();
    return a.exec();
}
