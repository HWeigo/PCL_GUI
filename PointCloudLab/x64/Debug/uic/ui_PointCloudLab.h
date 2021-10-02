/********************************************************************************
** Form generated from reading UI file 'PointCloudLab.ui'
**
** Created by: Qt User Interface Compiler version 5.9.9
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_POINTCLOUDLAB_H
#define UI_POINTCLOUDLAB_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QFrame>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenu>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QTextBrowser>
#include <QtWidgets/QTreeWidget>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>
#include "QVTKWidget.h"

QT_BEGIN_NAMESPACE

class Ui_PointCloudLabClass
{
public:
    QAction *openFileAction;
    QAction *saveFileAction;
    QAction *filterAction1;
    QAction *filterAction2;
    QAction *filterAction3;
    QAction *copyPointAction;
    QAction *extractPointAction;
    QWidget *centralWidget;
    QVBoxLayout *verticalLayout;
    QWidget *widget_3;
    QFrame *line_2;
    QWidget *widget_2;
    QHBoxLayout *horizontalLayout;
    QVTKWidget *qvtkWidget;
    QFrame *line;
    QWidget *widget;
    QVBoxLayout *verticalLayout_3;
    QGroupBox *groupBox;
    QHBoxLayout *horizontalLayout_2;
    QTreeWidget *treeWidget;
    QGroupBox *groupBox_2;
    QVBoxLayout *verticalLayout_2;
    QTextBrowser *textBrowser;
    QMenuBar *menuBar;
    QMenu *menuFile;
    QMenu *menuFilter;
    QMenu *menuFit;
    QMenu *menuMatch;
    QMenu *menuMesh;
    QMenu *menuMeasure;
    QMenu *menuRoi;

    void setupUi(QMainWindow *PointCloudLabClass)
    {
        if (PointCloudLabClass->objectName().isEmpty())
            PointCloudLabClass->setObjectName(QStringLiteral("PointCloudLabClass"));
        PointCloudLabClass->resize(1280, 720);
        PointCloudLabClass->setMinimumSize(QSize(1280, 720));
        openFileAction = new QAction(PointCloudLabClass);
        openFileAction->setObjectName(QStringLiteral("openFileAction"));
        saveFileAction = new QAction(PointCloudLabClass);
        saveFileAction->setObjectName(QStringLiteral("saveFileAction"));
        filterAction1 = new QAction(PointCloudLabClass);
        filterAction1->setObjectName(QStringLiteral("filterAction1"));
        filterAction2 = new QAction(PointCloudLabClass);
        filterAction2->setObjectName(QStringLiteral("filterAction2"));
        filterAction3 = new QAction(PointCloudLabClass);
        filterAction3->setObjectName(QStringLiteral("filterAction3"));
        copyPointAction = new QAction(PointCloudLabClass);
        copyPointAction->setObjectName(QStringLiteral("copyPointAction"));
        extractPointAction = new QAction(PointCloudLabClass);
        extractPointAction->setObjectName(QStringLiteral("extractPointAction"));
        centralWidget = new QWidget(PointCloudLabClass);
        centralWidget->setObjectName(QStringLiteral("centralWidget"));
        verticalLayout = new QVBoxLayout(centralWidget);
        verticalLayout->setSpacing(0);
        verticalLayout->setContentsMargins(11, 11, 11, 11);
        verticalLayout->setObjectName(QStringLiteral("verticalLayout"));
        verticalLayout->setContentsMargins(0, 0, 0, 0);
        widget_3 = new QWidget(centralWidget);
        widget_3->setObjectName(QStringLiteral("widget_3"));
        widget_3->setMinimumSize(QSize(0, 30));
        widget_3->setMaximumSize(QSize(16777215, 30));

        verticalLayout->addWidget(widget_3);

        line_2 = new QFrame(centralWidget);
        line_2->setObjectName(QStringLiteral("line_2"));
        line_2->setFrameShape(QFrame::HLine);
        line_2->setFrameShadow(QFrame::Sunken);

        verticalLayout->addWidget(line_2);

        widget_2 = new QWidget(centralWidget);
        widget_2->setObjectName(QStringLiteral("widget_2"));
        horizontalLayout = new QHBoxLayout(widget_2);
        horizontalLayout->setSpacing(0);
        horizontalLayout->setContentsMargins(11, 11, 11, 11);
        horizontalLayout->setObjectName(QStringLiteral("horizontalLayout"));
        horizontalLayout->setContentsMargins(0, 0, 0, 0);
        qvtkWidget = new QVTKWidget(widget_2);
        qvtkWidget->setObjectName(QStringLiteral("qvtkWidget"));

        horizontalLayout->addWidget(qvtkWidget);

        line = new QFrame(widget_2);
        line->setObjectName(QStringLiteral("line"));
        line->setFrameShape(QFrame::VLine);
        line->setFrameShadow(QFrame::Sunken);

        horizontalLayout->addWidget(line);

        widget = new QWidget(widget_2);
        widget->setObjectName(QStringLiteral("widget"));
        widget->setMinimumSize(QSize(300, 0));
        widget->setMaximumSize(QSize(300, 16777215));
        verticalLayout_3 = new QVBoxLayout(widget);
        verticalLayout_3->setSpacing(0);
        verticalLayout_3->setContentsMargins(11, 11, 11, 11);
        verticalLayout_3->setObjectName(QStringLiteral("verticalLayout_3"));
        verticalLayout_3->setContentsMargins(0, 0, 0, 0);
        groupBox = new QGroupBox(widget);
        groupBox->setObjectName(QStringLiteral("groupBox"));
        horizontalLayout_2 = new QHBoxLayout(groupBox);
        horizontalLayout_2->setSpacing(0);
        horizontalLayout_2->setContentsMargins(11, 11, 11, 11);
        horizontalLayout_2->setObjectName(QStringLiteral("horizontalLayout_2"));
        horizontalLayout_2->setContentsMargins(0, 0, 0, 0);
        treeWidget = new QTreeWidget(groupBox);
        QTreeWidgetItem *__qtreewidgetitem = new QTreeWidgetItem();
        __qtreewidgetitem->setText(0, QStringLiteral("1"));
        treeWidget->setHeaderItem(__qtreewidgetitem);
        treeWidget->setObjectName(QStringLiteral("treeWidget"));
        treeWidget->setMouseTracking(true);
        treeWidget->setFocusPolicy(Qt::StrongFocus);

        horizontalLayout_2->addWidget(treeWidget);


        verticalLayout_3->addWidget(groupBox);

        groupBox_2 = new QGroupBox(widget);
        groupBox_2->setObjectName(QStringLiteral("groupBox_2"));
        verticalLayout_2 = new QVBoxLayout(groupBox_2);
        verticalLayout_2->setSpacing(0);
        verticalLayout_2->setContentsMargins(11, 11, 11, 11);
        verticalLayout_2->setObjectName(QStringLiteral("verticalLayout_2"));
        verticalLayout_2->setContentsMargins(0, 0, 0, 0);
        textBrowser = new QTextBrowser(groupBox_2);
        textBrowser->setObjectName(QStringLiteral("textBrowser"));
        textBrowser->setAcceptRichText(false);

        verticalLayout_2->addWidget(textBrowser);


        verticalLayout_3->addWidget(groupBox_2);


        horizontalLayout->addWidget(widget);


        verticalLayout->addWidget(widget_2);

        PointCloudLabClass->setCentralWidget(centralWidget);
        menuBar = new QMenuBar(PointCloudLabClass);
        menuBar->setObjectName(QStringLiteral("menuBar"));
        menuBar->setGeometry(QRect(0, 0, 1280, 37));
        menuFile = new QMenu(menuBar);
        menuFile->setObjectName(QStringLiteral("menuFile"));
        menuFilter = new QMenu(menuBar);
        menuFilter->setObjectName(QStringLiteral("menuFilter"));
        menuFit = new QMenu(menuBar);
        menuFit->setObjectName(QStringLiteral("menuFit"));
        menuMatch = new QMenu(menuBar);
        menuMatch->setObjectName(QStringLiteral("menuMatch"));
        menuMesh = new QMenu(menuBar);
        menuMesh->setObjectName(QStringLiteral("menuMesh"));
        menuMeasure = new QMenu(menuBar);
        menuMeasure->setObjectName(QStringLiteral("menuMeasure"));
        menuRoi = new QMenu(menuBar);
        menuRoi->setObjectName(QStringLiteral("menuRoi"));
        PointCloudLabClass->setMenuBar(menuBar);

        menuBar->addAction(menuFile->menuAction());
        menuBar->addAction(menuFilter->menuAction());
        menuBar->addAction(menuFit->menuAction());
        menuBar->addAction(menuMatch->menuAction());
        menuBar->addAction(menuMesh->menuAction());
        menuBar->addAction(menuMeasure->menuAction());
        menuBar->addAction(menuRoi->menuAction());
        menuFile->addAction(openFileAction);
        menuFile->addAction(saveFileAction);
        menuFilter->addAction(filterAction1);
        menuFilter->addAction(filterAction2);
        menuFilter->addAction(filterAction3);
        menuRoi->addAction(copyPointAction);
        menuRoi->addAction(extractPointAction);

        retranslateUi(PointCloudLabClass);

        QMetaObject::connectSlotsByName(PointCloudLabClass);
    } // setupUi

    void retranslateUi(QMainWindow *PointCloudLabClass)
    {
        PointCloudLabClass->setWindowTitle(QApplication::translate("PointCloudLabClass", "PointCloudLab", Q_NULLPTR));
        openFileAction->setText(QApplication::translate("PointCloudLabClass", "Open File", Q_NULLPTR));
        saveFileAction->setText(QApplication::translate("PointCloudLabClass", "Save File", Q_NULLPTR));
        filterAction1->setText(QApplication::translate("PointCloudLabClass", "\347\233\264\351\200\232\346\273\244\346\263\242", Q_NULLPTR));
        filterAction2->setText(QApplication::translate("PointCloudLabClass", "\344\275\223\347\264\240\346\273\244\346\263\242\n"
"", Q_NULLPTR));
        filterAction3->setText(QApplication::translate("PointCloudLabClass", "\347\273\237\350\256\241\346\273\244\346\263\242", Q_NULLPTR));
        copyPointAction->setText(QApplication::translate("PointCloudLabClass", "\345\244\215\345\210\266\346\211\200\351\200\211\347\202\271\344\272\221", Q_NULLPTR));
        extractPointAction->setText(QApplication::translate("PointCloudLabClass", "\346\217\220\345\217\226\346\211\200\351\200\211\347\202\271\344\272\221", Q_NULLPTR));
        groupBox->setTitle(QApplication::translate("PointCloudLabClass", "\346\211\223\345\274\200\347\232\204\347\202\271\344\272\221", Q_NULLPTR));
        groupBox_2->setTitle(QApplication::translate("PointCloudLabClass", "\350\276\223\345\207\272", Q_NULLPTR));
        menuFile->setTitle(QApplication::translate("PointCloudLabClass", "File", Q_NULLPTR));
        menuFilter->setTitle(QApplication::translate("PointCloudLabClass", "Filter", Q_NULLPTR));
        menuFit->setTitle(QApplication::translate("PointCloudLabClass", "Fit", Q_NULLPTR));
        menuMatch->setTitle(QApplication::translate("PointCloudLabClass", "Match", Q_NULLPTR));
        menuMesh->setTitle(QApplication::translate("PointCloudLabClass", "Mesh", Q_NULLPTR));
        menuMeasure->setTitle(QApplication::translate("PointCloudLabClass", "Measure", Q_NULLPTR));
        menuRoi->setTitle(QApplication::translate("PointCloudLabClass", "Roi", Q_NULLPTR));
    } // retranslateUi

};

namespace Ui {
    class PointCloudLabClass: public Ui_PointCloudLabClass {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_POINTCLOUDLAB_H
