/********************************************************************************
** Form generated from reading UI file 'colorwidget.ui'
**
** Created by: Qt User Interface Compiler version 5.9.9
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_COLORWIDGET_H
#define UI_COLORWIDGET_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QRadioButton>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_ColorWidget
{
public:
    QHBoxLayout *horizontalLayout;
    QLabel *label;
    QGroupBox *groupBox;
    QVBoxLayout *verticalLayout;
    QRadioButton *radioButton;
    QRadioButton *radioButton_2;
    QRadioButton *radioButton_3;

    void setupUi(QWidget *ColorWidget)
    {
        if (ColorWidget->objectName().isEmpty())
            ColorWidget->setObjectName(QStringLiteral("ColorWidget"));
        ColorWidget->resize(370, 221);
        horizontalLayout = new QHBoxLayout(ColorWidget);
        horizontalLayout->setSpacing(6);
        horizontalLayout->setContentsMargins(11, 11, 11, 11);
        horizontalLayout->setObjectName(QStringLiteral("horizontalLayout"));
        label = new QLabel(ColorWidget);
        label->setObjectName(QStringLiteral("label"));

        horizontalLayout->addWidget(label);

        groupBox = new QGroupBox(ColorWidget);
        groupBox->setObjectName(QStringLiteral("groupBox"));
        verticalLayout = new QVBoxLayout(groupBox);
        verticalLayout->setSpacing(6);
        verticalLayout->setContentsMargins(11, 11, 11, 11);
        verticalLayout->setObjectName(QStringLiteral("verticalLayout"));
        radioButton = new QRadioButton(groupBox);
        radioButton->setObjectName(QStringLiteral("radioButton"));

        verticalLayout->addWidget(radioButton);

        radioButton_2 = new QRadioButton(groupBox);
        radioButton_2->setObjectName(QStringLiteral("radioButton_2"));

        verticalLayout->addWidget(radioButton_2);

        radioButton_3 = new QRadioButton(groupBox);
        radioButton_3->setObjectName(QStringLiteral("radioButton_3"));

        verticalLayout->addWidget(radioButton_3);


        horizontalLayout->addWidget(groupBox);


        retranslateUi(ColorWidget);

        QMetaObject::connectSlotsByName(ColorWidget);
    } // setupUi

    void retranslateUi(QWidget *ColorWidget)
    {
        ColorWidget->setWindowTitle(QApplication::translate("ColorWidget", "ColorWidget", Q_NULLPTR));
        label->setText(QApplication::translate("ColorWidget", "TextLabel", Q_NULLPTR));
        groupBox->setTitle(QApplication::translate("ColorWidget", "GroupBox", Q_NULLPTR));
        radioButton->setText(QApplication::translate("ColorWidget", "RadioButton1", Q_NULLPTR));
        radioButton_2->setText(QApplication::translate("ColorWidget", "RadioButton2", Q_NULLPTR));
        radioButton_3->setText(QApplication::translate("ColorWidget", "RadioButton3", Q_NULLPTR));
    } // retranslateUi

};

namespace Ui {
    class ColorWidget: public Ui_ColorWidget {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_COLORWIDGET_H
