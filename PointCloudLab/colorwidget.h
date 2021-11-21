#pragma once

#include <QWidget>
#include "ui_colorwidget.h"

class ColorWidget : public QWidget
{
	Q_OBJECT

public:
	ColorWidget(QWidget *parent = Q_NULLPTR);
	~ColorWidget();
	
	int current_index();

private:
	Ui::ColorWidget ui;
};
