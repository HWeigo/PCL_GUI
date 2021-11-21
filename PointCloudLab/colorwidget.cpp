#include "colorwidget.h"

ColorWidget::ColorWidget(QWidget *parent)
	: QWidget(parent)
{
	ui.setupUi(this);
}

ColorWidget::~ColorWidget()
{
}

int ColorWidget::current_index()
{
	return 0;
}
