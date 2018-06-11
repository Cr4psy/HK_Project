#pragma once

#include <QtWidgets/QMainWindow>
#include "ui_QtGuiApp_Demo.h"
#include <cstring>
#include <vector>

using paths_to_paint = std::vector<QPainterPath>;

using namespace std;
extern string zone_area[3];

class QtGuiApp_Demo : public QMainWindow
{
    Q_OBJECT

private slots:
    void update_area();

public:
    QtGuiApp_Demo(QWidget *parent = Q_NULLPTR);

private:
    Ui::QtGuiApp_DemoClass ui;
};


class Drawing_area : public QWidget
{

public:
	Drawing_area(
		const float inner_dia,
		const float outer_dia,
		const float gap_angle,
		const int border_thickness,
        const int sensor_nb = 3,
        QWidget* parent = Q_NULLPTR);


private :
	void create_paths_to_paint();
    void paintEvent(QPaintEvent *);

	float			_in_dia;
	float			_out_dia;
	float			_gap_angle;
	int				_border_thickness;
	int				_sensor_nb;
	QPointF			_circle_center;

    paths_to_paint _paths;
};
