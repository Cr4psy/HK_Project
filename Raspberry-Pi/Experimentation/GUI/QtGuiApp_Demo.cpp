#include "QtGuiApp_Demo.h"
#include <qpainter.h>
#include <qevent.h>
#include <qmessagebox.h>
#include <QPointF>
#include <cstring>
#include <QTimer>
#include <QDebug>
constexpr int offset = 10; //ugly solution but without that, the outer circle is cut out. 
constexpr float PI = 3.14159265;
using namespace std;

/* 
 *-------------------------------------------------------------
 * MODIFY THOSE CONSTANTS TO CHANGE THE WAY THE DRAWING ZONE LOOK
 * -------------------------------------------------------------
*/


/* Position of the drawing area relative to the main windows */

constexpr float cst_x_dist_drawing_area = 30;
constexpr float cst_y_dist_drawing_area = 30;


/* Dimensions of the circles and other drawings parameters*/

constexpr float cst_in_dia = 400;
constexpr float cst_out_dia = 600;
constexpr float cst_gap_angle = 2;
constexpr int   cst_border_thickness = 2;
constexpr int   cst_sensor_nb = 15;

/* To change the colors (html code)*/
									
constexpr float cst_border_color = 222233; 
constexpr float cst_bckgnd_color = 442156;
constexpr int   cst_tranparency = 50;

/* To change the colors (html code) of the area with focus*/

constexpr float cst_transparency_focus = 250;
string zone1_area;
string zone2_area;
string zone3_area;

//Constructor for the main windows

QtGuiApp_Demo::QtGuiApp_Demo(QWidget *parent)
	: QMainWindow(parent)
{
	
	auto* drawing_area = new Drawing_area( //create the drawing area object (widget) 
		cst_in_dia, 
		cst_out_dia,
        cst_gap_angle,
        cst_border_thickness,
		cst_sensor_nb,
		this);

	drawing_area->setGeometry( //set its dimensions
		cst_x_dist_drawing_area, 
		cst_y_dist_drawing_area, 
		cst_out_dia + 2 * offset, 
		cst_out_dia + 2 * offset);

    QTimer *timer = new QTimer(this);
    qDebug() << connect(timer, SIGNAL(timeout()),this,SLOT(update_area()));
    timer->start(1000);

    ui.setupUi(this);

}

// Constructor for the drawing area widget (the one on which we draw)

Drawing_area::Drawing_area(
	const float in_dia,
	const float out_dia,
    const float gap_angle,
    const int border_thickness,
	const int sensor_nb,
	QWidget* parent)
	:
	_in_dia(in_dia),
	_out_dia(out_dia),
    _gap_angle(gap_angle),
    _border_thickness(border_thickness),
	_sensor_nb(sensor_nb),
    QWidget(parent)
{

    create_paths_to_paint();
    show();
	
	
}

/* This function is used to create the QPainterPath (a shape basically) that are used later on to draw in the QPaint function*/

void Drawing_area::create_paths_to_paint()
{

    _circle_center = QPointF(offset + _out_dia / 2, offset + _out_dia / 2); //measured from the left upper window corner

    QPainterPath radar;
    radar.moveTo(310,301);
    radar.arcTo(10,1,600,600,65,50);
    radar.closeSubpath();

    QPainterPath rectangle_zone1;
    rectangle_zone1.addRect(10,1,600,181);

    QPainterPath zone1;
    zone1 = rectangle_zone1.intersected(radar);
    zone1.closeSubpath();
    _paths.push_back(zone1);

    QPainterPath pathIRFront;
    pathIRFront.moveTo(310,181);
    pathIRFront.lineTo(310,301);

    QPainterPath zone2;
    zone2 = pathIRFront;
    _paths.push_back(zone2);

    QPainterPath rectangle_zone3;
    rectangle_zone3.addRect(312,181,120,120);

    QPainterPath zone3;
    zone3 = rectangle_zone3.intersected(radar);
    zone3.closeSubpath();
    _paths.push_back(zone3);

    QPainterPath rectangle_zone4;
    rectangle_zone4.addRect(QRectF(QPointF(120,181),QPointF(308,301)));

    QPainterPath zone4;
    zone4 = rectangle_zone4.intersected(radar);
    zone4.closeSubpath();
    _paths.push_back(zone4);

    QPainterPath pathIRRight;
    pathIRRight.moveTo(375,198);
    pathIRRight.lineTo(315,302);
    pathIRRight.closeSubpath();

    QPainterPath pathIRLeft;
    pathIRLeft.moveTo(245,198);
    pathIRLeft.lineTo(305,302);

    QPainterPath rectangle_zone5;
    rectangle_zone5.addRect(QRectF(QPointF(345,198),QPointF(375,250)));

    QPainterPath rectangle_zone6;
    rectangle_zone6.addRect(QRectF(QPointF(275,198),QPointF(245,250)));

    QPainterPath rectangle_zone7;
    rectangle_zone7.addRect(QRectF(QPointF(315,250),QPointF(345,302)));

    QPainterPath rectangle_zone8;
    rectangle_zone8.addRect(QRectF(QPointF(275,250),QPointF(305,302)));

    QPainterPath zone5;
    zone5 = rectangle_zone5.intersected(pathIRRight);
    zone5.closeSubpath();
    _paths.push_back(zone5);

    QPainterPath zone6;
    zone6 = rectangle_zone6.intersected(pathIRLeft);
    zone6.closeSubpath();
    _paths.push_back(zone6);

    QPainterPath zone7;
    zone7 = rectangle_zone7.intersected(pathIRRight);
    zone7.closeSubpath();
    _paths.push_back(zone7);

    QPainterPath zone8;
    zone8 = rectangle_zone8.intersected(pathIRLeft);
    zone8.closeSubpath();
    _paths.push_back(zone8);

    QPainterPath pathsonarFrontRight;
    pathsonarFrontRight.moveTo(316,304);
    pathsonarFrontRight.lineTo(323,272);
    pathsonarFrontRight.lineTo(431,164);
    pathsonarFrontRight.lineTo(456,189);
    pathsonarFrontRight.lineTo(348,297);
    pathsonarFrontRight.closeSubpath();

    QPainterPath pathsonarFrontLeft;
    pathsonarFrontLeft.moveTo(304,304);
    pathsonarFrontLeft.lineTo(297,272);
    pathsonarFrontLeft.lineTo(189,164);
    pathsonarFrontLeft.lineTo(164,189);
    pathsonarFrontLeft.lineTo(272,297);
    pathsonarFrontLeft.closeSubpath();

    QPainterPath pathsonarRight;
    pathsonarRight.moveTo(319,310);
    pathsonarRight.lineTo(346,292);
    pathsonarRight.lineTo(499,292);
    pathsonarRight.lineTo(499,328);
    pathsonarRight.lineTo(346,328);
    pathsonarRight.closeSubpath();

    QPainterPath pathsonarLeft;
    pathsonarLeft.moveTo(301,310);
    pathsonarLeft.lineTo(274,292);
    pathsonarLeft.lineTo(121,292);
    pathsonarLeft.lineTo(121,328);
    pathsonarLeft.lineTo(274,328);
    pathsonarLeft.closeSubpath();

    QPainterPath zone9;
    zone9 = pathsonarFrontRight.subtracted(pathsonarRight);
    zone9.closeSubpath();
    _paths.push_back(zone9);

    QPainterPath zone10;
    zone10 = pathsonarFrontLeft.subtracted(pathsonarLeft);
    zone10.closeSubpath();
    _paths.push_back(zone10);

    QPainterPath zone11;
    zone11 = pathsonarFrontRight.intersected(pathsonarRight);
    zone11.closeSubpath();
    _paths.push_back(zone11);

    QPainterPath zone12;
    zone12 = pathsonarFrontLeft.intersected(pathsonarLeft);
    zone12.closeSubpath();
    _paths.push_back(zone12);

    QPainterPath zone13;
    zone13 = pathsonarRight.subtracted(pathsonarFrontRight);
    zone13.closeSubpath();
    _paths.push_back(zone13);

    QPainterPath zone14;
    zone14 = pathsonarLeft.subtracted(pathsonarFrontLeft);
    zone14.closeSubpath();
    _paths.push_back(zone14);

    QPainterPath pathsonarBack;
    pathsonarBack.moveTo(310,319);
    pathsonarBack.lineTo(328,346);
    pathsonarBack.lineTo(328,499);
    pathsonarBack.lineTo(292,499);
    pathsonarBack.lineTo(292,346);
    pathsonarBack.closeSubpath();

    QPainterPath zone15;
    zone15 = pathsonarBack;
    _paths.push_back(pathsonarBack);

}

/* This function is called each time the widget is updated. It's used to draw on the widget*/

void Drawing_area::paintEvent(
	QPaintEvent *)
{
    QColor border_color(0,0,0);
    QColor bckgnd_color(192,192,192);
	bckgnd_color.setAlpha(cst_tranparency);

	QPainter painter(this);
	painter.setPen(QPen(QColor(border_color), _border_thickness, Qt::SolidLine, Qt::SquareCap, Qt::BevelJoin));
	painter.setRenderHints(QPainter::HighQualityAntialiasing);

	painter.setBrush(QColor(bckgnd_color));

    for (auto& zone1 : _paths) {
        painter.drawPath(zone1);
    }

    painter.drawEllipse(_circle_center,9,9);
    painter.drawPoint(310,310);

    for (int i = 0; i < 3; i++){

        if (!(zone_area[i] == "")){ //change the colors of the QPainterPath targetted by the sensors

            QColor border_color_focus;
            QColor bckgnd_color_focus;

            if (zone_area[i] == "Green"){
                border_color_focus.setGreen(255);
                bckgnd_color_focus.setGreen(153);
            }
            if (zone_area[i] == "Yellow"){
                border_color_focus.setRed(255);
                border_color_focus.setGreen(255);
                bckgnd_color_focus.setRed(204);
                bckgnd_color_focus.setGreen(204);
            }
            if (zone_area[i] == "Red"){
                border_color_focus.setRed(255);
                bckgnd_color_focus.setRed(204);
            }

            bckgnd_color_focus.setAlpha(cst_transparency_focus);

            painter.setPen(QPen(QColor(bckgnd_color_focus), _border_thickness, Qt::SolidLine, Qt::SquareCap, Qt::BevelJoin));
            painter.setBrush(QColor(border_color_focus));
            painter.drawPath(_paths.at(i));
        }
    }
}

void QtGuiApp_Demo::update_area(){
    zone1_area = "Yellow";
    zone2_area = "Green";
    zone3_area = "Red";

    zone_area[0] = zone1_area;
    zone_area[1] = zone2_area;
    zone_area[2] = zone3_area;

    qDebug() << "Updating";

    update();
}

