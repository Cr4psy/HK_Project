#include "QtGuiApp_Demo.h"
#include <QtWidgets/QApplication>
#include <qdesktopwidget.h>
#include <cstring>
#include <unistd.h>
#include <QWidget>


using namespace std;

/* To modify the dimensions of the windows */

constexpr float window_width = 700;
constexpr float window_height = 700;

string zone_area[3];

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
	QDesktopWidget desktop;
	QRect screenRectangle = desktop.availableGeometry();
    QtGuiApp_Demo w;
	w.setGeometry(
		(screenRectangle.width() - window_width)/2, 
		(screenRectangle.height() - window_height)/2,
		window_width,
		window_height);

	w.setWindowFlags(Qt::MSWindowsFixedSizeDialogHint);
    w.show();

    return a.exec();
}
