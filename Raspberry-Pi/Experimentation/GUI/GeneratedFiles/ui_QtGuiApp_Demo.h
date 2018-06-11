/********************************************************************************
** Form generated from reading UI file 'QtGuiApp_Demo.ui'
**
** Created by: Qt User Interface Compiler version 5.9.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_QTGUIAPP_DEMO_H
#define UI_QTGUIAPP_DEMO_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QToolBar>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_QtGuiApp_DemoClass
{
public:
    QWidget *centralWidget;
    QWidget *widget_to_draw;
    QMenuBar *menuBar;
    QToolBar *mainToolBar;
    QStatusBar *statusBar;

    void setupUi(QMainWindow *QtGuiApp_DemoClass)
    {
        if (QtGuiApp_DemoClass->objectName().isEmpty())
            QtGuiApp_DemoClass->setObjectName(QStringLiteral("QtGuiApp_DemoClass"));
        QtGuiApp_DemoClass->resize(600, 400);
        centralWidget = new QWidget(QtGuiApp_DemoClass);
        centralWidget->setObjectName(QStringLiteral("centralWidget"));
        widget_to_draw = new QWidget(centralWidget);
        widget_to_draw->setObjectName(QStringLiteral("widget_to_draw"));
        widget_to_draw->setGeometry(QRect(50, 20, 491, 291));
        QtGuiApp_DemoClass->setCentralWidget(centralWidget);
        menuBar = new QMenuBar(QtGuiApp_DemoClass);
        menuBar->setObjectName(QStringLiteral("menuBar"));
        menuBar->setGeometry(QRect(0, 0, 600, 21));
        QtGuiApp_DemoClass->setMenuBar(menuBar);
        mainToolBar = new QToolBar(QtGuiApp_DemoClass);
        mainToolBar->setObjectName(QStringLiteral("mainToolBar"));
        QtGuiApp_DemoClass->addToolBar(Qt::TopToolBarArea, mainToolBar);
        statusBar = new QStatusBar(QtGuiApp_DemoClass);
        statusBar->setObjectName(QStringLiteral("statusBar"));
        QtGuiApp_DemoClass->setStatusBar(statusBar);

        retranslateUi(QtGuiApp_DemoClass);

        QMetaObject::connectSlotsByName(QtGuiApp_DemoClass);
    } // setupUi

    void retranslateUi(QMainWindow *QtGuiApp_DemoClass)
    {
        QtGuiApp_DemoClass->setWindowTitle(QApplication::translate("QtGuiApp_DemoClass", "QtGuiApp_Demo", Q_NULLPTR));
    } // retranslateUi

};

namespace Ui {
    class QtGuiApp_DemoClass: public Ui_QtGuiApp_DemoClass {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_QTGUIAPP_DEMO_H
