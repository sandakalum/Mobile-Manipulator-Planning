#include <QtGui/QApplication>
#include "mainwindow.h"
#include <QSplashScreen>
#include <unistd.h>


int main(int argc, char *argv[])
{


    QApplication a(argc, argv);
    Q_INIT_RESOURCE(images);

    // Show a loading screen, I am not sure this is showed even on slow computers...
    QSplashScreen *splash = new QSplashScreen;
    splash->setPixmap(QPixmap("../resources/images/splash.jpg"));
    splash->show();
    splash->showMessage(QObject::tr("ROS GUI is launching...."),Qt::AlignBottom,Qt::black);

    //Make the main window appear
    MainWindow w;
    w.show();
    w.trajView.size_changed();

    // The main window is fully loaded, we can now delete the splash screen
    splash->finish(&w);
    delete splash;

    a.setStyle("gtk+");
    a.exec();

    return 0;
}
