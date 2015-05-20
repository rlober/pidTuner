#include "mainwindow.h"
#include <QApplication>


int main(int argc, char *argv[])
{
    QApplication a(argc, argv);

    MainWindow w;
    w.setWindowTitle("PID Tuner");
    w.show();

    w.initializeGui();

    return a.exec();
}
