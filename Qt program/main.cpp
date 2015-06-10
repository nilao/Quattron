#include "mainwindow.h"
#include <QApplication>
#include <qwt.h>



int main(int argc, char *argv[])
{
//   QString buf = "d"
//system()
    QApplication a(argc, argv);
    MainWindow w;
    w.show();

    return a.exec();
}
