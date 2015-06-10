#ifndef DATA_TRANS_H
#define DATA_TRANS_H

#include <QDialog>
//#include "mainwindow.h"
#include "myglwidget.h"

namespace Ui {
class data_trans;

}

class data_trans : public QDialog
{
    Q_OBJECT

public:

    explicit data_trans(QWidget *parent = 0);
    ~data_trans();
    MyGLWidget cube; //MyGLWidget 을cube 로불러옴
    //MainWindow mainwin;
    //void data_trans::recive_data(int recive_data);
    void on_pushButton_clicked(double x,double y, double degree);


private slots:

private:
    Ui::data_trans *ui;
};

#endif // DATA_TRANS_H
