#include "data_trans.h"
#include "ui_data_trans.h"
#include "myglwidget.h"
#include "drone_image.h"
#include "mainwindow.h"
#include <QApplication>
#include <QTimer>
#include <stdint.h>

data_trans::data_trans(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::data_trans)
{

    ui->setupUi(this);




}

data_trans::~data_trans()
{
    delete ui;


}


void data_trans::on_pushButton_clicked(double x,double y, double degree)
{


}

/*
int16_t zyro_x, zyro_y ,zyro_z;
int16_t acc_x, acc_y ,acc_z;
int curr_x, curr_y, curr_z;
int read_m_pwm0, read_m_pwm1, read_m_pwm2, read_m_pwm3;
int roll, pitch, yaw;
int bat, press, temp, sonic;
*/
