#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <qprocess.h>
#include <qdebug.h>
#include <unistd.h>
#include <QTimer>
#include <signal.h>
#include <QWidget>
#include <pthread.h>
#include <QEventLoop>
#include <QKeyEvent>
#include <QDateTime>
#include <QThread>
#include <QtMath>
#include <QApplication>
#include <QTcpServer>
#include <QRegExp>
#include <QTcpSocket>


MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    /************VALUE SET**********/
    _connect=false;


    /*************TELNET SET********/
    telnet = new QProcess(this);
    telnet->setProcessChannelMode(QProcess::MergedChannels);
    connect(telnet, SIGNAL(readyRead()), this, SLOT(read_telnet()));
    connect(telnet, SIGNAL(started()), this, SLOT(slotProcessStarted()));
    connect(telnet, SIGNAL(finished(int)), this, SLOT(slotProcessFinished()));

    /**********OPEN GL**************/
    cubeshow=1;
    xval=0,yval=0,zval=0;
    cube2 = new MyGLWidget;
    DrawCube(x,y,degree);
    init =0;
    /************GATT TOOL**********/
    gatttool = new QProcess(this);
    gatttool->setProcessChannelMode(QProcess::MergedChannels);
    connect(gatttool, SIGNAL(readyRead()), this, SLOT(read_gatttool()));
    connect(gatttool, SIGNAL(started()), this, SLOT(slotProcessStarted()));
    connect(gatttool, SIGNAL(finished(int)), this, SLOT(slotProcessFinished()));
    ping_timer = new QTimer(this);
    connect(ping_timer, SIGNAL(timeout()), this, SLOT(send_ping()));
    connect(ui->ble_connect_button, SIGNAL(clicked()),this, SLOT(gatttool_Connect()));
    cmd_value=5;
    ping_value=0;
    config_value=1;
     _connected=false;
    configset=true;
    /***********KEY EVENT*************/
    key_event=true;
    key_check_timer = new QTimer(this);
    //connect(key_check_timer, SIGNAL(timeout()), this, SLOT());

    /*********************************/
    mission_timer = new QTimer(this);
    connect(mission_timer, SIGNAL(timeout()), this, SLOT(show_time()));
    /************ADB SOCKET***********/
    /* nextBlocksize=0;
     connect(ui->socket_connect_Button, SIGNAL(clicked()), this, SLOT(connect_to_server()));
     connect(&adb_socket, SIGNAL(connected()), this, SLOT(on_Connect_Server()));
     connect(ui->socket_send_Button, SIGNAL(clicked()), this, SLOT(send_Request()));
     connect(&adb_socket, SIGNAL(readyRead()), this, SLOT(read_Message()));
     connect(&adb_socket, SIGNAL(disconnected()), this, SLOT(connection_Closed_By_Server()));
     connect(&adb_socket, SIGNAL(error(QAbstractSocket::SocketError)), this, SLOT(error()));*/



}

MainWindow::~MainWindow()
{
    delete ui;
}


/***************************************
    SYSTEM BUTTON
***************************************/

void MainWindow::on_scan_button_clicked()
{
    ui->textEdit->append("clicked scan\n");
    //hcitool scan

     QProcess p;
    QString cmd("hcitool scan");

        //start the process
        p.start(cmd);

        //Wait for the processs to finish with a timeout of 20 seconds
        if(p.waitForFinished(20000))
        {
            //Clear the list widget
            //this->ui->listWidget->clear();

            //Read the command line output and store it in QString out
            QString out(p.readAllStandardOutput());

            //Split the QString every new line and save theve in a QStringList
            QStringList OutSplit = out.split("\n");

            foreach(QString line , OutSplit){
                ui->textEdit->append(line);
            }

        }
        p.close();
}



void MainWindow::on_pand_button_clicked()
{
    system("pand --killall");
    QString cmd ="sudo pand";

        cmd.append(" --connect ");
        cmd.append(ui->pand_combobox->currentText());
        cmd.append(" -dGN -n");
        system(cmd.toUtf8());

     ui->ip_button->setEnabled(true);
    system("ifconfig bnep0 192.168.1.2 up");

}
void MainWindow::on_ip_button_clicked()
{
     QStringList args;
   if(ui->ip_button->text()=="CONNECT"){
       ui->textEdit->clear();
       ui->ip_line->setEnabled(false);
       ui->ip_button->setText("DIS CONNECT");
       if(telnet->isOpen()==true)
           telnet->finished(1);
       telnet->setProgram("telnet");
       qDebug()<<"telnet set";
       args<<ui->ip_line->text().toUtf8();
       qDebug()<<"telnet args set";
       telnet->setArguments(args);
       qDebug()<<args;
       telnet->start(QProcess::Unbuffered | QProcess::ReadWrite);
       qDebug()<<"telnet read write";
       if(telnet->waitForStarted(3000))
       {
           qDebug()<<"init if";
           telnet->setTextModeEnabled(true);
           ui->statusBar->showMessage("Connecting...");

       }

    }else{

       ui->lineEdit->setText("exit");
       write_telnet();

       ui->lineEdit->clear();;
       ui->textEdit->clear();
       ui->textEdit->append("disconnect");
       ui->ip_button->setText("CONNECT");
       ui->statusBar->showMessage("Disconnect");
       _connect=false;
       ui->ip_line->setEnabled(true);
   }
}

void MainWindow::on_ctrl_C_clicked()
{
    const char b=0x03;
    const char *a = &b;

     telnet->write(a);
     ui->lineEdit->clear();
     ui->lineEdit->setText(" ");
     write_telnet();
     ui->lineEdit->clear();
     ui->lineEdit->setText(" ");
     write_telnet();
     ui->lineEdit->clear();
}
void MainWindow::on_dragon_prog_clicked()
{
    ui->lineEdit->setText("dragon-prog");
    write_telnet();
    ui->lineEdit->clear();
    sleep(2);
    on_ctrl_C_clicked();
    ui->lineEdit->clear();
    ui->lineEdit->setText(" ");
    write_telnet();
}

void MainWindow::on_clear_button_clicked()
{
    ui->textEdit->clear();
}

/***************************************
    READ WRITE BUTTON
***************************************/

void MainWindow::on_lineEdit_editingFinished()
{
    write_telnet();
    ui->lineEdit->clear();
}

void MainWindow::read_telnet(){//slotConnect

    QString allText = telnet->readAll();
    QStringList trash_value,blue_data;
    QStringList lines = allText.split('\n');

    foreach(QString line, lines)
    {
        if(line.contains("[Delos]"))
        {
            if(!_connect)
                ui->statusBar->showMessage("Connected");
            _connect = true;
        }
        _dataReceived = true;

        if(line.contains("_J_DATA")){
            trash_value = line.split(":");
            blue_data = trash_value[1].split(" ");
            for(int i=0;i<blue_data.length();i++){
                    /******************DT*************/
                    if(blue_data.at(i).compare("dT")==0){
                        pre_DT=blue_data.at(i+1).toDouble()/1000000000;
                    }
                    //****************KALMAN DATA************
                    if(blue_data.at(i).compare("QR")==0){
                        zval=blue_data.at(i+1).toDouble();
                        ui->roll_out_line->setText(blue_data.at(i+1));
                        DrawCube(xval,yval,zval);
                    }else if(blue_data.at(i).compare("QP")==0){
                        xval=blue_data.at(i+1).toDouble()*-1;
                        ui->pitch_out_line->setText(blue_data.at(i+1));
                        DrawCube(xval,yval,zval);
                    }else if(blue_data.at(i).compare("QY")==0){
                        yval=blue_data.at(i+1).toDouble()*-1;
                        if(yval>180)
                            yval = yval-360;
                        else if(yval<-180)
                            yval = yval+360;
                        yval=yval-init;
                        QString yaw_qstring = QString::number(yval);
                        ui->yaw_out_line->setText(yaw_qstring);
                        DrawCube(xval,yval,zval);
                    }
                    //****************ACCEL DATA************
                    if(blue_data.at(i).compare("WX")==0){
                        //acc_x=blue_data.at(i+1).toDouble();
                        ui->accel_x_out_line->setText(blue_data.at(i+1));
                    }else if(blue_data.at(i).compare("WY")==0){
                        //acc_y=blue_data.at(i+1).toDouble();
                        ui->accel_y_out_line->setText(blue_data.at(i+1));
                    }else if(blue_data.at(i).compare("WZ")==0){
                        //acc_z=blue_data.at(i+1).toDouble();
                        ui->accel_z_out_line->setText(blue_data.at(i+1));
                    }
                    //****************GRYo DATA***********
                    if(blue_data.at(i).compare("GX")==0){
                        //gyro_x=blue_data.at(i+1).toDouble();
                        ui->gyro_x_out_line->setText(blue_data.at(i+1));
                    }else if(blue_data.at(i).compare("GY")==0){
                        //gyro_y=blue_data.at(i+1).toDouble();
                        ui->gyro_y_out_line->setText(blue_data.at(i+1));
                    }else if(blue_data.at(i).compare("GZ")==0){
                        //gyro_z=blue_data.at(i+1).toDouble();
                        ui->gyro_z_out_line->setText(blue_data.at(i+1));
                    }
                    //****************press DATA***********
                    if(blue_data.at(i).compare("GND")==0){
                        press_ground=blue_data.at(i+1).toDouble();
                        //ui->height_out_line->setText(blue_data.at(i+1));
                    }else if(blue_data.at(i).compare("NOW")==0){
                        height=(press_ground-blue_data.at(i+1).toDouble())/10;
                        QString _height =QString::number(height);
                        ui->height_out_line->setText(_height);
                    }
                    //*****************BATTERY****************
                    if(blue_data.at(i).compare("BAT")==0){
                        double bat_per;
                        //gyro_x=blue_data.at(i+1).toDouble();
                        bat_per=(blue_data.at(i+1).toDouble()-710)/200*100;
                        QString diff = QString("%1%").arg(bat_per);
                        ui->bat_label->setText(diff);
                    }
                }
            }
        if(line.contains("_J_DATA")){
            ui->textEdit->clear();
        }else
        ui->textEdit->append(line);
        }
}
void MainWindow::on_yaw_Button_clicked()
{
    init +=yval;
    //pre_init=init;

}

void MainWindow::write_telnet(){//slotTestRun

    QString test, text = ui->lineEdit-> text();
    QTextStream textStream(&text, QIODevice::ReadOnly);
    QEventLoop eventLoop;
    QTimer timer;
    connect(&timer, SIGNAL(timeout()), &eventLoop, SLOT(quit()));

    while((test = textStream.readLine()) != "")
    {
        test = test.remove("\n");
        if(test.contains("wait"))
        {
            timer.setSingleShot(true);
            disconnect(telnet, SIGNAL(readyRead()), &eventLoop, SLOT(quit()));
            timer.start(5000);
            eventLoop.exec();
            read_telnet();
        }
        else
        {
            connect(&timer, SIGNAL(timeout()), &eventLoop, SLOT(quit()));
            _dataReceived = false;
            telnet->write(test.toUtf8());

            timer.setSingleShot(false);
            timer.setInterval(50);
            timer.start();
            int retries = 100;
            while(!_dataReceived && retries--)
            {
                telnet->write("\n");
                eventLoop.exec(QEventLoop::ExcludeUserInputEvents);
            }

        }
    }
}

/***************************************
    OPEN GL
***************************************/

void MainWindow::DrawCube(double x,double y, double z)
{
    ui->gl_Layout->removeWidget(cube2);

    if(cubeshow==1){
        cube2->resize(500,400);
        ui->gl_Layout->addWidget(cube2);
        cubeshow=0;

    }
    qDebug()<<x;
    qDebug()<<y;
    qDebug()<<z;
    qDebug()<<"------------------";
    ui->gl_Layout->addWidget(cube2);

    cube2->SetXYZD(x,y,z);//cube에데이터전
    cube2->paintGL();//cube다시그리기
    cube2->update();//cube 갱신


    glTranslatef(0.0f,0.0f,-20); //move along z-axis

}

/***************************************
  MISSION BUTTON
***************************************/

void MainWindow::on_aout_clicked()
{

    ui->lineEdit->clear();
    ui->lineEdit->setText(" ");
    write_telnet();
    ui->lineEdit->setText("/bin/JAE/QTtest.out");
    write_telnet();
    ui->lineEdit->clear();

}
void MainWindow::on_sub_start_clicked()
{

    ui->lineEdit->setText(ui->sub_pro_lineEdit->text());
    write_telnet();
    ui->lineEdit->clear();

}
/***************************************
  SENSOR TEST
***************************************/

void MainWindow::on_sensor_button_clicked()
{
    if(ui->sensor_button->text()=="Sensor ON"){

        if(ui->accel_checkbox->checkState()==2)
        {
            ui->lineEdit->setText("4 7 1");
            write_telnet();
            ui->lineEdit->clear();
            qDebug()<<"accel on";
        }
        if(ui->gyro_checkbox->checkState()==2)
        {
            ui->lineEdit->setText("4 1 1");
            write_telnet();
            ui->lineEdit->clear();
            qDebug()<<"gyro on";
        }
        if(ui->kalman_checkbox->checkState()==2)
        {
            ui->lineEdit->setText("4 2 1");
            write_telnet();
            ui->lineEdit->clear();
            qDebug()<<"kalman on";
        }
        if(ui->bat_checkbox->checkState()==2)
        {
            ui->lineEdit->setText("4 3 1");
            write_telnet();
            ui->lineEdit->clear();
            qDebug()<<"BAT on";
        }
        if(ui->press_checkBox->checkState()==2)
        {
            ui->lineEdit->setText("4 4 1");
            write_telnet();
            ui->lineEdit->clear();
            qDebug()<<"BARO on";
        }
        ui->lineEdit->clear();
        ui->lineEdit->setText("1");
        write_telnet();
        ui->lineEdit->clear();
        sensor_line_clean();
        qDebug()<<"------------------------";
        ui->sensor_button->setText("Sensor Off");
    }else{
        ui->lineEdit->setText("4 1 0");
        write_telnet();
        ui->lineEdit->clear();

        ui->lineEdit->setText("4 2 0");
        write_telnet();
        ui->lineEdit->clear();

        ui->lineEdit->setText("4 3 0");
        write_telnet();
        ui->lineEdit->clear();

        ui->lineEdit->setText("4 4 0");
        write_telnet();
        ui->lineEdit->clear();

        ui->lineEdit->setText("0");
        write_telnet();
        ui->lineEdit->clear();
        ui->sensor_button->setText("Sensor ON");
    }
}
void MainWindow::sensor_line_clean(){
    ui->accel_x_out_line->clear();
    ui->accel_y_out_line->clear();
    ui->accel_z_out_line->clear();
    ui->gyro_x_out_line->clear();
    ui->gyro_y_out_line->clear();
    ui->gyro_z_out_line->clear();
    ui->pitch_out_line->clear();
    ui->roll_out_line->clear();
    ui->yaw_out_line->clear();
}

/***************************************
  MOTER SET SLOT
***************************************/

/*
void MainWindow::on_moter_speed_button_clicked()
{
    //QStringList args;

    motor0_val = ui->motor0_offset_line->text();
    motor1_val = ui->motor1_offset_line->text();
    motor2_val = ui->motor2_offset_line->text();
    motor3_val = ui->motor3_offset_line->text();
    //ui->lineEdit->setText("1 "+motor0_val+" "+motor1_val+" "+motor2_val+" "+motor3_val);
    //write_telnet();
    //ui->lineEdit->clear();




}

void MainWindow::on_moter_stop_clicked()
{
    ui->lineEdit->setText("1 0 0 0 0");
    write_telnet();
    ui->lineEdit->clear();
    ui->motor0_line->clear();
    ui->motor1_line->clear();
    ui->motor2_line->clear();
    ui->motor3_line->clear();
}
*/
/***************************************
  PID TEST
***************************************/


void MainWindow::on_pid_test_clicked()
{
    roll_kp = ui->roll_kp_line->text();
    roll_ki = ui->roll_ki_line->text();
    roll_kd = ui->roll_kd_line->text();
    pitch_kp = ui->pitch_kp_line->text();
    pitch_ki = ui->pitch_ki_line->text();
    pitch_kd = ui->pitch_kd_line->text();
    z_kp=ui->z_kp_line->text();
    z_ki=ui->z_ki_line->text();
    z_kd=ui->z_kd_line->text();
    double aa = ui->motor_throttle_line->text().toDouble()+ui->motor0_offset_line->text().toDouble();
    motor_throttle0 = QString::number(aa);
    aa = ui->motor_throttle_line->text().toDouble()+ui->motor1_offset_line->text().toDouble();
    motor_throttle1 = QString::number(aa);
     aa = ui->motor_throttle_line->text().toDouble()+ui->motor2_offset_line->text().toDouble();
    motor_throttle2 = QString::number(aa);
     aa = ui->motor_throttle_line->text().toDouble()+ui->motor3_offset_line->text().toDouble();
    motor_throttle3 = QString::number(aa);

    if(ui->pid_test->text()=="start")
     {
        ui->lineEdit->clear();
        ui->lineEdit->setText("2 0 "+roll_kp+" "+roll_ki+" "+roll_kd);
        write_telnet();
        ui->lineEdit->setText("2 1 "+pitch_kp+" "+pitch_ki+" "+pitch_kd);
        write_telnet();
        ui->lineEdit->setText("2 5 "+z_kp+" "+z_ki+" "+z_kd);
        write_telnet();
        ui->lineEdit->setText("3 "+motor_throttle0);
        write_telnet();
        ui->lineEdit->setText("1");
        write_telnet();
        ui->lineEdit->clear();
        ui->pid_test->setText("stop");
        pid_line_update(false);
     }else{
        ui->lineEdit->setText("0");
        write_telnet();
        ui->pid_test->setText("start");
        ui->lineEdit->clear();
        pid_line_update(true);
    }
}
void MainWindow::pid_line_update(bool state){
    if(state==true){

        ui->motor_throttle_line->setEnabled(true);
        ui->motor0_offset_line->setEnabled(true);
        ui->motor1_offset_line->setEnabled(true);
        ui->motor2_offset_line->setEnabled(true);
        ui->motor3_offset_line->setEnabled(true);
        ui->roll_kp_line->setEnabled(true);
        ui->roll_ki_line->setEnabled(true);
        ui->roll_kd_line->setEnabled(true);
        ui->pitch_kp_line->setEnabled(true);
        ui->pitch_ki_line->setEnabled(true);
        ui->pitch_kd_line->setEnabled(true);
        ui->z_kp_line->setEnabled(true);
        ui->z_ki_line->setEnabled(true);
        ui->z_kd_line->setEnabled(true);
    }else{
        ui->motor_throttle_line->setEnabled(false);
        ui->motor0_offset_line->setEnabled(false);
        ui->motor1_offset_line->setEnabled(false);
        ui->motor2_offset_line->setEnabled(false);
        ui->motor3_offset_line->setEnabled(false);
        ui->roll_kp_line->setEnabled(false);
        ui->roll_ki_line->setEnabled(false);
        ui->roll_kd_line->setEnabled(false);
        ui->pitch_kp_line->setEnabled(false);
        ui->pitch_ki_line->setEnabled(false);
        ui->pitch_kd_line->setEnabled(false);
        ui->z_kp_line->setEnabled(false);
        ui->z_ki_line->setEnabled(false);
        ui->z_kd_line->setEnabled(false);
    }

}




/***************************************
  trash slot
***************************************/
void MainWindow::slotProcessStarted(){
    qDebug() << __FUNCTION__;
}
void MainWindow::slotProcessFinished(){
    qDebug() << __FUNCTION__;
}

/***************************************
    GATT TOOL
***************************************/

void MainWindow::gatttool_Connect()
{
    QStringList args;

    if(!_connected)
    {
        gatttool->setProgram("gatttool");
        args << "-b" <<ui->ble_comboBox->currentText();
        args << "-I" << "-t";
        args << "random";
//        args << "-b 00:01:5B:00:15:21 -I -t random";
        gatttool->setArguments(args);
        gatttool->start(QProcess::Unbuffered | QProcess::ReadWrite);
        if(gatttool->waitForStarted(3000))
        {
            gatttool->setTextModeEnabled(true);
            gatttool->write("connect\n");
            ui->ble_label->setText("Connecting...");
        }
        else
        {
            qDebug() << "! failed to start gatttool";
        }
    }
    else
    {
        gatttool->write("exit\n");
        if(gatttool->waitForFinished(3000))
        {
            _connected = false;
            ui->ble_label->setText("Disconnect");
        }
        else
        {
            qDebug() << "! failed to quit gatttool";
        }
    }

    updateInterface();
}

void MainWindow::write_gatttool()
{
    QString test, text = ui->ble_lineEdit->text();
    QTextStream textStream(&text, QIODevice::ReadOnly);

    QEventLoop eventLoop;
    QTimer timer;
    connect(&timer, SIGNAL(timeout()), &eventLoop, SLOT(quit()));

    while((test = textStream.readLine()) != "")
    {
        test = test.remove("\n");
        if(test.at(0) == '#')
        {
            ui->ble_log_textEdit->append(test);
            continue;
        }

        qDebug() << "test:" << test;

        if(test.contains("wait"))
        {
            timer.setSingleShot(true);
            disconnect(gatttool, SIGNAL(readyRead()), &eventLoop, SLOT(quit()));
            timer.start(5000);
            eventLoop.exec();
            read_gatttool();
        }
        else
        {
            connect(&timer, SIGNAL(timeout()), &eventLoop, SLOT(quit()));

            ble_dataReceived = false;
            gatttool->write(test.toUtf8());

            timer.setSingleShot(false);
            timer.setInterval(50);
            timer.start();
            int retries = 100;
            while(!ble_dataReceived && retries--)
            {
                gatttool->write("\n");
                eventLoop.exec(QEventLoop::ExcludeUserInputEvents);
            }
        }
    }

ui->ble_lineEdit->clear();

}



void MainWindow::read_gatttool()
{


    QString allText = gatttool->readAll();
    QStringList lines = allText.split('\n');

    foreach(QString line, lines)
    {

        if(line.contains("[CON]"))
        {
            if(!_connected)
                ui->ble_label->setText("Connected");
            _connected = true;
        }
        else if(line.contains("[   ]"))
        {
            if(_connected)
                ui->ble_label->setText("Disconnect");
            _connected = false;
        }
           

       /* if(line.contains("Notification") ||
              line.contains("Indication") ||
               line.contains("Characteristic")||
              line.contains("[CON]")||
              line.contains("[   ]"))
        {
            ble_dataReceived = true;
            ui->ble_log_textEdit->append(line);
        }*/
            ble_dataReceived = true;
            ui->ble_log_textEdit->append(line);




    }

    updateInterface();
}

void MainWindow::updateInterface()
{
    ui->ble_comboBox->setEnabled(!_connected);
    if(!_connected)
        ui->ble_connect_button->setText("Connect");
    else
        ui->ble_connect_button->setText("Disconnect");

}


void MainWindow::on_ble_lineEdit_editingFinished()
{
       write_gatttool();
       ui->ble_lineEdit->clear();
}


void MainWindow::on_takeoff_Button_clicked()
{
    QStringList send_text;
    QString list_text;
    convertedValue = QByteArray::number(cmd_value,16);
    if(cmd_value<16)
        send_text<<"char-write-cmd 0x0043 0x040"<<convertedValue<<"02000100";
    else
      send_text<<"char-write-cmd 0x0043 0x04"<<convertedValue<<"02000100";
    list_text=send_text.join("");
    ui->ble_lineEdit->setText(list_text);
    write_gatttool();
    cmd_value++;
}



void MainWindow::on_land_Button_clicked()
{
    QStringList send_text;
    QString list_text;
    convertedValue = QByteArray::number(cmd_value,16);
    if(cmd_value<16)
        send_text<<"char-write-cmd 0x0043 0x040"<<convertedValue<<"02000300";
    else
      send_text<<"char-write-cmd 0x0043 0x04"<<convertedValue<<"02000300";
    list_text=send_text.join("");
    ui->ble_lineEdit->setText(list_text);
    write_gatttool();
    cmd_value++;
}


void MainWindow::on_pushButton_clicked()
{
    if(configset==true){
        qDebug()<<"connect";
        ui->ble_lineEdit->setText("char-write-cmd 0x00c0 0100");
        write_gatttool();
        ui->ble_lineEdit->setText("char-write-cmd 0x00bd 0100");
        write_gatttool();
        ui->ble_lineEdit->setText("char-write-cmd 0x00e4 0100");
        write_gatttool();
        ui->ble_lineEdit->setText("char-write-cmd 0x00e7 0100");
        write_gatttool();
        ui->ble_lineEdit->setText("char-write-cmd 0x0116 0100");
        write_gatttool();
        ui->ble_lineEdit->setText("char-write-cmd 0x0126 0100");
        write_gatttool();

        ui->ble_lineEdit->setText("char-write-cmd 0x0043 0x00401000401005058595245494845505600");
        write_gatttool();
        ui->ble_lineEdit->setText("char-write-cmd 0x007c 0x010101");
        write_gatttool();
        ui->ble_lineEdit->setText("char-write-cmd 0x0043 0x04020004020084495353515357434849484800");
        write_gatttool();
        ui->ble_lineEdit->setText("char-write-cmd 0x007c 0x010202");
        write_gatttool();
        ui->ble_lineEdit->setText("char-write-cmd 0x0043 0x040300020000");
        write_gatttool();
        for(config_value=3;config_value<15;config_value++){
            QStringList send_text;
            QString list_text;
            QString i = QString::number(config_value);
            if(config_value<10)
                send_text<<"char-write-cmd 0x007c 0x0100"<<i<<"00"<<i;
            else
                send_text<<"char-write-cmd 0x007c 0x010"<<i<<"0"<<i;
            list_text=send_text.join("");
            ui->ble_lineEdit->setText(list_text);
            write_gatttool();
        }
        ui->ble_lineEdit->setText("char-write-cmd 0x0043 0x040400040000");
        write_gatttool();
        ui->ble_lineEdit->clear();
        on_ping_test_Button_clicked();
        configset=false;
    }else{
        QStringList send_text;
        QString list_text;
        QString i = QString::number(config_value);
        send_text<<"char-write-cmd 0x007c 0x010"<<i<<"0"<<i;
        list_text=send_text.join("");
        ui->ble_lineEdit->setText(list_text);
        write_gatttool();
    }
    config_value++;

}
void MainWindow::on_ping_test_Button_clicked()
{

    if(ui->ping_test_Button->text()=="Ping Test"){
    ping_timer->start(100);
    ui->ping_test_Button->setText("Stop ping");
    }else{
        ping_timer->stop();
        ui->ping_test_Button->setText("Ping Test");
    }

}

void MainWindow::send_ping()
{
    on_pushButton_clicked();
    QStringList send_text;
    QString list_text;
    convertedValue = QByteArray::number(ping_value,16);
    if(ping_value<16)
        send_text<<"char-write-cmd 0x0040 0x020"<<convertedValue<<"02000200";
    else
        send_text<<"char-write-cmd 0x0040 0x02"<<convertedValue<<"02000200";
    if(ui->checkBox->checkState()==2)
        send_text<<"010032000000000000";
    else
        send_text<<"000000000000000000";

    list_text=send_text.join("");
    ui->ble_lineEdit->setText(list_text);
    write_gatttool();
     ping_value++;
     if(ping_value>255)
         ping_value=0;

}

/***************************************
    KEY EVENT
***************************************/


void MainWindow::keyPressEvent(QKeyEvent *event){

    if(key_event==true){
        switch(event->key())
        {
            case Qt::Key_Q:
                speed = 0;
                key_write();
            break;
            case Qt::Key_E:
                speed = 9000;
                key_write();
                sleep(1);
                speed = 8000;
                key_write();
            break;
            case Qt::Key_W:
                speed = speed +1000;
                key_write();
            break;
            case Qt::Key_S:
                speed = speed - 1000;
               key_write();
            break;

         }

    }
}
void MainWindow::key_write(){
    if(speed<0)
        speed=0;
    else if(speed>9999)
        speed=9999;
    qst_speed=QString::number(speed);
    ui->lineEdit->setText("3 "+qst_speed);
    write_telnet();
    ui->lineEdit->clear();
}

void MainWindow::on_joystick_clicked()
{
    if(ui->joystick->text()=="JOY ON"){
        pid_line_update(false);
        ui->lineEdit->setEnabled(false);
        ui->ble_lineEdit->setEnabled(false);
        key_event=true;
        ui->joystick->setText("JOY OFF");

    }else{
        pid_line_update(true);
        ui->lineEdit->setEnabled(true);
        ui->ble_lineEdit->setEnabled(true);
        key_event=false;
        ui->joystick->setText("JOY ON");
    }
}

void MainWindow::on_emergency_clicked()
{
    ui->lineEdit->setText("3 0");
    write_telnet();
    ui->lineEdit->clear();
    on_ctrl_C_clicked();
    ui->lineEdit->setText("a");
    write_telnet();
    ui->lineEdit->clear();
    mission_timer->stop();


}

void MainWindow::show_time(){
    qint64 current_time = QTime::currentTime().msecsTo(start_time)*-1;
    int h = current_time / 1000 / 60 / 60;
    int m = (current_time / 1000 / 60) - (h * 60);
    int s = (current_time / 1000) - (m * 60);
    current_time = current_time - (s * 1000);
    QString diff = QString("%1:%2:%3:%4").arg(h).arg(m).arg(s).arg(current_time);
    ui->clock_label->setText(diff);

}

void MainWindow::on_mission_button_clicked()
{
    QApplication::processEvents();
    QString diff = QString("0:0:0:000");
    ui->clock_label->setText(diff);
    start_time = QTime::currentTime();
    mission_timer->start(1);
    pid_set(false);
    on_pid_test_clicked();

    speed = 9000;
    key_write();
    speed = 7500;
    key_check_timer->singleShot(1000,this,SLOT(key_write()));
    key_check_timer->singleShot(2000,this,SLOT(speed_8000()));
    key_check_timer->singleShot(3000,this,SLOT(pid_stop()));
    /*on_sub_start_clicked();
    QTimer::singleShot(2000,this,SLOT(on_pid_test_clicked()));
    QTimer::singleShot(500,this,SLOT(on_pid_test_clicked()));*/
    //ui->mission_button->setText("MISSION STOP");
    //on_mission_button_clicked();



}
void MainWindow::pid_stop(){
    ui->lineEdit->setText("0");
    write_telnet();
    ui->pid_test->setText("start");
    ui->lineEdit->clear();
    pid_line_update(true);
    mission_timer->stop();
    ui->lineEdit->setText("3 0");
    write_telnet();
    pid_set(true);
    //on_pid_test_clicked();
    on_sensor_button_clicked();
    if(ui->sensor_button->text()=="Sensor ON")
    on_sensor_button_clicked();
    //ui->sensor_button->setText();

}
void MainWindow::speed_8000(){
    speed=8000;
    key_write();
}


void MainWindow::pid_set(bool state){
    if(state==true){
        ui->pitch_kp_line->setText("0");
        ui->pitch_ki_line->setText("0");
        ui->pitch_kd_line->setText("0");
        ui->roll_kp_line->setText("0");
        ui->roll_ki_line->setText("0");
        ui->roll_kd_line->setText("0");
        ui->z_kp_line->setText("0");
        ui->z_ki_line->setText("0");
        ui->z_kd_line->setText("0");
        ui->motor_throttle_line->setText("0");
    }else{
        ui->pitch_kp_line->setText("25");
        ui->pitch_ki_line->setText("78");
        ui->pitch_kd_line->setText("0.03");
        ui->roll_kp_line->setText("25");
        ui->roll_ki_line->setText("78");
        ui->roll_kd_line->setText("0.03");
        ui->z_kp_line->setText("0");
        ui->z_ki_line->setText("0");
        ui->z_kd_line->setText("0");
        ui->motor_throttle_line->setText("0");
    }


}

void MainWindow::on_pid_init_set_Button_clicked()
{
    pid_set(false);
}

/********************ADB SOCKET*****************************/
/*
void MainWindow::connect_to_server(){
    adb_socket.connectToHost(QHostAddress::LocalHost, 8010);//"127.0.0.1",8888);
}
void MainWindow::on_Connect_Server(){
    ui->socket_textEdit->setText("Socket Connected\n");

}

void MainWindow::send_Request(){
    QByteArray block;
    QDataStream out(&block, QIODevice::WriteOnly);
    out<<quint16(0);
    out<<ui->socket_lineEdit->text();
    out.device()->seek(0);
    out<<quint16(block.size() - sizeof(quint16));
    adb_socket.write(block);
    ui->socket_textEdit->append("send<< "+block);
}
void MainWindow::read_Message(){
    QDataStream in(&adb_socket);
    while(true){
             //nextBlcokSize 가 0 이면 아직 데이터를 못받은것
            if(nextBlocksize == 0){
                //수신된 데이터가 nextBlockSize 바이트보다 큰지 확인
                if(adb_socket.bytesAvailable() < sizeof(quint16))
                    ;
                else
                    in>>nextBlocksize;
                continue;
            }
            //nextBlcokSize가 도착하면 사이즈만큼 데이터가 도착했는지 확인
           else if(adb_socket.bytesAvailable() < nextBlocksize)
                continue;

            //데이터를 표시
           else if(adb_socket.bytesAvailable() >= nextBlocksize){
                QString strBuf;
                in>>strBuf;

                ui->socket_textEdit->setText(strBuf);
                this->nextBlocksize = 0;

                break;
            }
        }
}

void MainWindow::connection_Closed_By_Server(){
    adb_socket.close();
}

void MainWindow::error(){
    ui->socket_textEdit->setText(adb_socket.errorString());
}

*/
void MainWindow::on_socket_connect_Button_clicked()
{

    socket=new QTcpSocket(this);

    connect(socket,SIGNAL(readyRead()),this,SLOT(socket_readyRead()));
    connect(socket,SIGNAL(connected()),this,SLOT(socket_connected()));
    socket->connectToHost(ui->socket_ip_lineEdit->text(),ui->socket_port_lineEdit->text().toInt());

}


void MainWindow::socket_readyRead()
{
    while(socket->canReadLine())
    {
        qDebug()<<"ready read";
        QString line=QString::fromUtf8(socket->readLine());
        line=line.left(line.length()-1);
        ui->socket_textEdit->append(QString("%2").arg(line));
    }
    //ui->socket_textEdit->scrollToBottom();
}

void MainWindow::socket_connected()
{
    qDebug()<<"socket connected";
    socket->write(QString("me:"+ui->socket_lineEdit->text()+"\n").toUtf8());
}


void MainWindow::on_socket_lineEdit_returnPressed()
{
    on_socket_send_Button_clicked();
}

void MainWindow::on_socket_send_Button_clicked()
{
    QString message=ui->socket_lineEdit->text().trimmed();

    if(!message.isEmpty())
    {
        socket->write(QString(message+"\n").toUtf8());
    }
    ui->socket_lineEdit->clear();
    //ui->socket_lineEdit->setFocus();
}
