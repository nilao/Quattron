#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QProcess>
#include <myglwidget.h>
#include <qthread.h>
#include <QTimer>
#include <QTime>
#include <QTcpSocket>


namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();
    Ui::MainWindow *ui;

    /***************CONNECT&READ/WRITE***************/
    bool _connect;
    bool _dataReceived;
    bool ble_dataReceived;
    /********************OPEN GL*********************/
    MyGLWidget *cube2;
    int cubeshow;
    double x,y,z,degree;
    double xval,yval,zval,init,pre_init;
    /*******************MOTOR************************/
    QString motor0_val,motor1_val,motor2_val,motor3_val;
    /*****************PID TEST***********************/
    QString roll_kp,roll_ki,roll_kd,pitch_kp,pitch_ki,pitch_kd,motor_throttle0,motor_throttle1,motor_throttle2,motor_throttle3;
    QString z_kp,z_ki,z_kd;

    /*****************SENSOR*************************/

    double acc_x,acc_y,acc_z,gyro_x,gyro_y,gyro_z;
    double pre_gyro_z,pre_DT,press_ground,height;
    /****************GATT TOOL***********************/
    //pthread_t ping_thread;
    int ping_value,cmd_value,config_value;
    bool configset;
    //pingthread *pthread;
    //ThreadTest *ping_thread;
    QByteArray convertedValue;
    /*************KEY EVENT**************************/
    int speed;
    QString qst_speed;
    bool key_event;
    /************************************************/
    QTime start_time;
    /**************ADB SOCKET************************/



private slots:


    /*****************CONNECT************************/
        void on_scan_button_clicked();
        void on_pand_button_clicked();
        void on_ip_button_clicked();
        void on_dragon_prog_clicked();
        void on_sub_start_clicked();
        void on_clear_button_clicked();
    /*************READ/WRITE*************************/
        void read_telnet();
        void write_telnet();
        void on_lineEdit_editingFinished();
    /****************OPEN GL*************************/
        void DrawCube(double x,double y, double z);
        void on_ctrl_C_clicked();
        void on_aout_clicked();
    /****************MOTOR & PID TEST****************/
        //void on_moter_speed_button_clicked();
        //void on_moter_stop_clicked();
        void on_pid_test_clicked();
        void pid_line_update(bool state);
        void pid_set(bool state);
    /*****************senser-test********************/
        void on_sensor_button_clicked();
        void sensor_line_clean();
        void on_yaw_Button_clicked();
        void show_time();
    /****************nothing*************************/
        void slotProcessStarted();
        void slotProcessFinished();
    /*****************gatt tool**********************/
        void gatttool_Connect();
        void read_gatttool();
        void on_ble_lineEdit_editingFinished();
        void updateInterface();
        void write_gatttool();
        void on_ping_test_Button_clicked();
        void on_takeoff_Button_clicked();
        void on_land_Button_clicked();
        void on_pushButton_clicked();
        void send_ping();
   /********************KEY EVENT*********************/
        void keyPressEvent(QKeyEvent *key);
        void key_write();
        void on_joystick_clicked();
        void on_emergency_clicked();
        void on_mission_button_clicked();
        void pid_stop();
        void speed_8000();
        void on_pid_init_set_Button_clicked();
   /********************ADB SOCKET********************/
       /*
       void connect_to_server();
       void on_Connect_Server();
       void send_Request();
       void read_Message();
       void connection_Closed_By_Server();
       void error();*/


        void on_socket_connect_Button_clicked();

        void on_socket_lineEdit_returnPressed();

        void socket_connected();
        void socket_readyRead();

        void on_socket_send_Button_clicked();

private:
    bool _connected;

    QProcess *telnet;
    QProcess *gatttool;
    QTimer *ping_timer;
    QTimer *key_check_timer;
    QTimer *mission_timer;
    QTcpSocket* socket;
};

#endif // MAINWINDOW_H
