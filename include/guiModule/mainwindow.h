#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QMessageBox>

#include <yarp/os/Network.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Port.h>
#include <yarp/os/all.h>

#include <tinyxml.h>


//
// #include <yarp/os/RateThread.h>
// #include <yarp/os/Time.h>
// #include <yarp/sig/Vector.h>
// #include <yarp/sig/all.h>
// #include <yarp/math/Math.h>

// YARP_DECLARE_DEVICES(icubmod)

// using namespace yarp::os;

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

    void initializeGui();

private slots:
    void createPlot();
    void on_gainTestButton_clicked();
    void addPartsToList();
    void on_partList_currentIndexChanged(int partId);
    void on_jointList_currentIndexChanged(int jointId);
    void on_closeButton_clicked();
    void on_homeButton_clicked();
    void on_nextJointButton_clicked();
    void on_previousJointButton_clicked();
    void on_posContButton_clicked(bool checked);
    void on_velContButton_clicked(bool checked);
    void on_torContButton_clicked(bool checked);
    void on_kd_in_editingFinished();
    void on_kp_in_editingFinished();
    void on_ki_in_editingFinished();
    void on_saveGainsButton_clicked();
    void on_gainResetButton_clicked();
    // void on_partList_highlighted(int index);
    // void on_jointList_highlighted(int index);

private:
    Ui::MainWindow *ui;





    //Functions
    void resetYLabel();
    void setCurrentPartAndJoint();
    bool discardChanges();
    void saveGains();
    void refreshGainDisplays();
    bool getPidGains();
    bool setPidGains();
    void sendPartAndJointIndexes();
    void sendControlMode();
    void writeDataToLogs();
    void createDataLogs();
    const std::string currentDateTime();

    //Variables
    QString yPlotLabel;
    bool isOnlyMajorJoints, gainsHaveBeenChanged;
    int controlMode, partIndex, jointIndex;

    bool initFinished;


    double Kp_new, Kd_new, Ki_new;
    double Kp_old, Kd_old, Ki_old;

    yarp::os::Network yarp;

    yarp::os::BufferedPort<yarp::os::Bottle> gainsBufPort_out;
    yarp::os::Port                           gainsPort_in;


    yarp::os::BufferedPort<yarp::os::Bottle> goToHomeBufPort_out;

    yarp::os::BufferedPort<yarp::os::Bottle> robotPartAndJointBufPort_out;

    yarp::os::BufferedPort<yarp::os::Bottle> controlModeBufPort_out;


    yarp::os::Port                           dataPort_in;



    std::string controlMode_string;
    std::string logFilePath;

};

#endif // MAINWINDOW_H
