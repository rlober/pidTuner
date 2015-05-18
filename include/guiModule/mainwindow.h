#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>


namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

public slots:
    void createPlot();
    void on_gainTestButton_clicked();
    void addPartsToList();
    void on_partList_currentIndexChanged(int index);
    void on_closeButton_clicked();

private slots:
    void on_posContButton_clicked();

    void on_velContButton_clicked();

    void on_torContButton_clicked();

    void on_homeButton_clicked();

private:
    Ui::MainWindow *ui;
    //Foo *fooObject;
    QString controlType, yPlotLabel;
    bool isOnlyMajorJoints;
    void resetYLabel();
};

#endif // MAINWINDOW_H
