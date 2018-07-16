#ifndef ACTUATORPVTINTERFACE33_H
#define ACTUATORPVTINTERFACE33_H

#include <QWidget>
#include <QCloseEvent>
#include <QTableWidgetItem>
#include <QFile>
#include <QDir>
#include <QVector>
#include <QTextStream>

#include "cablerobotmaster.h" //User declaration

namespace Ui {
class ActuatorPvtInterface33;
}

class ActuatorPvtInterface33 : public QWidget
{
    Q_OBJECT
private:
    constexpr static uint8_t set = 1;
    constexpr static uint8_t reset = 0;
    QFile dataFile;
public:
    explicit ActuatorPvtInterface33(QWidget *parent = 0, CableRobotMaster *theMaster = 0);
    ~ActuatorPvtInterface33();

signals:
    void GoBackIdle(int);
    void SendEnableRequest(int);
    void SendClearFaultRequest();
    void SendStartRequest();
    void SendExportDataRequest();
    void SendSimulationData(int, double*, double*, double*);

private slots:
    void on_ImportDataButton_clicked();
    void on_StartButton_toggled(bool checked);
    void on_ExportDataButton_clicked();
    void on_EnableButton_toggled(bool checked);
    void on_ClearFaultsButton_clicked();

    void on_StartButton_clicked();

public slots:
    void CollectFaultPresentAdvice(int theMotor);
    void CollectEnableCommandProcessed(int status, int theMotor);
    void CollectClearFaultRequestProcessed(int theMotor);
    void CollectActuatorPvt33Control(int state);
    void CollectData(double s0, double s1, double s2);

private:
    CableRobotMaster *cableRobotMaster;
    QVector<double> pulleyAnglesOut[3];
    QVector<double> cableLengthIn[3];
    CableRobot *cableRobot;
    Ui::ActuatorPvtInterface33 *ui;

    virtual void closeEvent(QCloseEvent *event);
};

#endif // ACTUATORPVTINTERFACE33_H
