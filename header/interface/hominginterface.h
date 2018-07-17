#ifndef HOMINGINTERFACE_H
#define HOMINGINTERFACE_H

#include <QWidget>
#include <QCloseEvent>
#include <QTableWidgetItem>
#include <QFile>
#include <QDir>
#include <QVector>
#include <QTextStream>

#include "cablerobotmaster.h" //User declaration

namespace Ui
{
class HomingInterface;
}

class HomingInterface : public QWidget
{
  Q_OBJECT
private:
  constexpr static uint8_t set = 1;
  constexpr static uint8_t reset = 0;
  QFile dataFile;

public:
  explicit HomingInterface(QWidget* parent = 0, CableRobotMaster* theMaster = 0);
  ~HomingInterface();

signals:
  void GoBackIdle(int);
  void SendEnableRequest(int);
  void SendClearFaultRequest();
  void SendHomingProcessControl(int);
  void SendMeasurementRequest();
  void SendHomingData(QVector<double>);

private slots:
  void on_HomingEnableButton_toggled(bool checked);
  void on_HomingClearFaultsButton_clicked();
  void on_HomingAcquireDataButton_clicked();
  void on_HomingStopSaveButton_clicked();
  void on_InternaHomingButton_clicked();
  void on_LoadExtenalHomingButton_clicked();
  void on_HomingStartButton_toggled(bool checked);

public slots:
  void CollectFaultPresentAdvice(int theMotor);
  void CollectEnableCommandProcessed(int status, int theMotor);
  void CollectClearFaultRequestProcessed(int theMotor);
  void CollectHomingControl(int state);
  void CollectMeasurements(QVector<double> measurements);

private:
  uint8_t collectingData = 0;
  CableRobotMaster* cableRobotMaster;
  CableRobot* cableRobot;
  Ui::HomingInterface* ui;

  virtual void closeEvent(QCloseEvent* event);
};

#endif // HOMINGINTERFACE_H
