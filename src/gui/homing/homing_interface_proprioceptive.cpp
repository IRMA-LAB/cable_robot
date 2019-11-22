/**
 * @file homing_interface_proprioceptive.cpp
 * @author Simone Comari
 * @date 18 Jul 2019
 * @brief This file includes definitions of classes present in
 * homing_interface_proprioceptive.h.
 */

#include "gui/homing/homing_interface_proprioceptive.h"
#include "ui_homing_interface.h"
#include "ui_homing_interface_proprioceptive.h"

//------------------------------------------------------------------------------------//
//--------- HomingInterfaceProprioceptiveWidget class --------------------------------//
//------------------------------------------------------------------------------------//

HomingInterfaceProprioceptiveWidget::HomingInterfaceProprioceptiveWidget(
  QWidget* parent, CableRobot* robot)
  : QWidget(parent), app(this, robot), ui(new Ui::HomingInterfaceProprioceptiveWidget),
    robot_ptr_(robot), acquisition_complete_(false), ext_close_cmd_(false)
{
  ui->setupUi(this);

  quint8 pos = 5; // insert position in vertical layout
  for (id_t motor_id : robot->getActiveMotorsID())
  {
    init_torque_forms_.append(new InitTorqueForm(motor_id, this));
#if HOMING_ACK
    init_torque_forms_.last()->EnableMaxTorque(false);
#endif
    ui->verticalLayout_2->insertWidget(pos++, init_torque_forms_.last());
  }

#if HOMING_ACK
  ui->checkBox_maxTorque->setDisabled(true);
#endif

  connect(robot_ptr_, SIGNAL(printToQConsole(QString)), this,
          SLOT(appendText2Browser(QString)), Qt::ConnectionType::QueuedConnection);
  connect(&app, SIGNAL(printToQConsole(QString)), this, SLOT(appendText2Browser(QString)),
          Qt::ConnectionType::DirectConnection);
  connect(&app, SIGNAL(acquisitionComplete()), this, SLOT(handleAcquisitionComplete()));
  connect(&app, SIGNAL(stateChanged(quint8)), this, SLOT(handleStateChanged(quint8)),
          Qt::ConnectionType::QueuedConnection);
  connect(&app, SIGNAL(homingComplete()), this, SLOT(handleHomingComplete()));
}

HomingInterfaceProprioceptiveWidget::~HomingInterfaceProprioceptiveWidget()
{
  disconnect(robot_ptr_, SIGNAL(printToQConsole(QString)), this,
             SLOT(appendText2Browser(QString)));
  disconnect(&app, SIGNAL(printToQConsole(QString)), this,
             SLOT(appendText2Browser(QString)));
  disconnect(&app, SIGNAL(progressValue(int)), this,
             SLOT(updateAcquisitionProgress(int)));
  disconnect(&app, SIGNAL(acquisitionComplete()), this,
             SLOT(handleAcquisitionComplete()));
  disconnect(&app, SIGNAL(homingComplete()), this, SLOT(handleHomingComplete()));
  disconnect(&app, SIGNAL(stateChanged(quint8)), this, SLOT(handleStateChanged(quint8)));

  for (InitTorqueForm* form : init_torque_forms_)
    delete form;
  delete ui;
  CLOG(INFO, "event") << "Homing interface proprioceptive closed";
}

//--------- Private GUI slots -------------------------------------------------------//

void HomingInterfaceProprioceptiveWidget::on_pushButton_enable_clicked()
{
  bool robot_enabled = ui->pushButton_enable->text() == "Disable";
  CLOG(TRACE, "event") << (robot_enabled ? "DISABLE" : "ENABLE");
  ui->pushButton_enable->setChecked(false);

  if (!robot_enabled)
  {
    app.Start(nullptr); // IDLE --> ENABLED
    return;
  }

  if (app.IsCollectingData())
  {
    QMessageBox::StandardButton reply =
      QMessageBox::question(this, "Acquisition in progress",
                            "If you stop now all data collected so far will be lost and "
                            "you will have to start over the procedure before starting a "
                            "new application.\nAre you sure you want to continue?",
                            QMessageBox::Yes | QMessageBox::No);
    CLOG(INFO, "event") << "STOP? --> " << (reply == QMessageBox::Yes);
    if (reply == QMessageBox::No)
      return;
  }
  else if (acquisition_complete_)
  {
    QMessageBox::StandardButton reply =
      QMessageBox::question(this, "Homing incomplete",
                            "If you disable the motors all data collected so far "
                            "will be lost and you will have to start over the "
                            "procedure before starting a new application.\nAre you "
                            "sure you want to continue?",
                            QMessageBox::Yes | QMessageBox::No);
    CLOG(INFO, "event") << "DISABLE? --> " << (reply == QMessageBox::Yes);
    if (reply == QMessageBox::No)
      return;
  }
  app.Disable(); // any --> ENABLED --> IDLE
}

void HomingInterfaceProprioceptiveWidget::on_pushButton_clearFaults_clicked()
{
  CLOG(TRACE, "event");
  ui->pushButton_start->setChecked(false);
  app.FaultReset();
}

void HomingInterfaceProprioceptiveWidget::on_checkBox_useCurrentTorque_stateChanged(int)
{
  CLOG(TRACE, "event");
  if (ui->checkBox_useCurrentTorque->isChecked())
    ui->checkBox_initTorque->setChecked(false);
  for (auto* form : init_torque_forms_)
    form->EnableInitTorque(!ui->checkBox_useCurrentTorque->isChecked());
  UpdateTorquesLimits();
}

void HomingInterfaceProprioceptiveWidget::on_checkBox_initTorque_stateChanged(int)
{
  CLOG(TRACE, "event");
  if (ui->checkBox_initTorque->isChecked())
  {
    ui->checkBox_useCurrentTorque->setChecked(false);
    for (auto* init_torque_form : init_torque_forms_)
      init_torque_form->SetInitTorque(ui->spinBox_initTorque->value());
  }
  for (auto* init_torque_form : init_torque_forms_)
    init_torque_form->EnableInitTorque(!ui->checkBox_initTorque->isChecked());
  ui->spinBox_initTorque->setEnabled(ui->checkBox_initTorque->isChecked());
  UpdateTorquesLimits();
}

void HomingInterfaceProprioceptiveWidget::on_spinBox_initTorque_valueChanged(int value)
{
  for (auto* init_torque_form : init_torque_forms_)
    init_torque_form->SetInitTorque(value);
}

void HomingInterfaceProprioceptiveWidget::on_checkBox_maxTorque_stateChanged(int)
{
  CLOG(TRACE, "event");
  for (auto* init_torque_form : init_torque_forms_)
    init_torque_form->EnableMaxTorque(!ui->checkBox_maxTorque->isChecked());
  ui->spinBox_maxTorque->setEnabled(ui->checkBox_maxTorque->isChecked());
  UpdateTorquesLimits();
}

void HomingInterfaceProprioceptiveWidget::on_spinBox_maxTorque_valueChanged(int value)
{
  for (auto* init_torque_form : init_torque_forms_)
    init_torque_form->SetMaxTorque(value);
}

void HomingInterfaceProprioceptiveWidget::on_pushButton_start_clicked()
{
  ui->pushButton_start->setChecked(false);
  if (app.IsCollectingData())
  {
    QMessageBox::StandardButton reply =
      QMessageBox::question(this, "Homing in progress",
                            "If you stop now all data collected so far will be lost and "
                            "you will have to start over the procedure before starting a "
                            "new application.\nAre you sure you want to continue?",
                            QMessageBox::Yes | QMessageBox::No);
    CLOG(TRACE, "event") << "(STOP?) --> " << (reply == QMessageBox::Yes);
    if (reply == QMessageBox::Yes)
    {
      app.Stop(); // any --> ENABLED
      disconnect(&app, SIGNAL(progressValue(int)), this,
                 SLOT(updateAcquisitionProgress(int)));
    }
    return;
  }

  CLOG(TRACE, "event") << "(START)";
  HomingProprioceptiveStartData* data = new HomingProprioceptiveStartData();
  data->num_meas                      = static_cast<quint8>(ui->spinBox_numMeas->value());
  for (InitTorqueForm* form : init_torque_forms_)
  {
    data->init_torques.push_back(form->GetInitTorque());
    data->max_torques.push_back(ui->checkBox_maxTorque->isChecked()
                                  ? static_cast<qint16>(ui->spinBox_maxTorque->value())
                                  : form->GetMaxTorque());
  }
  ui->progressBar_acquisition->setValue(0);
  ui->radioButton_internal->setDisabled(true);
  ui->lineEdit_extFile->clear();
  ui->radioButton_external->toggled(true);
  ui->pushButton_ok->setDisabled(true);
  acquisition_complete_ = false;
  connect(&app, SIGNAL(progressValue(int)), this, SLOT(updateAcquisitionProgress(int)),
          Qt::ConnectionType::DirectConnection);
  app.Start(data);
}

void HomingInterfaceProprioceptiveWidget::on_radioButton_internal_clicked()
{
  CLOG(TRACE, "event");
  ui->radioButton_external->toggled(false);
  ui->lineEdit_extFile->setDisabled(true);
  ui->pushButton_extFile->setDisabled(true);
  ui->pushButton_ok->setEnabled(true);
}

void HomingInterfaceProprioceptiveWidget::on_radioButton_external_clicked()
{
  CLOG(TRACE, "event");
  ui->radioButton_internal->toggled(false);
  ui->lineEdit_extFile->setEnabled(true);
  ui->pushButton_extFile->setEnabled(true);
  ui->pushButton_ok->setEnabled(acquisition_complete_ &&
                                !ui->lineEdit_extFile->text().isEmpty());
}

void HomingInterfaceProprioceptiveWidget::on_pushButton_extFile_clicked()
{
  CLOG(TRACE, "event");
  QString config_filename =
    QFileDialog::getOpenFileName(this, tr("Load Optimization Results"), tr("../.."),
                                 tr("Optimization results (*.json)"));
  if (config_filename.isEmpty())
  {
    QMessageBox::warning(this, "File Error", "File name is empty!");
    return;
  }
  ui->lineEdit_extFile->setText(config_filename);
}

void HomingInterfaceProprioceptiveWidget::on_pushButton_ok_clicked()
{
  CLOG(TRACE, "event");
  ui->pushButton_ok->setChecked(false);
  ui->progressBar_optimization->setValue(0);
  connect(&app, SIGNAL(progressValue(int)), this, SLOT(updateOptimizationProgress(int)),
          Qt::ConnectionType::DirectConnection);
  // "Internal" optimization (ext call to Matlab)
  if (ui->radioButton_internal->isChecked())
  {
    if (QFile::exists(app.kMatlabOptimizationResultsLoc))
      if (QFile::remove(app.kMatlabOptimizationResultsLoc))
        CLOG(INFO, "event") << "Removed old matlab homing optimization results";
    ui->groupBox_dataCollection->setEnabled(false);
    app.Optimize();
    return;
  }
  // "External" optimization = load results obtained somehow externally
  HomingProprioceptiveHomeData* home_data = new HomingProprioceptiveHomeData;
  if (!app.ParseExtFile(ui->lineEdit_extFile->text(), home_data))
  {
    QMessageBox::warning(this, "File Error",
                         "File content is not valid!\nPlease load a different file.");
    return;
  }
  ui->progressBar_optimization->setValue(100);
  ui->groupBox_dataCollection->setEnabled(false);
  app.GoHome(home_data);
}

//--------- Private slots -----------------------------------------------------------//

void HomingInterfaceProprioceptiveWidget::appendText2Browser(const QString& text)
{
  if (text.contains("warning", Qt::CaseSensitivity::CaseInsensitive))
  {
    CLOG(WARNING, "browser") << text;
    ui->textBrowser_logs->append(
      QString("<span style='color: orange'>%1</span>").arg(text));
  }
  else if (text.contains("error", Qt::CaseSensitivity::CaseInsensitive))
  {
    CLOG(ERROR, "browser") << text;
    ui->textBrowser_logs->append(QString("<span style='color: red'>%1</span>").arg(text));
  }
  else
  {
    CLOG(INFO, "browser") << text;
    ui->textBrowser_logs->append(text);
  }
}

void HomingInterfaceProprioceptiveWidget::updateAcquisitionProgress(const int value)
{
  if (static_cast<int>(value) > ui->progressBar_acquisition->value())
    ui->progressBar_acquisition->setValue(static_cast<int>(value));
}

void HomingInterfaceProprioceptiveWidget::updateOptimizationProgress(const int value)
{
  if (value > ui->progressBar_optimization->value())
    ui->progressBar_optimization->setValue(value);
}

void HomingInterfaceProprioceptiveWidget::handleAcquisitionComplete()
{
  CLOG(INFO, "event") << "Acquisition complete";
  ui->radioButton_internal->setEnabled(true);
  ui->pushButton_ok->setEnabled(true);
  acquisition_complete_ = true;

  disconnect(&app, SIGNAL(progressValue(int)), this,
             SLOT(updateAcquisitionProgress(int)));
}

void HomingInterfaceProprioceptiveWidget::handleStateChanged(const quint8& state)
{
  switch (state)
  {
    case HomingProprioceptiveApp::ST_IDLE:
      ui->pushButton_enable->setText(tr("Enable"));
      ui->pushButton_start->setDisabled(true);
      ui->pushButton_start->setText(tr("Start"));
      ui->pushButton_clearFaults->setDisabled(true);
      break;
    case HomingProprioceptiveApp::ST_ENABLED:
    {
      ui->pushButton_enable->setText(tr("Disable"));
      ui->pushButton_start->setEnabled(true);
      ui->pushButton_start->setText(tr("Start"));
      ui->pushButton_clearFaults->setDisabled(true);
      // Update initial torques now because unless enabled values are unknown
      UpdateTorquesLimits();
      break;
    }
    case HomingProprioceptiveApp::ST_START_UP:
      ui->pushButton_start->setText(tr("Stop"));
      app.Start(nullptr);
      break;
    case HomingProprioceptiveApp::ST_SWITCH_CABLE:
      app.Start(nullptr);
      break;
    case HomingProprioceptiveApp::ST_COILING:
      app.Start(nullptr);
      break;
    case HomingProprioceptiveApp::ST_UNCOILING:
      app.Start(nullptr);
      break;
    case HomingProprioceptiveApp::ST_FAULT:
      ui->pushButton_enable->setDisabled(true);
      ui->pushButton_enable->setText(tr("Enable"));
      ui->pushButton_start->setDisabled(true);
      ui->pushButton_clearFaults->setEnabled(true);
      break;
    case HomingProprioceptiveApp::ST_HOME:
      disconnect(&app, SIGNAL(progressValue(int)), this,
                 SLOT(updateOptimizationProgress(int)));
      break;
    default:
      break;
  }
}

void HomingInterfaceProprioceptiveWidget::handleHomingComplete()
{
  CLOG(INFO, "event") << "Homing complete";
  ui->groupBox_dataCollection->setEnabled(true);
}

//--------- Private functions -------------------------------------------------------//

void HomingInterfaceProprioceptiveWidget::UpdateTorquesLimits()
{
  if (app.GetCurrentState() == HomingProprioceptiveApp::ST_ENABLED &&
      ui->checkBox_useCurrentTorque->isChecked())
  {
    std::vector<id_t> motors_id = app.GetActuatorsID();
    for (size_t i = 0; i < motors_id.size(); i++)
      init_torque_forms_[static_cast<int>(i)]->SetInitTorque(
        app.GetActuatorStatus(motors_id[i]).motor_torque);
  }
  if (ui->checkBox_maxTorque->isChecked())
  {
    qint16 max_torque_minimum = static_cast<qint16>(ui->spinBox_maxTorque->maximum());
    for (auto& init_torque_form : init_torque_forms_)
    {
      max_torque_minimum =
        std::min(max_torque_minimum, init_torque_form->GetMaxTorqueMinumum());
      init_torque_form->SetMaxTorque(ui->spinBox_maxTorque->value());
    }
    ui->spinBox_maxTorque->setMaximum(max_torque_minimum);
  }
}

//------------------------------------------------------------------------------------//
//--------- HomingInterfaceProprioceptive class --------------------------------------//
//------------------------------------------------------------------------------------//

HomingInterfaceProprioceptive::HomingInterfaceProprioceptive(QWidget* parent,
                                                             CableRobot* robot)
  : HomingInterface(parent, robot), widget_(this, robot)
{
  connect(&(widget_.app), SIGNAL(homingComplete()), this, SLOT(enableOkButton()));
  ui->verticalLayout->insertWidget(0, &widget_);
}

HomingInterfaceProprioceptive::~HomingInterfaceProprioceptive()
{
  disconnect(&(widget_.app), SIGNAL(homingComplete()), this, SLOT(enableOkButton()));
}

//--------- Private functions -------------------------------------------------------//

bool HomingInterfaceProprioceptive::rejectedExitRoutine(const bool force_exit /*= false*/)
{
  switch (static_cast<HomingProprioceptiveApp::States>(widget_.app.GetCurrentState()))
  {
    case HomingProprioceptiveApp::ST_IDLE:
      break;
    case HomingProprioceptiveApp::ST_OPTIMIZING:
    {
      if (!force_exit)
      {
        QMessageBox::StandardButton reply = QMessageBox::question(
          this, "Optimization in progress",
          "The application is still evaluating data to complete the "
          "homing procedure. If you quit now all progress will be "
          "lost.\nAre you sure you want to abort the operation?",
          QMessageBox::Yes | QMessageBox::No);
        if (reply == QMessageBox::No)
          return false;
      }
      CLOG(INFO, "event") << "Homing interrupted by user during optimizazion";
      break;
    }
    case HomingProprioceptiveApp::ST_HOME:
    {
      if (!force_exit)
      {
        QMessageBox::StandardButton reply =
          QMessageBox::question(this, "Homing in progress",
                                "The robot is moving to the homing position. If you "
                                "quit now all progress will be lost.\nAre you sure "
                                "you want to abort the operation?",
                                QMessageBox::Yes | QMessageBox::No);
        if (reply == QMessageBox::No)
          return false;
      }
      CLOG(INFO, "event") << "Homing interrupted by user while moving to home position";
      break;
    }
    case HomingProprioceptiveApp::ST_FAULT:
    {
      if (force_exit)
        break;
      QMessageBox::information(this, "Fault present",
                               "Please clear faults before quitting the application.");
      return false;
    }
    default:
    {
      if (!force_exit)
      {
        QMessageBox::StandardButton reply = QMessageBox::question(
          this, "Acquisition in progress",
          "The application is still acquiring data from the robot. If you quit now all "
          "progress will be lost.\nAre you sure you want to abort the operation?",
          QMessageBox::Yes | QMessageBox::No);
        if (reply == QMessageBox::No)
          return false;
      }
      CLOG(INFO, "event") << "Homing interrupted by user during data acquisition";
    }
  }
  widget_.app.Disable(); // any --> IDLE
  return true;
}
