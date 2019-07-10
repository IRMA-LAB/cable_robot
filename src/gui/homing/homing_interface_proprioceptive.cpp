/**
 * @file homing_interface_proprioceptive.cpp
 * @author Simone Comari
 * @date 09 Jul 2019
 * @brief This file includes definitions of classes present in
 * homing_interface_proprioceptive.h.
 */

#include "gui/homing/homing_interface_proprioceptive.h"
#include "ui_homing_interface_proprioceptive.h"


HomingInterfaceProprioceptive::HomingInterfaceProprioceptive(QWidget* parent,
                                                             CableRobot* robot)
  : HomingInterface(parent, robot), ui(new Ui::HomingInterfaceProprioceptive),
    app_(this, robot), acquisition_complete_(false), ext_close_cmd_(false)
{
  ui->setupUi(this);

  quint8 pos = 5; // insert position in vertical layout
  for (id_t motor_id : robot->GetActiveMotorsID())
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
  connect(&app_, SIGNAL(printToQConsole(QString)), this,
          SLOT(appendText2Browser(QString)), Qt::ConnectionType::DirectConnection);
  connect(&app_, SIGNAL(acquisitionComplete()), this, SLOT(handleAcquisitionComplete()));
  connect(&app_, SIGNAL(homingComplete()), this, SLOT(handleHomingComplete()));
  connect(&app_, SIGNAL(stateChanged(quint8)), this, SLOT(handleStateChanged(quint8)),
          Qt::ConnectionType::QueuedConnection);
}

HomingInterfaceProprioceptive::~HomingInterfaceProprioceptive()
{
  disconnect(robot_ptr_, SIGNAL(printToQConsole(QString)), this,
             SLOT(appendText2Browser(QString)));
  disconnect(&app_, SIGNAL(printToQConsole(QString)), this,
             SLOT(appendText2Browser(QString)));
  disconnect(&app_, SIGNAL(progressValue(int)), this,
             SLOT(updateAcquisitionProgress(int)));
  disconnect(&app_, SIGNAL(acquisitionComplete()), this,
             SLOT(handleAcquisitionComplete()));
  disconnect(&app_, SIGNAL(homingComplete()), this, SLOT(handleHomingComplete()));
  disconnect(&app_, SIGNAL(stateChanged(quint8)), this, SLOT(handleStateChanged(quint8)));

  for (InitTorqueForm* form : init_torque_forms_)
    delete form;
  delete ui;
  CLOG(INFO, "event") << "Homing interface proprioceptive closed";
}

//--------- Private GUI slots -------------------------------------------------------//

void HomingInterfaceProprioceptive::closeEvent(QCloseEvent* event)
{
  if (ext_close_cmd_)
  {
    ext_close_cmd_ = false;
    event->accept();
  }
  else
  {
    event->ignore();
    // This becomes like user hit Cancel button.
    ui->pushButton_cancel->click();
  }
}

void HomingInterfaceProprioceptive::on_pushButton_enable_clicked()
{
  bool robot_enabled = ui->pushButton_enable->text() == "Disable";
  CLOG(TRACE, "event") << (robot_enabled ? "DISABLE" : "ENABLE");
  ui->pushButton_enable->setChecked(false);

  if (!robot_enabled)
  {
    app_.Start(nullptr); // IDLE --> ENABLED
    return;
  }

  if (app_.IsCollectingData())
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
  app_.Disable(); // any --> ENABLED --> IDLE
}

void HomingInterfaceProprioceptive::on_pushButton_clearFaults_clicked()
{
  CLOG(TRACE, "event");
  ui->pushButton_start->setChecked(false);
  app_.FaultReset();
}

void HomingInterfaceProprioceptive::on_checkBox_useCurrentTorque_stateChanged(int)
{
  CLOG(TRACE, "event");
  if (ui->checkBox_useCurrentTorque->isChecked())
    ui->checkBox_initTorque->setChecked(false);
  for (auto* form : init_torque_forms_)
    form->EnableInitTorque(!ui->checkBox_useCurrentTorque->isChecked());
  UpdateTorquesLimits();
}

void HomingInterfaceProprioceptive::on_checkBox_initTorque_stateChanged(int)
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

void HomingInterfaceProprioceptive::on_spinBox_initTorque_valueChanged(int value)
{
  for (auto* init_torque_form : init_torque_forms_)
    init_torque_form->SetInitTorque(value);
}

void HomingInterfaceProprioceptive::on_checkBox_maxTorque_stateChanged(int)
{
  CLOG(TRACE, "event");
  for (auto* init_torque_form : init_torque_forms_)
    init_torque_form->EnableMaxTorque(!ui->checkBox_maxTorque->isChecked());
  ui->spinBox_maxTorque->setEnabled(ui->checkBox_maxTorque->isChecked());
  UpdateTorquesLimits();
}

void HomingInterfaceProprioceptive::on_spinBox_maxTorque_valueChanged(int value)
{
  for (auto* init_torque_form : init_torque_forms_)
    init_torque_form->SetMaxTorque(value);
}

void HomingInterfaceProprioceptive::on_pushButton_start_clicked()
{
  ui->pushButton_start->setChecked(false);
  if (app_.IsCollectingData())
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
      app_.Stop(); // any --> ENABLED
      disconnect(&app_, SIGNAL(progressValue(int)), this,
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
  connect(&app_, SIGNAL(progressValue(int)), this, SLOT(updateAcquisitionProgress(int)),
          Qt::ConnectionType::DirectConnection);
  app_.Start(data);
}

void HomingInterfaceProprioceptive::on_radioButton_internal_clicked()
{
  CLOG(TRACE, "event");
  ui->radioButton_external->toggled(false);
  ui->lineEdit_extFile->setDisabled(true);
  ui->pushButton_extFile->setDisabled(true);
  ui->pushButton_ok->setEnabled(true);
}

void HomingInterfaceProprioceptive::on_radioButton_external_clicked()
{
  CLOG(TRACE, "event");
  ui->radioButton_internal->toggled(false);
  ui->lineEdit_extFile->setEnabled(true);
  ui->pushButton_extFile->setEnabled(true);
  ui->pushButton_ok->setEnabled(acquisition_complete_ &&
                                !ui->lineEdit_extFile->text().isEmpty());
}

void HomingInterfaceProprioceptive::on_pushButton_extFile_clicked()
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

void HomingInterfaceProprioceptive::on_pushButton_ok_clicked()
{
  CLOG(TRACE, "event");
  ui->pushButton_ok->setChecked(false);
  ui->progressBar_optimization->setValue(0);
  connect(&app_, SIGNAL(progressValue(int)), this, SLOT(updateOptimizationProgress(int)),
          Qt::ConnectionType::DirectConnection);
  // "Internal" optimization (ext call to Matlab)
  if (ui->radioButton_internal->isChecked())
  {
    if (QFile::exists(app_.kMatlabOptimizationResultsLoc))
      if (QFile::remove(app_.kMatlabOptimizationResultsLoc))
        CLOG(INFO, "event") << "Removed old matlab homing optimization results";
    ui->groupBox_dataCollection->setEnabled(false);
    app_.Optimize();
    return;
  }
  // "External" optimization = load results obtained somehow externally
  HomingProprioceptiveHomeData* home_data = new HomingProprioceptiveHomeData;
  if (!app_.ParseExtFile(ui->lineEdit_extFile->text(), home_data))
  {
    QMessageBox::warning(this, "File Error",
                         "File content is not valid!\nPlease load a different file.");
    return;
  }
  ui->progressBar_optimization->setValue(100);
  ui->groupBox_dataCollection->setEnabled(false);
  app_.GoHome(home_data);
}

void HomingInterfaceProprioceptive::on_pushButton_cancel_clicked()
{
  CLOG(TRACE, "event");
  switch (static_cast<HomingProprioceptiveApp::States>(app_.GetCurrentState()))
  {
    case HomingProprioceptiveApp::ST_IDLE:
      break;
    case HomingProprioceptiveApp::ST_OPTIMIZING:
    {
      QMessageBox::StandardButton reply =
        QMessageBox::question(this, "Optimization in progress",
                              "The application is still evaluating data to complete the "
                              "homing procedure. If you quit now all progress will be "
                              "lost.\nAre you sure you want to abort the operation?",
                              QMessageBox::Yes | QMessageBox::No);
      if (reply == QMessageBox::No)
        return;
      CLOG(INFO, "event") << "Homing interrupted by user during optimizazion";
      break;
    }
    case HomingProprioceptiveApp::ST_HOME:
    {
      QMessageBox::StandardButton reply =
        QMessageBox::question(this, "Homing in progress",
                              "The robot is moving to the homing position. If you "
                              "quit now all progress will be lost.\nAre you sure "
                              "you want to abort the operation?",
                              QMessageBox::Yes | QMessageBox::No);
      if (reply == QMessageBox::No)
        return;
      CLOG(INFO, "event") << "Homing interrupted by user while moving to home position";
      break;
    }
    case HomingProprioceptiveApp::ST_FAULT:
    {
      QMessageBox::information(this, "Fault present",
                               "Please clear faults before quitting the application.");
      return;
    }
    default:
    {
      QMessageBox::StandardButton reply = QMessageBox::question(
        this, "Acquisition in progress",
        "The application is still acquiring data from the robot. If you quit now all "
        "progress will be lost.\nAre you sure you want to abort the operation?",
        QMessageBox::Yes | QMessageBox::No);
      if (reply == QMessageBox::No)
        return;
      CLOG(INFO, "event") << "Homing interrupted by user during data acquisition";
    }
  }
  app_.Disable(); // any --> IDLE
  emit homingFailed();
  hide();
  CLOG(INFO, "event") << "Hide homing interface proprioceptive";
}

void HomingInterfaceProprioceptive::on_pushButton_done_clicked()
{
  CLOG(TRACE, "event");
  emit homingSuccess();
  hide();
}

//--------- Private slots -----------------------------------------------------------//

void HomingInterfaceProprioceptive::appendText2Browser(const QString& text)
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

void HomingInterfaceProprioceptive::updateAcquisitionProgress(const int value)
{
  if (static_cast<int>(value) > ui->progressBar_acquisition->value())
    ui->progressBar_acquisition->setValue(static_cast<int>(value));
}

void HomingInterfaceProprioceptive::updateOptimizationProgress(const int value)
{
  if (value > ui->progressBar_optimization->value())
    ui->progressBar_optimization->setValue(value);
}

void HomingInterfaceProprioceptive::handleAcquisitionComplete()
{
  CLOG(INFO, "event") << "Acquisition complete";
  ui->radioButton_internal->setEnabled(true);
  ui->pushButton_ok->setEnabled(true);
  acquisition_complete_ = true;

  disconnect(&app_, SIGNAL(progressValue(int)), this,
             SLOT(updateAcquisitionProgress(int)));
}

void HomingInterfaceProprioceptive::handleHomingComplete()
{
  CLOG(INFO, "event") << "Homing complete";
  ui->groupBox_dataCollection->setEnabled(true);
  ui->pushButton_done->setEnabled(true);
  emit homingCompleted();
}

void HomingInterfaceProprioceptive::handleStateChanged(const quint8& state)
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
      app_.Start(nullptr);
      break;
    case HomingProprioceptiveApp::ST_SWITCH_CABLE:
      app_.Start(nullptr);
      break;
    case HomingProprioceptiveApp::ST_COILING:
      app_.Start(nullptr);
      break;
    case HomingProprioceptiveApp::ST_UNCOILING:
      app_.Start(nullptr);
      break;
    case HomingProprioceptiveApp::ST_FAULT:
      ui->pushButton_enable->setDisabled(true);
      ui->pushButton_enable->setText(tr("Enable"));
      ui->pushButton_start->setDisabled(true);
      ui->pushButton_clearFaults->setEnabled(true);
      break;
    case HomingProprioceptiveApp::ST_HOME:
      disconnect(&app_, SIGNAL(progressValue(int)), this,
                 SLOT(updateOptimizationProgress(int)));
      break;
    default:
      break;
  }
}

//--------- Private functions ------------------------------------------------------//

void HomingInterfaceProprioceptive::UpdateTorquesLimits()
{
  if (app_.GetCurrentState() == HomingProprioceptiveApp::ST_ENABLED &&
      ui->checkBox_useCurrentTorque->isChecked())
  {
    std::vector<id_t> motors_id = app_.GetActuatorsID();
    for (size_t i = 0; i < motors_id.size(); i++)
      init_torque_forms_[static_cast<int>(i)]->SetInitTorque(
        app_.GetActuatorStatus(motors_id[i]).motor_torque);
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

void HomingInterfaceProprioceptive::Close()
{
  ext_close_cmd_ = true;
  close();
}
