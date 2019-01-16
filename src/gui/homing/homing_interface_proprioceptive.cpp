#include "gui/homing/homing_interface_proprioceptive.h"
#include "ui_homing_interface_proprioceptive.h"

HomingInterfaceProprioceptive::HomingInterfaceProprioceptive(QWidget* parent,
                                                             CableRobot* robot)
  : HomingInterface(parent, robot), ui(new Ui::HomingInterfaceProprioceptive),
    app_(this, robot)
{
  ui->setupUi(this);

  quint8 i = 4;
  for (quint8 motor_id = 0; motor_id < 8; motor_id++) // debug
  //  for (quint8 motor_id : robot->GetMotorsID())
  {
    init_torque_forms_.append(new InitTorqueForm(motor_id, this));
    ui->verticalLayout_2->insertWidget(i++, init_torque_forms_.last());
  }

  connect(robot_ptr_, SIGNAL(printToQConsole(QString)), this,
          SLOT(appendText2Browser(QString)));
  connect(&app_, SIGNAL(printToQConsole(QString)), this,
          SLOT(appendText2Browser(QString)));
  connect(&app_, SIGNAL(acquisitionComplete()), this, SLOT(handleAcquisitionComplete()));
  connect(&app_, SIGNAL(homingComplete()), this, SLOT(handleHomingComplete()));
  connect(&app_, SIGNAL(stateChanged(quint8)), this, SLOT(handleStateChanged(quint8)));

  app_.Stop(); // make sure we start in IDLE mode
}

HomingInterfaceProprioceptive::~HomingInterfaceProprioceptive()
{
  disconnect(robot_ptr_, SIGNAL(printToQConsole(QString)), this,
             SLOT(appendText2Browser(QString)));
  disconnect(&app_, SIGNAL(printToQConsole(QString)), this,
             SLOT(appendText2Browser(QString)));
  disconnect(&app_, SIGNAL(acquisitionComplete()), this,
             SLOT(handleAcquisitionComplete()));
  disconnect(&app_, SIGNAL(homingComplete()), this, SLOT(handleHomingComplete()));
  disconnect(&app_, SIGNAL(stateChanged(quint8)), this, SLOT(handleStateChanged(quint8)));

  for (InitTorqueForm* form : init_torque_forms_)
    delete form;
  delete ui;
  CLOG(INFO, "event") << "Homing interface proprioceptive closed";
}

//--------- Public slots --------------------------------------------------------//

void HomingInterfaceProprioceptive::setFault(const bool value)
{
  ui->pushButton_clearFaults->setEnabled(value);
  ui->pushButton_enable->setEnabled(!value);
  ui->pushButton_start->setEnabled(!value);

  if (app_.IsCollectingData())
    ui->pushButton_enable->setText(tr("Enable"));

  if (value)
  {
    CLOG(WARNING, "event") << "Fault present";
    app_.FaultTrigger();
  }
  else
    CLOG(INFO, "event") << "Fault cleared";
}

//--------- Private GUI slots ------------------------------------------------//

void HomingInterfaceProprioceptive::closeEvent(QCloseEvent*)
{
  ui->pushButton_cancel->click();
}

void HomingInterfaceProprioceptive::on_pushButton_enable_clicked()
{
  bool robot_enabled = ui->pushButton_enable->text() == "Disable";
  CLOG(TRACE, "event") << (robot_enabled ? "ENABLE" : "DISABLE");

  if (robot_enabled)
  {
    app_.Start(NULL); // IDLE --> ENABLED
    return;
  }

  if (app_.IsCollectingData())
  {
    QMessageBox::StandardButton reply =
      QMessageBox::question(this, "Homing in progress",
                            "If you stop now all data collected so far will be lost and "
                            "you will have to start over the procedure before starting a "
                            "new application.\nAre you sure you want to continue?",
                            QMessageBox::Yes | QMessageBox::No);
    CLOG(INFO, "event") << "STOP? --> " << (reply == QMessageBox::Yes);
    if (reply == QMessageBox::Yes)
      app_.Stop(); // any --> ENABLED
    else
      return;
  }
  else if (acquisition_complete_)
  {
    QMessageBox::StandardButton reply = QMessageBox::question(
      this, "Homing in progress", "If you disable the motors all data collected so far "
                                  "will be lost and you will have to start over the "
                                  "procedure before starting a new application.\nAre you "
                                  "sure you want to continue?",
      QMessageBox::Yes | QMessageBox::No);
    CLOG(INFO, "event") << "DISABLE? --> " << (reply == QMessageBox::Yes);
    if (reply == QMessageBox::No)
      return;
  }
  app_.Stop(); // ENABLED --> IDLE
}

void HomingInterfaceProprioceptive::on_pushButton_clearFaults_clicked()
{
  CLOG(TRACE, "event");
  robot_ptr_->ClearFaults();
}

void HomingInterfaceProprioceptive::on_checkBox_useCurrentTorque_stateChanged(int)
{
  CLOG(TRACE, "event");
  std::vector<quint8> motors_id = {0, 1, 2, 3,
                                   4, 5, 6, 7}; // robot_ptr_->GetMotorsID(); debug
  for (quint8 motor_id : motors_id)
  {
    if (ui->checkBox_useCurrentTorque->isChecked())
      init_torque_forms_[motor_id]->SetInitTorque(300 + motor_id * 100);
    // robot_ptr_->GetMotorStatus(motor_id).torque_target);
    init_torque_forms_[motor_id]->EnableInitTorque(
      !ui->checkBox_useCurrentTorque->isChecked());
  }
}

void HomingInterfaceProprioceptive::on_checkBox_maxTorque_stateChanged(int)
{
  CLOG(TRACE, "event");
  qint16 max_init_torque = 0;
  for (auto& init_torque_form : init_torque_forms_)
  {
    init_torque_form->EnableMaxTorque(!ui->checkBox_maxTorque->isChecked());
    max_init_torque = std::max(max_init_torque, init_torque_form->GetInitTorque());
  }
  ui->spinBox_maxTorque->setMinimum(max_init_torque);
  ui->spinBox_maxTorque->setEnabled(ui->checkBox_maxTorque->isChecked());
}

void HomingInterfaceProprioceptive::on_pushButton_start_clicked()
{
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
      app_.Stop(); // any --> ENABLED
    return;
  }

  CLOG(TRACE, "event") << "(START)";
  HomingProprioceptiveStartData* data = new HomingProprioceptiveStartData();
  data->num_meas = static_cast<quint8>(ui->spinBox_numMeas->value());
  for (InitTorqueForm* form : init_torque_forms_)
  {
    if (!ui->checkBox_useCurrentTorque->isChecked())
      data->init_torques.push_back(form->GetInitTorque());

    data->max_torques.push_back(ui->checkBox_maxTorque->isChecked()
                                  ? static_cast<qint16>(ui->spinBox_maxTorque->value())
                                  : form->GetMaxTorque());
  }
  app_.Start(data);
  acquisition_complete_ = false;
}

void HomingInterfaceProprioceptive::on_radioButton_internal_clicked()
{
  CLOG(TRACE, "event");
  ui->radioButton_external->toggled(false);
  ui->lineEdit_extFile->setDisabled(true);
  ui->pushButton_extFile->setDisabled(true);
}

void HomingInterfaceProprioceptive::on_radioButton_external_clicked()
{
  CLOG(TRACE, "event");
  ui->radioButton_internal->toggled(false);
  ui->lineEdit_extFile->setEnabled(true);
  ui->pushButton_extFile->setEnabled(true);
}

void HomingInterfaceProprioceptive::on_pushButton_extFile_clicked()
{
  CLOG(TRACE, "event");
  QString config_filename =
    QFileDialog::getOpenFileName(this, tr("Load Optimization Results"), tr("../.."),
                                 tr("Optimization results (*.txt)"));
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
  if (ui->radioButton_internal->isChecked())
  {
    ui->groupBox_dataCollection->setEnabled(false);
    app_.Optimize();
    return;
  }

  HomingProprioceptiveHomeData* res = new HomingProprioceptiveHomeData;
  if (!ParseExtFile(res))
  {
    QMessageBox::warning(this, "File Error",
                         "File content is not valid!\nPlease load a different file.");
    return;
  }
  ui->groupBox_dataCollection->setEnabled(false);
  app_.GoHome(res);
}

void HomingInterfaceProprioceptive::on_pushButton_cancel_clicked()
{
  CLOG(TRACE, "event");
  switch (static_cast<HomingProprioceptive::States>(app_.GetCurrentState()))
  {
  case HomingProprioceptive::ST_IDLE:
    break;
  case HomingProprioceptive::ST_OPTIMIZING:
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
  case HomingProprioceptive::ST_HOME:
  {
    QMessageBox::StandardButton reply = QMessageBox::question(
      this, "Homing in progress", "The robot is moving to the homing position. If you "
                                  "quit now all progress will be lost.\nAre you sure "
                                  "you want to abort the operation?",
      QMessageBox::Yes | QMessageBox::No);
    if (reply == QMessageBox::No)
      return;
    CLOG(INFO, "event") << "Homing interrupted by user while moving to home position";
    break;
  }
  case HomingProprioceptive::ST_FAULT:
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

  ui->pushButton_enable->click();
  emit homingFailed();
  close();
}

void HomingInterfaceProprioceptive::on_pushButton_done_clicked()
{
  CLOG(TRACE, "event");
  emit homingSuccess();
  close();
}

//--------- Private slots -----------------------------------------------------//

void HomingInterfaceProprioceptive::appendText2Browser(const QString& text)
{
  if (text.contains("warning", Qt::CaseSensitivity::CaseInsensitive))
    CLOG(WARNING, "browser") << text;
  else if (text.contains("error", Qt::CaseSensitivity::CaseInsensitive))
    CLOG(ERROR, "browser") << text;
  else
    CLOG(INFO, "browser") << text;
  ui->textBrowser_logs->append(text);
}

void HomingInterfaceProprioceptive::updateAcquisitionProgress(const int value)
{
  if (value > ui->progressBar_acquisition->value())
    ui->progressBar_acquisition->setValue(value);
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
  acquisition_complete_ = true;
}

void HomingInterfaceProprioceptive::handleHomingComplete()
{
  CLOG(INFO, "event") << "Homing complete";
  ui->groupBox_dataCollection->setEnabled(true);
  ui->pushButton_done->setEnabled(true);
}

void HomingInterfaceProprioceptive::handleStateChanged(const quint8& state)
{
  switch (state)
  {
  case HomingProprioceptive::ST_IDLE:
    ui->pushButton_enable->setText(tr("Enable"));
    ui->pushButton_start->setDisabled(true);
    ui->pushButton_start->setText(tr("Start"));
    break;
  case HomingProprioceptive::ST_ENABLED:
    ui->pushButton_enable->setText(tr("Disable"));
    ui->pushButton_start->setEnabled(true);
    ui->pushButton_start->setText(tr("Start"));
    break;
  case HomingProprioceptive::ST_START_UP:
    ui->pushButton_start->setText(tr("Stop"));
    break;
  case HomingProprioceptive::ST_FAULT:
    ui->pushButton_enable->setDisabled(true);
    ui->pushButton_start->setDisabled(true);
    ui->pushButton_clearFaults->setEnabled(true);
    break;
  default:
    break;
  }
}

//--------- Private functions -----------------------------------------------//

bool HomingInterfaceProprioceptive::ParseExtFile(HomingProprioceptiveHomeData* res)
{
  // Read external file and fill res struct
  return true; // dummy
}
