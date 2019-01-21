#include "gui/main_gui.h"
#include "ui_main_gui.h"

MainGUI::MainGUI(QWidget* parent, const grabcdpr::Params& config)
  : QDialog(parent), ui(new Ui::MainGUI), robot_(this, config)
{
  ui->setupUi(this);

  for (size_t i = 0; i < config.actuators.size(); i++)
    if (config.actuators[i].active)
      ui->comboBox_motorAxis->addItem(QString::number(i + 1));

  connect(&robot_, SIGNAL(printToQConsole(QString)), this,
          SLOT(appendText2Browser(QString)));
  connect(&robot_, SIGNAL(motorStatus(quint64, grabec::GSWDriveInPdos)), this,
          SLOT(updateDriveStatusTable(quint64, grabec::GSWDriveInPdos)));
  connect(&robot_, SIGNAL(ecStateChanged(Bitfield8)), this,
          SLOT(updateEcStatusLED(Bitfield8)));
  connect(&robot_, SIGNAL(requestSatisfied()), this, SLOT(updateDirectDriveCtrlPanel()));

  robot_.eventSuccess(); // pwd & config OK --> robot ENABLED
  if (robot_.GetCurrentState() == CableRobot::ST_ENABLED)
    robot_.Start();
}

MainGUI::~MainGUI()
{
  disconnect(&robot_, SIGNAL(printToQConsole(QString)), this,
             SLOT(appendText2Browser(QString)));
  disconnect(&robot_, SIGNAL(motorStatus(quint64, grabec::GSWDriveInPdos)), this,
             SLOT(updateDriveStatusTable(quint64, grabec::GSWDriveInPdos)));
  disconnect(&robot_, SIGNAL(ecStateChanged(Bitfield8)), this,
             SLOT(updateEcStatusLED(Bitfield8)));
  disconnect(&robot_, SIGNAL(requestSatisfied()), this,
             SLOT(updateDirectDriveCtrlPanel()));

  if (calib_dialog_ != NULL)
  {
    disconnect(calib_dialog_, SIGNAL(enableMainGUI()), this, SLOT(enableInterface()));
    disconnect(calib_dialog_, SIGNAL(calibrationEnd()), &robot_, SLOT(eventSuccess()));
    delete calib_dialog_;
  }
  if (homing_dialog_ != NULL)
  {
    disconnect(homing_dialog_, SIGNAL(enableMainGUI(bool)), this,
               SLOT(enableInterface(bool)));
    disconnect(homing_dialog_, SIGNAL(homingSuccess()), &robot_, SLOT(eventSuccess()));
    disconnect(homing_dialog_, SIGNAL(homingFailed()), &robot_, SLOT(eventFailure()));
    delete homing_dialog_;
  }
  delete ui;
  CLOG(INFO, "event") << "Main window closed";
}

//--------- Public GUI slots --------------------------------------------------//

void MainGUI::on_pushButton_calib_clicked()
{
  CLOG(TRACE, "event");
  if (!ec_network_valid_)
    return;

  if (robot_.GetCurrentState() == CableRobot::ST_READY)
    if (!ExitReadyStateRequest())
      return;

  ui->pushButton_homing->setDisabled(true);
  ui->pushButton_calib->setDisabled(true);
  ui->groupBox_app->setDisabled(true);
  ui->frame_manualControl->setDisabled(true);

  robot_.enterCalibrationMode();

  calib_dialog_ = new CalibrationDialog(this, &robot_);
  connect(calib_dialog_, SIGNAL(enableMainGUI()), this, SLOT(enableInterface()));
  connect(calib_dialog_, SIGNAL(calibrationEnd()), &robot_, SLOT(eventSuccess()));
  calib_dialog_->show();
  CLOG(INFO, "event") << "Prompt calibration dialog";
}

void MainGUI::on_pushButton_homing_clicked()
{
  CLOG(TRACE, "event");
  if (!ec_network_valid_)
    return;

  ui->pushButton_homing->setDisabled(true);
  ui->pushButton_calib->setDisabled(true);
  ui->groupBox_app->setDisabled(true);
  ui->frame_manualControl->setDisabled(true);

  robot_.enterHomingMode();

  if (homing_dialog_ == NULL)
  {
    homing_dialog_ = new HomingDialog(this, &robot_);
    connect(homing_dialog_, SIGNAL(enableMainGUI(bool)), this,
            SLOT(enableInterface(bool)));
    connect(homing_dialog_, SIGNAL(homingSuccess()), &robot_, SLOT(eventSuccess()));
    connect(homing_dialog_, SIGNAL(homingFailed()), &robot_, SLOT(eventFailure()));
  }
  homing_dialog_->show();
  CLOG(INFO, "event") << "Prompt homing dialog";
}

void MainGUI::on_pushButton_startApp_clicked()
{
  CLOG(TRACE, "event");
  //  ui->pushButton_homing->setDisabled(true);
  //  ui->pushButton_calib->setDisabled(true);
  //  ui->groupBox_app->setDisabled(true);
  //  ui->frame_manualControl->setDisabled(true);
}

void MainGUI::on_pushButton_enable_clicked()
{
  CLOG(TRACE, "event");
  if (!ec_network_valid_)
    return;

  if (robot_.GetCurrentState() == CableRobot::ST_READY)
    if (!ExitReadyStateRequest())
      return;

  bool manual_ctrl_enabled = !manual_ctrl_enabled_;

  SetupDirectMotorCtrl(manual_ctrl_enabled);
  waiting_for_response_ = true;
}

void MainGUI::on_pushButton_faultReset_clicked()
{
  CLOG(TRACE, "event");
  robot_.ClearFaults();
}

void MainGUI::on_radioButton_posMode_clicked()
{
  CLOG(TRACE, "event");
  ui->radioButton_posMode->setChecked(true);
  ui->radioButton_velMode->setChecked(false);
  ui->radioButton_torqueMode->setChecked(false);

  if (manual_ctrl_enabled_)
  {
    DisablePosCtrlButtons(false);
    DisableVelCtrlButtons(true);
    DisableTorqueCtrlButtons(true);
    pthread_mutex_lock(&robot_.Mutex());
    man_ctrl_ptr_->SetCableLenTarget(robot_.GetActuatorStatus(motor_id_).cable_length);
    man_ctrl_ptr_->SetMode(ControlMode::CABLE_LENGTH);
    pthread_mutex_unlock(&robot_.Mutex());
  }
}

void MainGUI::on_radioButton_velMode_clicked()
{
  CLOG(TRACE, "event");
  ui->radioButton_posMode->setChecked(false);
  ui->radioButton_velMode->setChecked(true);
  ui->radioButton_torqueMode->setChecked(false);

  if (manual_ctrl_enabled_)
  {
    DisablePosCtrlButtons(true);
    DisableVelCtrlButtons(false);
    DisableTorqueCtrlButtons(true);
    pthread_mutex_lock(&robot_.Mutex());
    man_ctrl_ptr_->SetMotorSpeedTarget(robot_.GetActuatorStatus(motor_id_).motor_speed);
    man_ctrl_ptr_->SetMode(ControlMode::MOTOR_SPEED);
    pthread_mutex_unlock(&robot_.Mutex());
  }
}

void MainGUI::on_radioButton_torqueMode_clicked()
{
  CLOG(TRACE, "event");
  ui->radioButton_posMode->setChecked(false);
  ui->radioButton_velMode->setChecked(false);
  ui->radioButton_torqueMode->setChecked(true);

  if (manual_ctrl_enabled_)
  {
    DisablePosCtrlButtons(true);
    DisableVelCtrlButtons(true);
    DisableTorqueCtrlButtons(false);
    pthread_mutex_lock(&robot_.Mutex());
    man_ctrl_ptr_->SetMotorTorqueTarget(robot_.GetActuatorStatus(motor_id_).motor_torque);
    man_ctrl_ptr_->SetMode(ControlMode::MOTOR_TORQUE);
    pthread_mutex_unlock(&robot_.Mutex());
  }
}

void MainGUI::on_pushButton_posPlus_pressed()
{
  CLOG(TRACE, "event");
  pthread_mutex_lock(&robot_.Mutex());
  man_ctrl_ptr_->CableLenIncrement(true, Sign::POS, false);
  pthread_mutex_unlock(&robot_.Mutex());
}

void MainGUI::on_pushButton_posPlus_released()
{
  CLOG(TRACE, "event");
  pthread_mutex_lock(&robot_.Mutex());
  man_ctrl_ptr_->CableLenIncrement(false);
  pthread_mutex_unlock(&robot_.Mutex());
}

void MainGUI::on_pushButton_posMinus_pressed()
{
  CLOG(TRACE, "event");
  pthread_mutex_lock(&robot_.Mutex());
  man_ctrl_ptr_->CableLenIncrement(true, Sign::NEG, false);
  pthread_mutex_unlock(&robot_.Mutex());
}

void MainGUI::on_pushButton_posMinus_released()
{
  CLOG(TRACE, "event");
  pthread_mutex_lock(&robot_.Mutex());
  man_ctrl_ptr_->CableLenIncrement(false);
  pthread_mutex_unlock(&robot_.Mutex());
}

void MainGUI::on_pushButton_posMicroPlus_pressed()
{
  CLOG(TRACE, "event");
  pthread_mutex_lock(&robot_.Mutex());
  man_ctrl_ptr_->CableLenIncrement(true, Sign::POS, true);
  pthread_mutex_unlock(&robot_.Mutex());
}

void MainGUI::on_pushButton_posMicroPlus_released()
{
  CLOG(TRACE, "event");
  pthread_mutex_lock(&robot_.Mutex());
  man_ctrl_ptr_->CableLenIncrement(false);
  pthread_mutex_unlock(&robot_.Mutex());
}

void MainGUI::on_pushButton_posMicroMinus_pressed()
{
  CLOG(TRACE, "event");
  pthread_mutex_lock(&robot_.Mutex());
  man_ctrl_ptr_->CableLenIncrement(true, Sign::NEG, true);
  pthread_mutex_unlock(&robot_.Mutex());
}

void MainGUI::on_pushButton_posMicroMinus_released()
{
  CLOG(TRACE, "event");
  pthread_mutex_lock(&robot_.Mutex());
  man_ctrl_ptr_->CableLenIncrement(false);
  pthread_mutex_unlock(&robot_.Mutex());
}

void MainGUI::on_pushButton_speedPlus_clicked()
{
  CLOG(TRACE, "event");
  pthread_mutex_lock(&robot_.Mutex());
  man_ctrl_ptr_->MotorSpeedIncrement(Sign::POS);
  pthread_mutex_unlock(&robot_.Mutex());
}

void MainGUI::on_pushButton_speedMinus_clicked()
{
  CLOG(TRACE, "event");
  pthread_mutex_lock(&robot_.Mutex());
  man_ctrl_ptr_->MotorSpeedIncrement(Sign::NEG);
  pthread_mutex_unlock(&robot_.Mutex());
}

void MainGUI::on_pushButton_torquePlus_clicked()
{
  CLOG(TRACE, "event");
  pthread_mutex_lock(&robot_.Mutex());
  man_ctrl_ptr_->MotorTorqueIncrement(Sign::POS);
  pthread_mutex_unlock(&robot_.Mutex());
}

void MainGUI::on_pushButton_torqueMinus_clicked()
{
  CLOG(TRACE, "event");
  pthread_mutex_lock(&robot_.Mutex());
  man_ctrl_ptr_->MotorTorqueIncrement(Sign::NEG);
  pthread_mutex_unlock(&robot_.Mutex());
}

//--------- Private slots --------------------------------------------------//

void MainGUI::enableInterface(const bool op_outcome /*= false*/)
{
  ui->pushButton_homing->setEnabled(true);
  ui->pushButton_calib->setEnabled(true);
  ui->groupBox_app->setEnabled(op_outcome);
  ui->frame_manualControl->setEnabled(true);
  CVLOG(2, "event") << "Interface enabled with app selection "
                    << (op_outcome ? "enabled" : "disabled");
}

void MainGUI::appendText2Browser(const QString& text)
{
  if (text.contains("warning", Qt::CaseSensitivity::CaseInsensitive))
    CLOG(WARNING, "browser") << text;
  else if (text.contains("error", Qt::CaseSensitivity::CaseInsensitive))
    CLOG(ERROR, "browser") << text;
  else
    CLOG(INFO, "browser") << text;
  ui->textBrowser_logs->append(text);
}

void MainGUI::updateDriveStatusTable(const quint64 id,
                                     const grabec::GSWDriveInPdos& status)
{
  static const quint8 kUpdateCycle = 50; // ~ms
  static quint8 counter = 1;

  // Check signal corresponds to current selected axis
  if (id != ui->comboBox_motorAxis->currentText().toULong() - 1)
    return;
  // Considering a RT cycle of 1ms, update GUI at a slower rate
  if (counter++ > kUpdateCycle)
  {
    counter = 1;
    return;
  }
  // Update table
  //  std::string status_word = "0b" +
  //  status.status_word.GetBitset().to_string().substr(9, 7);
  std::string status_word =
    grabec::GoldSoloWhistleDrive::GetDriveStateStr(status.status_word);
  ui->table_inputPdos->item(0, 0)->setData(Qt::DisplayRole, status_word.c_str());
  std::string op_mode_str;
  switch (status.display_op_mode)
  {
  case grabec::NONE:
    op_mode_str = "CYCLIC_SYNC_POSITION";
    break;
  case grabec::PROFILE_POSITION:
    op_mode_str = "PROFILE_POSITION";
    break;
  case grabec::VELOCITY_MODE:
    op_mode_str = "VELOCITY_MODE";
    break;
  case grabec::PROFILE_VELOCITY:
    op_mode_str = "PROFILE_VELOCITY";
    break;
  case grabec::TORQUE_PROFILE:
    op_mode_str = "TORQUE_PROFILE";
    break;
  case grabec::HOMING:
    op_mode_str = "HOMING";
    break;
  case grabec::INTERPOLATED_POSITION:
    op_mode_str = "INTERPOLATED_POSITION";
    break;
  case grabec::CYCLIC_POSITION:
    op_mode_str = "CYCLIC_SYNC_POSITION";
    break;
  case grabec::CYCLIC_VELOCITY:
    op_mode_str = "CYCLIC_SYNC_VELOCITY";
    break;
  case grabec::CYCLIC_TORQUE:
    op_mode_str = "CYCLIC_SYNC_TORQUE";
    break;
  default:
    op_mode_str = "UNKNOWN: " + std::to_string(status.display_op_mode);
    break;
  }
  ui->table_inputPdos->item(1, 0)->setData(Qt::DisplayRole, op_mode_str.c_str());
  ui->table_inputPdos->item(2, 0)->setData(Qt::DisplayRole, status.pos_actual_value);
  ui->table_inputPdos->item(3, 0)->setData(Qt::DisplayRole, status.vel_actual_value);
  ui->table_inputPdos->item(4, 0)->setData(Qt::DisplayRole, status.torque_actual_value);
  ui->table_inputPdos->item(5, 0)->setData(Qt::DisplayRole, status.digital_inputs);
  ui->table_inputPdos->item(6, 0)->setData(Qt::DisplayRole, status.aux_pos_actual_value);
  CVLOG(2, "event") << "Drive status table updated";
}

void MainGUI::updateEcStatusLED(const Bitfield8& ec_status_flags)
{
  switch (ec_status_flags.Count())
  {
  case 3: // OK (all 3 checks passed)
    ui->label_ec_status_led->setPixmap(
      QPixmap(QString::fromUtf8(":/img/img/green_button.png")));
    ec_network_valid_ = true;
    break;
  case 0: // failed at initialization (no checks passed)
    ui->label_ec_status_led->setPixmap(
      QPixmap(QString::fromUtf8(":/img/img/red_button.png")));
    ec_network_valid_ = false;
    break;
  default: // something is wrong (some checks passed)
    ui->label_ec_status_led->setPixmap(
      QPixmap(QString::fromUtf8(":/img/img/yellow_button.png")));
    ec_network_valid_ = false;
    break;
  }
}

void MainGUI::updateDirectDriveCtrlPanel()
{
  if (!waiting_for_response_)
    return;

  manual_ctrl_enabled_ = !manual_ctrl_enabled_;

  ui->pushButton_enable->setText(tr(manual_ctrl_enabled_ ? "Disable" : "Enable"));

  ui->pushButton_homing->setDisabled(manual_ctrl_enabled_);
  ui->pushButton_calib->setDisabled(manual_ctrl_enabled_);
  ui->groupBox_app->setDisabled(true); // after we move we need to do the homing again

  DisablePosCtrlButtons(!(manual_ctrl_enabled_ && ui->radioButton_posMode->isChecked()));
  DisableVelCtrlButtons(!(manual_ctrl_enabled_ && ui->radioButton_velMode->isChecked()));
  DisableTorqueCtrlButtons(
    !(manual_ctrl_enabled_ && ui->radioButton_torqueMode->isChecked()));

  waiting_for_response_ = false;
}

//--------- Private GUI methods --------------------------------------------------//

void MainGUI::DisablePosCtrlButtons(const bool value)
{
  ui->pushButton_posMicroMinus->setDisabled(value);
  ui->pushButton_posMicroPlus->setDisabled(value);
  ui->pushButton_posMinus->setDisabled(value);
  ui->pushButton_posPlus->setDisabled(value);
  CVLOG(2, "event") << "Cable lenght direct control buttons "
                    << (value ? "enabled" : "disabled");
}

void MainGUI::DisableVelCtrlButtons(const bool value)
{
  ui->pushButton_speedMinus->setDisabled(value);
  ui->pushButton_speedPlus->setDisabled(value);
  CVLOG(2, "event") << "Motor velocity direct control buttons "
                    << (value ? "enabled" : "disabled");
}

void MainGUI::DisableTorqueCtrlButtons(const bool value)
{
  ui->pushButton_torqueMinus->setDisabled(value);
  ui->pushButton_torquePlus->setDisabled(value);
  CVLOG(2, "event") << "Motor torque direct control buttons "
                    << (value ? "enabled" : "disabled");
}

bool MainGUI::ExitReadyStateRequest()
{
  QMessageBox::StandardButton reply = QMessageBox::question(
    this, "Robot ready", "If you proceed with direct manual motor control all homing "
                         "results will be lost and you will have to repeat the "
                         "procedure before starting a new application.\nAre you sure "
                         "you want to continue?",
    QMessageBox::Yes | QMessageBox::No);
  CLOG(TRACE, "event") << "--> " << (reply == QMessageBox::Yes);
  return (reply == QMessageBox::Yes);
}

//--------- Private methods --------------------------------------------------//

void MainGUI::SetupDirectMotorCtrl(const bool enable)
{
  robot_.stop(); // robot: READY | ENABLED --> ENABLED

  if (enable)
  {
    motor_id_ = ui->comboBox_motorAxis->currentText().toULong() - 1;
    // Setup controller before enabling the motor
    man_ctrl_ptr_ = new ControllerSingleDriveNaive(motor_id_);
    ActuatorStatus current_status = robot_.GetActuatorStatus(motor_id_);
    if (ui->radioButton_posMode->isChecked())
    {
      man_ctrl_ptr_->SetMode(ControlMode::CABLE_LENGTH);
      man_ctrl_ptr_->SetCableLenTarget(current_status.cable_length);
    }
    if (ui->radioButton_velMode->isChecked())
    {
      man_ctrl_ptr_->SetMode(ControlMode::MOTOR_SPEED);
      man_ctrl_ptr_->SetMotorSpeedTarget(current_status.motor_speed);
    }
    if (ui->radioButton_torqueMode->isChecked())
    {
      man_ctrl_ptr_->SetMode(ControlMode::MOTOR_TORQUE);
      man_ctrl_ptr_->SetMotorTorqueTarget(current_status.motor_torque);
    }
    robot_.SetController(man_ctrl_ptr_);

    robot_.EnableMotor(motor_id_);
  }
  else
  {
    pthread_mutex_lock(&robot_.Mutex());
    delete man_ctrl_ptr_;
    man_ctrl_ptr_ = NULL;
    pthread_mutex_unlock(&robot_.Mutex());
    robot_.SetController(man_ctrl_ptr_);

    robot_.DisableMotor(motor_id_);
  }
}
