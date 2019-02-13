#include "gui/main_gui.h"
#include "ui_main_gui.h"

MainGUI::MainGUI(QWidget* parent, const grabcdpr::Params& config)
  : QDialog(parent), ui(new Ui::MainGUI), config_params_(config)
{
  ui->setupUi(this);

  // Setup flags
  waiting_for_response_.ClearAll();
  desired_ctrl_mode_.ClearAll();
  desired_ctrl_mode_.Set(ControlMode::CABLE_LENGTH);

  for (size_t i = 0; i < config.actuators.size(); i++)
    if (config.actuators[i].active)
      ui->comboBox_motorAxis->addItem(QString::number(i));

  StartRobot(); // instantiate cable robot object
}

MainGUI::~MainGUI()
{
  CloseAllApps();
  DeleteRobot();
  delete ui;
  CLOG(INFO, "event") << "Main window closed";
}

//--------- Public GUI slots ---------------------------------------------------------//

void MainGUI::on_pushButton_reset_clicked()
{
  CLOG(TRACE, "event");
  robot_ptr_->Reset();
  ui->frame_manualControl->setEnabled(true);
}

void MainGUI::on_pushButton_calib_clicked()
{
  CLOG(TRACE, "event");
  if (!(ec_network_valid_ && rt_thread_running_))
    return;

  if (robot_ptr_->GetCurrentState() == CableRobot::ST_READY)
    if (!ExitReadyStateRequest())
      return;

  ui->pushButton_homing->setDisabled(true);
  ui->pushButton_calib->setDisabled(true);
  ui->groupBox_app->setDisabled(true);
  ui->frame_manualControl->setDisabled(true);

  robot_ptr_->enterCalibrationMode();

  calib_dialog_ = new CalibrationDialog(this, robot_ptr_);
  connect(calib_dialog_, SIGNAL(enableMainGUI()), this, SLOT(enableInterface()));
  connect(calib_dialog_, SIGNAL(calibrationEnd()), robot_ptr_, SLOT(eventSuccess()));
  calib_dialog_->show();
  CLOG(INFO, "event") << "Prompt calibration dialog";
}

void MainGUI::on_pushButton_homing_clicked()
{
  CLOG(TRACE, "event");
  if (!(ec_network_valid_ && rt_thread_running_))
    return;

  ui->pushButton_homing->setDisabled(true);
  ui->pushButton_calib->setDisabled(true);
  ui->groupBox_app->setDisabled(true);
  ui->frame_manualControl->setDisabled(true);

  robot_ptr_->enterHomingMode();

  if (homing_dialog_ == NULL)
  {
    homing_dialog_ = new HomingDialog(this, robot_ptr_);
    connect(homing_dialog_, SIGNAL(enableMainGUI(bool)), this,
            SLOT(enableInterface(bool)));
    connect(homing_dialog_, SIGNAL(homingSuccess()), robot_ptr_, SLOT(eventSuccess()));
    connect(homing_dialog_, SIGNAL(homingFailed()), robot_ptr_, SLOT(eventFailure()));
  }
  homing_dialog_->show();
  CLOG(INFO, "event") << "Prompt homing dialog";
}

void MainGUI::on_pushButton_startApp_clicked()
{
  CLOG(TRACE, "event");
  if (!(ec_network_valid_ && rt_thread_running_))
    return;

  ui->pushButton_homing->setDisabled(true);
  ui->pushButton_calib->setDisabled(true);
  ui->groupBox_app->setDisabled(true);
  ui->frame_manualControl->setDisabled(true);
}

void MainGUI::on_pushButton_enable_clicked()
{
  CLOG(TRACE, "event");
  ui->pushButton_enable->setChecked(false);
  if (!(ec_network_valid_ && rt_thread_running_))
    return;

  if (robot_ptr_->GetCurrentState() == CableRobot::ST_READY)
    if (!ExitReadyStateRequest())
      return;

  bool manual_ctrl_enabled = !manual_ctrl_enabled_;

  SetupDirectMotorCtrl(manual_ctrl_enabled);
  waiting_for_response_.Set(manual_ctrl_enabled ? Actuator::ST_ENABLED
                                                : Actuator::ST_IDLE);
}

void MainGUI::on_pushButton_faultReset_clicked()
{
  CLOG(TRACE, "event");
  robot_ptr_->ClearFaults();
}

void MainGUI::on_radioButton_posMode_clicked()
{
  CLOG(TRACE, "event");
  ui->radioButton_posMode->setChecked(true);
  ui->radioButton_velMode->setChecked(false);
  ui->radioButton_torqueMode->setChecked(false);

  if (desired_ctrl_mode_.CheckBit(ControlMode::CABLE_LENGTH))
    return;
  desired_ctrl_mode_.ClearAll();
  desired_ctrl_mode_.Set(ControlMode::CABLE_LENGTH);

  if (manual_ctrl_enabled_)
  {
    man_ctrl_ptr_->SetCableLenTarget(
      robot_ptr_->GetActuatorStatus(motor_id_).cable_length);
    pthread_mutex_lock(&robot_ptr_->Mutex());
    man_ctrl_ptr_->SetMode(ControlMode::CABLE_LENGTH);
    pthread_mutex_unlock(&robot_ptr_->Mutex());
    waiting_for_response_.Set(Actuator::ST_ENABLED);
  }
}

void MainGUI::on_radioButton_velMode_clicked()
{
  CLOG(TRACE, "event");
  ui->radioButton_posMode->setChecked(false);
  ui->radioButton_velMode->setChecked(true);
  ui->radioButton_torqueMode->setChecked(false);

  if (desired_ctrl_mode_.CheckBit(ControlMode::MOTOR_SPEED))
    return;
  desired_ctrl_mode_.ClearAll();
  desired_ctrl_mode_.Set(ControlMode::MOTOR_SPEED);

  if (manual_ctrl_enabled_)
  {
    man_ctrl_ptr_->SetMotorSpeedTarget(0);
    pthread_mutex_lock(&robot_ptr_->Mutex());
    man_ctrl_ptr_->SetMode(ControlMode::MOTOR_SPEED);
    pthread_mutex_unlock(&robot_ptr_->Mutex());
    waiting_for_response_.Set(Actuator::ST_ENABLED);
  }
}

void MainGUI::on_radioButton_torqueMode_clicked()
{
  CLOG(TRACE, "event");
  ui->radioButton_posMode->setChecked(false);
  ui->radioButton_velMode->setChecked(false);
  ui->radioButton_torqueMode->setChecked(true);

  if (desired_ctrl_mode_.CheckBit(ControlMode::MOTOR_TORQUE))
    return;
  desired_ctrl_mode_.ClearAll();
  desired_ctrl_mode_.Set(ControlMode::MOTOR_TORQUE);

  if (manual_ctrl_enabled_)
  {
    man_ctrl_ptr_->SetMotorTorqueTarget(
      robot_ptr_->GetActuatorStatus(motor_id_).motor_torque);
    pthread_mutex_lock(&robot_ptr_->Mutex());
    man_ctrl_ptr_->SetMode(ControlMode::MOTOR_TORQUE);
    pthread_mutex_unlock(&robot_ptr_->Mutex());
    waiting_for_response_.Set(Actuator::ST_ENABLED);
  }
}

void MainGUI::on_pushButton_posPlus_pressed()
{
  CLOG(TRACE, "event");
  pthread_mutex_lock(&robot_ptr_->Mutex());
  man_ctrl_ptr_->CableLenIncrement(true, Sign::POS, false);
  pthread_mutex_unlock(&robot_ptr_->Mutex());
}

void MainGUI::on_pushButton_posPlus_released()
{
  CLOG(TRACE, "event");
  pthread_mutex_lock(&robot_ptr_->Mutex());
  man_ctrl_ptr_->CableLenIncrement(false);
  pthread_mutex_unlock(&robot_ptr_->Mutex());
}

void MainGUI::on_pushButton_posMinus_pressed()
{
  CLOG(TRACE, "event");
  pthread_mutex_lock(&robot_ptr_->Mutex());
  man_ctrl_ptr_->CableLenIncrement(true, Sign::NEG, false);
  pthread_mutex_unlock(&robot_ptr_->Mutex());
}

void MainGUI::on_pushButton_posMinus_released()
{
  CLOG(TRACE, "event");
  pthread_mutex_lock(&robot_ptr_->Mutex());
  man_ctrl_ptr_->CableLenIncrement(false);
  pthread_mutex_unlock(&robot_ptr_->Mutex());
}

void MainGUI::on_pushButton_posMicroPlus_pressed()
{
  CLOG(TRACE, "event");
  pthread_mutex_lock(&robot_ptr_->Mutex());
  man_ctrl_ptr_->CableLenIncrement(true, Sign::POS, true);
  pthread_mutex_unlock(&robot_ptr_->Mutex());
}

void MainGUI::on_pushButton_posMicroPlus_released()
{
  CLOG(TRACE, "event");
  pthread_mutex_lock(&robot_ptr_->Mutex());
  man_ctrl_ptr_->CableLenIncrement(false);
  pthread_mutex_unlock(&robot_ptr_->Mutex());
}

void MainGUI::on_pushButton_posMicroMinus_pressed()
{
  CLOG(TRACE, "event");
  pthread_mutex_lock(&robot_ptr_->Mutex());
  man_ctrl_ptr_->CableLenIncrement(true, Sign::NEG, true);
  pthread_mutex_unlock(&robot_ptr_->Mutex());
}

void MainGUI::on_pushButton_posMicroMinus_released()
{
  CLOG(TRACE, "event");
  pthread_mutex_lock(&robot_ptr_->Mutex());
  man_ctrl_ptr_->CableLenIncrement(false);
  pthread_mutex_unlock(&robot_ptr_->Mutex());
}

void MainGUI::on_horizontalSlider_speed_ctrl_sliderPressed() { CLOG(TRACE, "event"); }

void MainGUI::on_horizontalSlider_speed_ctrl_sliderMoved(int position)
{
  CLOG(TRACE, "event");
  pthread_mutex_lock(&robot_ptr_->Mutex());
  man_ctrl_ptr_->ScaleMotorSpeed(position * 0.01);
  pthread_mutex_unlock(&robot_ptr_->Mutex());
}

void MainGUI::on_horizontalSlider_speed_ctrl_sliderReleased()
{
  CLOG(TRACE, "event");
  ui->horizontalSlider_speed_ctrl->setValue(0);
  ui->horizontalSlider_speed_ctrl->sliderMoved(ui->horizontalSlider_speed_ctrl->value());
}

void MainGUI::on_pushButton_torquePlus_pressed()
{
  CLOG(TRACE, "event");
  pthread_mutex_lock(&robot_ptr_->Mutex());
  man_ctrl_ptr_->MotorTorqueIncrement(true, Sign::POS);
  pthread_mutex_unlock(&robot_ptr_->Mutex());
}

void MainGUI::on_pushButton_torquePlus_released()
{
  CLOG(TRACE, "event");
  pthread_mutex_lock(&robot_ptr_->Mutex());
  man_ctrl_ptr_->MotorTorqueIncrement(false);
  pthread_mutex_unlock(&robot_ptr_->Mutex());
}

void MainGUI::on_pushButton_torqueMinus_pressed()
{
  CLOG(TRACE, "event");
  pthread_mutex_lock(&robot_ptr_->Mutex());
  man_ctrl_ptr_->MotorTorqueIncrement(true, Sign::NEG);
  pthread_mutex_unlock(&robot_ptr_->Mutex());
}

void MainGUI::on_pushButton_torqueMinus_released()
{
  CLOG(TRACE, "event");
  pthread_mutex_lock(&robot_ptr_->Mutex());
  man_ctrl_ptr_->MotorTorqueIncrement(false);
  pthread_mutex_unlock(&robot_ptr_->Mutex());
}

//--------- Private slots ------------------------------------------------------------//

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
  ui->pushButton_reset->setDisabled(ec_network_valid_ && rt_thread_running_);
}

void MainGUI::updateRtThreadStatusLED(const bool active)
{
  rt_thread_running_ = active;
  ui->pushButton_reset->setDisabled(ec_network_valid_ && rt_thread_running_);
  ui->label_rt_thread_status_led->setPixmap(QPixmap(QString::fromUtf8(
    active ? ":/img/img/green_button.png" : ":/img/img/red_button.png")));

  if (!rt_thread_running_)
  {
    QMessageBox::warning(this, "Thread Error",
                         "Real-Time thread missed its dealine and "
                         "automatically shut down!\nPlease reset the robot.");
    CloseAllApps();

    // Simulate a motor disabled event
    waiting_for_response_.ClearAll();
    waiting_for_response_.Set(Actuator::ST_IDLE);
    UpdateDriveCtrlPanel(Actuator::ST_IDLE);
  }
}

void MainGUI::handleMotorStatusUpdate(const id_t& id,
                                      const grabec::GSWDriveInPdos& motor_status)
{
  // Check if signal corresponds to current selected axis
  if (id != ui->comboBox_motorAxis->currentText().toULong())
    return;

  UpdateDriveStatusTable(motor_status);

  Actuator::States actuator_state = Actuator::DriveState2ActuatorState(
    grabec::GoldSoloWhistleDrive::GetDriveState(motor_status.status_word));

  // Check if motor is in fault and set panel accordingly
  if (actuator_state == Actuator::ST_FAULT)
  {
    ui->pushButton_enable->setText(tr("Enable"));
    ui->pushButton_enable->setDisabled(true);
    ui->pushButton_faultReset->setEnabled(true);
    ui->pushButton_homing->setDisabled(true);
    ui->pushButton_calib->setDisabled(true);
    ui->groupBox_app->setDisabled(true);
    DisablePosCtrlButtons(true);
    DisableVelCtrlButtons(true);
    DisableTorqueCtrlButtons(true);
    waiting_for_response_.ClearAll();
    waiting_for_response_.Set(Actuator::ST_IDLE);
    return;
  }

  UpdateDriveCtrlPanel(actuator_state);
  UpdateDriveCtrlButtons(DriveOpMode2CtrlMode(motor_status.display_op_mode));
}

//--------- Private GUI methods ------------------------------------------------------//

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
  ui->horizontalSlider_speed_ctrl->setDisabled(value);
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

void MainGUI::UpdateDriveStatusTable(const grabec::GSWDriveInPdos& status)
{
  std::string status_word =
    grabec::GoldSoloWhistleDrive::GetDriveStateStr(status.status_word);
  ui->table_inputPdos->item(0, 0)->setData(Qt::DisplayRole, status_word.c_str());
  std::string op_mode_str;
  switch (status.display_op_mode)
  {
    case grabec::NONE:
      op_mode_str = "NONE";
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

void MainGUI::UpdateDriveCtrlPanel(const Actuator::States state)
{
  if (waiting_for_response_.CheckBit(Actuator::ST_ENABLED) &&
      state != Actuator::ST_ENABLED)
    return;
  if (waiting_for_response_.CheckBit(Actuator::ST_IDLE) && state != Actuator::ST_IDLE)
    return;

  manual_ctrl_enabled_ = (state == Actuator::ST_ENABLED);

  ui->pushButton_enable->setEnabled(true);
  ui->pushButton_enable->setText(tr(manual_ctrl_enabled_ ? "Disable" : "Enable"));
  ui->pushButton_faultReset->setDisabled(true);
  ui->comboBox_motorAxis->setDisabled(
    manual_ctrl_enabled_); // prevent switching drive during manual control

  ui->pushButton_homing->setDisabled(manual_ctrl_enabled_);
  ui->pushButton_calib->setDisabled(manual_ctrl_enabled_);
  ui->groupBox_app->setDisabled(true); // after we move we need to do the homing again

  if (!manual_ctrl_enabled_)
  {
    DisablePosCtrlButtons(true);
    DisableVelCtrlButtons(true);
    DisableTorqueCtrlButtons(true);
    waiting_for_response_.ClearAll();
  }
}

void MainGUI::UpdateDriveCtrlButtons(const ControlMode ctrl_mode)
{
  if (!waiting_for_response_.AnyOn())
    return;

  if (desired_ctrl_mode_.CheckBit(ctrl_mode))
  {
    DisablePosCtrlButtons(ctrl_mode != ControlMode::CABLE_LENGTH);
    DisableVelCtrlButtons(ctrl_mode != ControlMode::MOTOR_SPEED);
    DisableTorqueCtrlButtons(ctrl_mode != ControlMode::MOTOR_TORQUE);
    waiting_for_response_.ClearAll();
  }
  else
  {
    DisablePosCtrlButtons(true);
    DisableVelCtrlButtons(true);
    DisableTorqueCtrlButtons(true);
  }
}

bool MainGUI::ExitReadyStateRequest()
{
  QMessageBox::StandardButton reply =
    QMessageBox::question(this, "Robot ready",
                          "If you proceed with direct manual motor control all homing "
                          "results will be lost and you will have to repeat the "
                          "procedure before starting a new application.\nAre you sure "
                          "you want to continue?",
                          QMessageBox::Yes | QMessageBox::No);
  CLOG(TRACE, "event") << "--> " << (reply == QMessageBox::Yes);
  return (reply == QMessageBox::Yes);
}

//--------- Private methods ----------------------------------------------------------//

void MainGUI::SetupDirectMotorCtrl(const bool enable)
{
  robot_ptr_->stop(); // robot: READY | ENABLED --> ENABLED

  if (enable)
  {
    motor_id_ = ui->comboBox_motorAxis->currentText().toUInt();
    robot_ptr_->UpdateHomeConfig(motor_id_, 0.0, 0.0); // (re-)initialize reference values
    // Setup controller before enabling the motor
    man_ctrl_ptr_ =
      new ControllerSingleDrive(motor_id_, robot_ptr_->GetRtCycleTimeNsec());
    ActuatorStatus current_status = robot_ptr_->GetActuatorStatus(motor_id_);
    if (ui->radioButton_posMode->isChecked())
    {
      man_ctrl_ptr_->SetMode(ControlMode::CABLE_LENGTH);
      man_ctrl_ptr_->SetCableLenTarget(current_status.cable_length);
    }
    if (ui->radioButton_velMode->isChecked())
    {
      man_ctrl_ptr_->SetMode(ControlMode::MOTOR_SPEED);
      man_ctrl_ptr_->SetMotorSpeedTarget(0);
    }
    if (ui->radioButton_torqueMode->isChecked())
    {
      man_ctrl_ptr_->SetMode(ControlMode::MOTOR_TORQUE);
      man_ctrl_ptr_->SetMotorTorqueTarget(current_status.motor_torque);
    }
    robot_ptr_->SetController(man_ctrl_ptr_);

    robot_ptr_->EnableMotor(motor_id_);
  }
  else
  {
    robot_ptr_->SetController(NULL);
    delete man_ctrl_ptr_;

    robot_ptr_->DisableMotor(motor_id_);
  }
}

ControlMode MainGUI::DriveOpMode2CtrlMode(const int8_t drive_op_mode)
{
  switch (drive_op_mode)
  {
    case grabec::CYCLIC_POSITION:
      return ControlMode::CABLE_LENGTH;
    case grabec::CYCLIC_VELOCITY:
      return ControlMode::MOTOR_SPEED;
    case grabec::CYCLIC_TORQUE:
      return ControlMode::MOTOR_TORQUE;
    default:
      return ControlMode::NONE;
  }
}

void MainGUI::StartRobot()
{
  robot_ptr_ = new CableRobot(this, config_params_);

  connect(robot_ptr_, SIGNAL(printToQConsole(QString)), this,
          SLOT(appendText2Browser(QString)), Qt::ConnectionType::QueuedConnection);
  connect(robot_ptr_, SIGNAL(motorStatus(id_t, grabec::GSWDriveInPdos)), this,
          SLOT(handleMotorStatusUpdate(id_t, grabec::GSWDriveInPdos)));
  connect(robot_ptr_, SIGNAL(ecStateChanged(Bitfield8)), this,
          SLOT(updateEcStatusLED(Bitfield8)), Qt::ConnectionType::QueuedConnection);
  connect(robot_ptr_, SIGNAL(rtThreadStatusChanged(bool)), this,
          SLOT(updateRtThreadStatusLED(bool)), Qt::ConnectionType::QueuedConnection);

  robot_ptr_->eventSuccess(); // pwd & config OK --> robot ENABLED
  if (robot_ptr_->GetCurrentState() == CableRobot::ST_ENABLED)
    robot_ptr_->Start(); // start rt thread (ec master)
}

void MainGUI::DeleteRobot()
{
  if (robot_ptr_ == NULL)
    return;

  // We don't disconnect printToQConsole so we still have logs on shutdown
  disconnect(robot_ptr_, SIGNAL(motorStatus(id_t, grabec::GSWDriveInPdos)), this,
             SLOT(handleMotorStatusUpdate(id_t, grabec::GSWDriveInPdos)));
  disconnect(robot_ptr_, SIGNAL(ecStateChanged(Bitfield8)), this,
             SLOT(updateEcStatusLED(Bitfield8)));
  disconnect(robot_ptr_, SIGNAL(rtThreadStatusChanged(bool)), this,
             SLOT(updateRtThreadStatusLED(bool)));

  delete robot_ptr_;
  robot_ptr_ = NULL;
  QCoreApplication::processEvents();
  CLOG(INFO, "event") << "Cable robot object deleted";
}

void MainGUI::CloseAllApps()
{
  if (calib_dialog_ != NULL)
  {
    disconnect(calib_dialog_, SIGNAL(calibrationEnd()), robot_ptr_, SLOT(eventSuccess()));
    disconnect(calib_dialog_, SIGNAL(enableMainGUI()), this, SLOT(enableInterface()));
    calib_dialog_->close();
    delete calib_dialog_;
  }

  if (homing_dialog_ != NULL)
  {
    disconnect(homing_dialog_, SIGNAL(homingSuccess()), robot_ptr_, SLOT(eventSuccess()));
    disconnect(homing_dialog_, SIGNAL(homingFailed()), robot_ptr_, SLOT(eventFailure()));
    disconnect(homing_dialog_, SIGNAL(enableMainGUI(bool)), this,
               SLOT(enableInterface(bool)));
    homing_dialog_->close();
    delete homing_dialog_;
  }
}
