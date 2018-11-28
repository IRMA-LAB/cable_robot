#include "gui/main_gui.h"
#include "ui_main_gui.h"

MainGUI::MainGUI(QWidget* parent, const grabcdpr::Params& config)
  : QDialog(parent), ui(new Ui::MainGUI), robot_(this, config)
{
  ui->setupUi(this);

  for (size_t i = 0; i < config.actuators.size(); i++)
    ui->comboBox_motorAxis->addItem(QString::number(i + 1));

  connect(&robot_, SIGNAL(printToQConsole(QString)), this,
          SLOT(AppendText2Browser(QString)));
  connect(&robot_, SIGNAL(motorStatus(quint8, grabec::GSWDriveInPdos)), this,
          SLOT(UpdateDriveStatusTable(quint8, grabec::GSWDriveInPdos)));

  robot_.EventSuccess(); // pwd & config OK --> robot ENABLED
#if ECNTW
  if (robot_.GetCurrentState() == CableRobot::ST_ENABLED)
    robot_.Start();
#endif
}

MainGUI::~MainGUI()
{
  disconnect(&robot_, SIGNAL(printToQConsole(QString)), this,
             SLOT(AppendText2Browser(QString)));
  disconnect(&robot_, SIGNAL(motorStatus(quint8, grabec::GSWDriveInPdos)), this,
             SLOT(UpdateDriveStatusTable(quint8, grabec::GSWDriveInPdos)));

  if (calib_dialog_ != NULL)
  {
    disconnect(calib_dialog_, SIGNAL(enableMainGUI()), this, SLOT(EnableInterface()));
    disconnect(calib_dialog_, SIGNAL(calibrationEnd()), &robot_, SLOT(EventSuccess()));
    delete calib_dialog_;
  }
  if (homing_dialog_ != NULL)
  {
    disconnect(homing_dialog_, SIGNAL(enableMainGUI(bool)), this,
               SLOT(EnableInterface(bool)));
    disconnect(homing_dialog_, SIGNAL(homingSuccess()), &robot_, SLOT(EventSuccess()));
    disconnect(homing_dialog_, SIGNAL(homingFailed()), &robot_, SLOT(EventFailure()));
    delete homing_dialog_;
  }
  delete ui;
  CLOG(INFO, "event") << "Main window closed";
}

////////////////////////////////////////
/// SLOTS GUI
////////////////////////////////////////

void MainGUI::on_pushButton_calib_clicked()
{
  CLOG(TRACE, "event");
  if (robot_.GetCurrentState() == CableRobot::ST_READY)
    if (!ExitReadyStateRequest())
      return;

  ui->pushButton_homing->setDisabled(true);
  ui->pushButton_calib->setDisabled(true);
  ui->groupBox_app->setDisabled(true);
  ui->frame_manualControl->setDisabled(true);

  robot_.EnterCalibrationMode();

  calib_dialog_ = new CalibrationDialog(this, &robot_);
  connect(calib_dialog_, SIGNAL(enableMainGUI()), this, SLOT(EnableInterface()));
  connect(calib_dialog_, SIGNAL(calibrationEnd()), &robot_, SLOT(EventSuccess()));
  calib_dialog_->show();
  CLOG(INFO, "event") << "Prompt calibration dialog";
}

void MainGUI::on_pushButton_homing_clicked()
{
  CLOG(TRACE, "event");
  ui->pushButton_homing->setDisabled(true);
  ui->pushButton_calib->setDisabled(true);
  ui->groupBox_app->setDisabled(true);
  ui->frame_manualControl->setDisabled(true);

  robot_.EnterHomingMode();

  if (homing_dialog_ == NULL)
  {
    homing_dialog_ = new HomingDialog(this, &robot_);
    connect(homing_dialog_, SIGNAL(enableMainGUI(bool)), this,
            SLOT(EnableInterface(bool)));
    connect(homing_dialog_, SIGNAL(homingSuccess()), &robot_, SLOT(EventSuccess()));
    connect(homing_dialog_, SIGNAL(homingFailed()), &robot_, SLOT(EventFailure()));
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
  if (robot_.GetCurrentState() == CableRobot::ST_READY)
    if (!ExitReadyStateRequest())
      return;

  manual_ctrl_enabled_ = !manual_ctrl_enabled_;

  if (manual_ctrl_enabled_)
  {
    motor_id_ = static_cast<uint8_t>(ui->comboBox_motorAxis->currentIndex());
    ui->pushButton_enable->setText(tr("Disable"));
  }
  else
    ui->pushButton_enable->setText(tr("Enable"));

  ui->pushButton_homing->setDisabled(manual_ctrl_enabled_);
  ui->pushButton_calib->setDisabled(manual_ctrl_enabled_);
  ui->groupBox_app->setDisabled(true); // after we move we need to do the homing again

  DisablePosCtrlButtons(!(manual_ctrl_enabled_ && ui->radioButton_posMode->isChecked()));
  DisableVelCtrlButtons(!(manual_ctrl_enabled_ && ui->radioButton_velMode->isChecked()));
  DisableTorqueCtrlButtons(
    !(manual_ctrl_enabled_ && ui->radioButton_torqueMode->isChecked()));

  SetupDirectMotorCtrl();
}

void MainGUI::on_pushButton_faultReset_clicked() { CLOG(TRACE, "event"); }

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
#if ECNTW
    man_ctrl_ptr_->SetCableLenTarget(robot_.GetActuatorStatus(motor_id_).cable_length);
    man_ctrl_ptr_->SetMode(ControlMode::CABLE_LENGTH);
#endif
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
#if ECNTW
    man_ctrl_ptr_->SetMotorSpeedTarget(robot_.GetActuatorStatus(motor_id_).motor_speed);
    man_ctrl_ptr_->SetMode(ControlMode::MOTOR_SPEED);
#endif
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
#if ECNTW
    man_ctrl_ptr_->SetMotorTorqueTarget(robot_.GetActuatorStatus(motor_id_).motor_torque);
    man_ctrl_ptr_->SetMode(ControlMode::MOTOR_TORQUE);
#endif
  }
}

#if ECNTW
void MainGUI::on_pushButton_posPlus_pressed()
{
  CLOG(TRACE, "event");
  man_ctrl_ptr_->CableLenIncrement(true, Sign::POS, false);
}

void MainGUI::on_pushButton_posPlus_released()
{
  CLOG(TRACE, "event");
  man_ctrl_ptr_->CableLenIncrement(false);
}

void MainGUI::on_pushButton_posMinus_pressed()
{
  CLOG(TRACE, "event");
  man_ctrl_ptr_->CableLenIncrement(true, Sign::NEG, false);
}

void MainGUI::on_pushButton_posMinus_released()
{
  CLOG(TRACE, "event");
  man_ctrl_ptr_->CableLenIncrement(false);
}

void MainGUI::on_pushButton_posMicroPlus_pressed()
{
  CLOG(TRACE, "event");
  man_ctrl_ptr_->CableLenIncrement(true, Sign::POS, true);
}

void MainGUI::on_pushButton_posMicroPlus_released()
{
  CLOG(TRACE, "event");
  man_ctrl_ptr_->CableLenIncrement(false);
}

void MainGUI::on_pushButton_posMicroMinus_pressed()
{
  CLOG(TRACE, "event");
  man_ctrl_ptr_->CableLenIncrement(true, Sign::NEG, true);
}

void MainGUI::on_pushButton_posMicroMinus_released()
{
  CLOG(TRACE, "event");
  man_ctrl_ptr_->CableLenIncrement(false);
}

void MainGUI::on_pushButton_speedPlus_clicked()
{
  CLOG(TRACE, "event");
  man_ctrl_ptr_->MotorSpeedIncrement(Sign::POS);
}

void MainGUI::on_pushButton_speedMinus_clicked()
{
  CLOG(TRACE, "event");
  man_ctrl_ptr_->MotorSpeedIncrement(Sign::NEG);
}

void MainGUI::on_pushButton_torquePlus_clicked()
{
  CLOG(TRACE, "event");
  man_ctrl_ptr_->MotorTorqueIncrement(Sign::POS);
}

void MainGUI::on_pushButton_torqueMinus_clicked()
{
  CLOG(TRACE, "event");
  man_ctrl_ptr_->MotorTorqueIncrement(Sign::NEG);
}
#endif

////////////////////////////////////////
/// SLOTS
////////////////////////////////////////

void MainGUI::EnableInterface(const bool op_outcome /*= false*/)
{
  ui->pushButton_homing->setEnabled(true);
  ui->pushButton_calib->setEnabled(true);
  ui->groupBox_app->setEnabled(op_outcome);
  ui->frame_manualControl->setEnabled(true);
  CVLOG(2, "event") << "Interface enabled with app selection "
                    << (op_outcome ? "enabled" : "disabled");
}

void MainGUI::AppendText2Browser(const QString& text)
{
  if (text.contains("warning", Qt::CaseSensitivity::CaseInsensitive))
    CLOG(WARNING, "browser") << text;
  else if (text.contains("error", Qt::CaseSensitivity::CaseInsensitive))
    CLOG(ERROR, "browser") << text;
  else
    CLOG(INFO, "browser") << text;
  ui->textBrowser_logs->append(text);
}

void MainGUI::UpdateDriveStatusTable(const quint8 id,
                                     const grabec::GSWDriveInPdos& status)
{
  static const qint16 kUpdateCycle = 50; // ~ms
  static quint16 counter = 1;

  // Check signal corresponds to current selected axis
  if (id != static_cast<uint8_t>(ui->comboBox_motorAxis->currentIndex()))
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

////////////////////////////////////////
/// GUI private methods
////////////////////////////////////////

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

////////////////////////////////////////
/// Private methods
////////////////////////////////////////

void MainGUI::SetupDirectMotorCtrl()
{
  robot_.Stop(); // robot: READY | ENABLED --> ENABLED

#if ECNTW
  if (manual_ctrl_enabled_)
  {
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

    robot_.EnableMotors(std::vector<uint8_t>(1, motor_id_));
  }
  else
  {
    delete man_ctrl_ptr_;
    man_ctrl_ptr_ = NULL;
    robot_.SetController(man_ctrl_ptr_);

    robot_.DisableMotors(std::vector<uint8_t>(1, motor_id_));
  }
#endif
}
