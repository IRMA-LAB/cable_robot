/**
 * @file main_gui.cpp
 * @author Simone Comari
 * @date 10 Jan 2020
 * @brief This file includes definitions of classes present in main_gui.h.
 */

#include "gui/main_gui.h"
#include "ui_main_gui.h"

MainGUI::MainGUI(QWidget* parent, const grabcdpr::RobotParams& robot_config)
  : QDialog(parent), ui(new Ui::MainGUI), robot_params_(robot_config)
{
  ui->setupUi(this);

  // Setup flags
  waiting_for_response_.reset();
  desired_ctrl_mode_.reset();
  desired_ctrl_mode_.set(ControlMode::CABLE_LENGTH);

  for (size_t i = 0; i < robot_config.actuators.size(); i++)
    if (robot_config.actuators[i].active)
      ui->comboBox_motorAxis->addItem(QString::number(i));

  StartRobot(); // instantiate cable robot object

#if DEBUG_GUI == 1
  pushButton_debug = new QPushButton("Debug", this);
  pushButton_debug->setMinimumSize(ui->pushButton_calib->size());
  pushButton_debug->setFont(ui->pushButton_calib->font());
  ui->verticalLayout_main->insertWidget(9, pushButton_debug, 0);
  ui->verticalSpacer_4->changeSize(0, 13, QSizePolicy::Policy::Expanding,
                                   QSizePolicy::Policy::MinimumExpanding);
  verticalSpacer_5 =
    new QSpacerItem(20, 0, QSizePolicy::Minimum, QSizePolicy::MinimumExpanding);
  ui->verticalLayout_main->insertItem(10, verticalSpacer_5);
  connect(pushButton_debug, SIGNAL(clicked()), this, SLOT(on_pushButton_debug_clicked()));
  ui->groupBox_app->setEnabled(true);
#endif
}

MainGUI::MainGUI(QWidget* parent, const grabcdpr::RobotParams& robot_config,
                 const SensorsParams& sensors_config)
  : MainGUI(parent, robot_config)
{
  sensors_params_ = sensors_config;
}

MainGUI::~MainGUI()
{
  DeleteRobot();
#if DEBUG_GUI == 1
  disconnect(pushButton_debug, SIGNAL(clicked()), this,
             SLOT(on_pushButton_debug_clicked()));
  if (debug_app_ != nullptr)
  {
    disconnect(debug_app_, SIGNAL(debugCompleted()), this, SLOT(handleDebugCompleted()));
    delete debug_app_;
  }
#endif
  delete ui;
  CLOG(INFO, "event") << "Main window closed";
}

//--------- Public GUI slots ---------------------------------------------------------//

void MainGUI::closeEvent(QCloseEvent* event)
{
  CloseAllApps();
  event->accept();
}

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

  calib_dialog_ = new CalibrationDialog(this, robot_ptr_, robot_params_);
  connect(calib_dialog_, SIGNAL(enableMainGUI(bool)), this, SLOT(enableInterface(bool)));
  connect(calib_dialog_, SIGNAL(calibrationEnd()), robot_ptr_, SLOT(eventSuccess()));
  calib_dialog_->show();
  CLOG(INFO, "event") << "Prompt calibration dialog";
}

void MainGUI::on_pushButton_homing_clicked()
{
  CLOG(TRACE, "event");
#if DEBUG_GUI == 0
  if (!(ec_network_valid_ && rt_thread_running_))
    return;
#endif

  ui->pushButton_homing->setDisabled(true);
  ui->pushButton_calib->setDisabled(true);
  ui->groupBox_app->setDisabled(true);
  ui->frame_manualControl->setDisabled(true);

  robot_ptr_->enterHomingMode();

  if (homing_dialog_ == nullptr)
  {
    homing_dialog_ = new HomingDialog(this, robot_ptr_, sensors_params_);
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
  CLOG(TRACE, "event") << ui->comboBox_apps->currentText();
#if DEBUG_GUI == 0
  if (!(ec_network_valid_ && rt_thread_running_))
    return;
#endif

  ui->pushButton_homing->setDisabled(true);
  ui->pushButton_calib->setDisabled(true);
  ui->groupBox_app->setDisabled(true);
  ui->frame_manualControl->setDisabled(true);

  if (ui->comboBox_apps->currentText() == "Joint PVT 33")
  {
    robot_ptr_->eventSuccess();
    joints_pvt_dialog_ = new JointsPVTDialog(this, robot_ptr_, robot_params_.actuators);
    connect(joints_pvt_dialog_, SIGNAL(destroyed()), robot_ptr_, SLOT(eventSuccess()));
    connect(joints_pvt_dialog_, SIGNAL(destroyed()), this, SLOT(enableInterface()));
    joints_pvt_dialog_->show();
    CLOG(INFO, "event") << "Prompt joints PVT dialog";
  }
  else if (ui->comboBox_apps->currentText() == "Manual Control")
  {
    robot_ptr_->eventSuccess();
    man_ctrl_dialog_ = new ManualControlDialog(this, robot_ptr_);
    connect(man_ctrl_dialog_, SIGNAL(destroyed()), robot_ptr_, SLOT(eventSuccess()));
    connect(man_ctrl_dialog_, SIGNAL(destroyed()), this, SLOT(enableInterface()));
    man_ctrl_dialog_->show();
    CLOG(INFO, "event") << "Prompt manual control dialog";
  }
}

#if DEBUG_GUI == 1
void MainGUI::on_pushButton_debug_clicked()
{
  CLOG(TRACE, "event");
  if (!(ec_network_valid_ && rt_thread_running_))
    return;

  pushButton_debug->setDisabled(true);

  // Insert debug operations/app here..
  if (debug_app_ == nullptr)
  {
    debug_app_ = new DebugClass(this, robot_ptr_);
    connect(debug_app_, SIGNAL(debugCompleted()), this, SLOT(handleDebugCompleted()));
    debug_app_->start();
  }
}

void MainGUI::handleDebugCompleted() { pushButton_debug->setEnabled(true); }
#endif

//--------- Public GUI slots of direct drive control panel --------------------------//

void MainGUI::on_pushButton_enable_clicked()
{
  CLOG(TRACE, "event");
  ui->pushButton_enable->setChecked(false);
  if (!(ec_network_valid_ && rt_thread_running_))
    return;

  bool manual_ctrl_enabled = !manual_ctrl_enabled_;

  SetupDirectMotorCtrl(manual_ctrl_enabled);
  waiting_for_response_.set(manual_ctrl_enabled ? Actuator::ST_ENABLED
                                                : Actuator::ST_IDLE);
}

void MainGUI::on_pushButton_freedrive_pressed()
{
  CLOG(TRACE, "event");
  if (!(ec_network_valid_ && rt_thread_running_))
    return;

  if (freedrive_)
  {
    robot_ptr_->SetController(nullptr);
    delete man_ctrl_ptr_;
    if (robot_ptr_->GetCurrentState() != CableRobot::ST_READY)
      robot_ptr_->DisableMotors();
    freedrive_ = false;
    return;
  }

  ui->pushButton_freedrive->setChecked(true);
  if (robot_ptr_->GetCurrentState() != CableRobot::ST_READY)
  {
    // Attempt to enable all motors
    robot_ptr_->EnableMotors();
    grabrt::ThreadClock clock(grabrt::Sec2NanoSec(CableRobot::kCycleWaitTimeSec));
    while (1)
    {
      if (robot_ptr_->MotorsEnabled())
        break;
      if (clock.ElapsedFromStart() > CableRobot::kMaxWaitTimeSec)
      {
        appendText2Browser("WARNING: Taking too long to enable freedrive mode");
        return;
      }
      clock.WaitUntilNext();
    }
  }

  // Set all motors in torque control mode
  man_ctrl_ptr_ = new ControllerSingleDrive(motor_id_, robot_ptr_->GetRtCycleTimeNsec());
  man_ctrl_ptr_->SetMotorTorqueSsErrTol(kTorqueSsErrTol_);
  robot_ptr_->SetController(man_ctrl_ptr_);
  for (const id_t id : robot_ptr_->GetActiveMotorsID())
  {
    pthread_mutex_lock(&robot_ptr_->Mutex());
    man_ctrl_ptr_->SetMotorID(id);
    man_ctrl_ptr_->SetMode(ControlMode::MOTOR_TORQUE);
    man_ctrl_ptr_->SetMotorTorqueTarget(kFreedriveTorque_);
    pthread_mutex_unlock(&robot_ptr_->Mutex());
    // Wait until each motor reached user-given initial torque setpoint
    if (robot_ptr_->WaitUntilTargetReached() != RetVal::OK)
    {
      appendText2Browser(
        QString("WARNING: Could not switch motor %1 to torque control mode").arg(id));
      break;
    }
  }
  appendText2Browser("Freedrive mode ACTIVATED\nYou can now manually move the platform");
  freedrive_ = true;
}

void MainGUI::on_pushButton_faultReset_clicked()
{
  CLOG(TRACE, "event");
  robot_ptr_->ClearFaults();
}

void MainGUI::on_pushButton_exitReady_clicked()
{
  CLOG(TRACE, "event");
  if (robot_ptr_->GetCurrentState() != CableRobot::ST_READY)
    return;
  if (!ExitReadyStateRequest())
    return;
  robot_ptr_->DisableMotors();
  robot_ptr_->stop();
  ui->pushButton_exitReady->setDisabled(true);
  enableInterface(false);
}

void MainGUI::on_radioButton_posMode_clicked()
{
  CLOG(TRACE, "event");
  ui->radioButton_posMode->setChecked(true);
  ui->radioButton_velMode->setChecked(false);
  ui->radioButton_torqueMode->setChecked(false);

  if (desired_ctrl_mode_.test(ControlMode::CABLE_LENGTH))
    return;
  desired_ctrl_mode_.reset();
  desired_ctrl_mode_.set(ControlMode::CABLE_LENGTH);

  if (manual_ctrl_enabled_)
  {
    man_ctrl_ptr_->SetCableLenTarget(
      robot_ptr_->GetActuatorStatus(motor_id_).cable_length);
    pthread_mutex_lock(&robot_ptr_->Mutex());
    man_ctrl_ptr_->SetMode(ControlMode::CABLE_LENGTH);
    pthread_mutex_unlock(&robot_ptr_->Mutex());
    waiting_for_response_.set(Actuator::ST_ENABLED);
  }
}

void MainGUI::on_radioButton_velMode_clicked()
{
  CLOG(TRACE, "event");
  ui->radioButton_posMode->setChecked(false);
  ui->radioButton_velMode->setChecked(true);
  ui->radioButton_torqueMode->setChecked(false);

  if (desired_ctrl_mode_.test(ControlMode::MOTOR_SPEED))
    return;
  desired_ctrl_mode_.reset();
  desired_ctrl_mode_.set(ControlMode::MOTOR_SPEED);

  if (manual_ctrl_enabled_)
  {
    man_ctrl_ptr_->SetMotorSpeedTarget(0);
    pthread_mutex_lock(&robot_ptr_->Mutex());
    man_ctrl_ptr_->SetMode(ControlMode::MOTOR_SPEED);
    pthread_mutex_unlock(&robot_ptr_->Mutex());
    waiting_for_response_.set(Actuator::ST_ENABLED);
  }
}

void MainGUI::on_radioButton_torqueMode_clicked()
{
  CLOG(TRACE, "event");
  ui->radioButton_posMode->setChecked(false);
  ui->radioButton_velMode->setChecked(false);
  ui->radioButton_torqueMode->setChecked(true);

  if (desired_ctrl_mode_.test(ControlMode::MOTOR_TORQUE))
    return;
  desired_ctrl_mode_.reset();
  desired_ctrl_mode_.set(ControlMode::MOTOR_TORQUE);

  if (manual_ctrl_enabled_)
  {
    man_ctrl_ptr_->SetMotorTorqueTarget(
      robot_ptr_->GetActuatorStatus(motor_id_).motor_torque);
    pthread_mutex_lock(&robot_ptr_->Mutex());
    man_ctrl_ptr_->SetMode(ControlMode::MOTOR_TORQUE);
    pthread_mutex_unlock(&robot_ptr_->Mutex());
    waiting_for_response_.set(Actuator::ST_ENABLED);
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

void MainGUI::enableInterface(const bool op_outcome /*= true*/)
{
  ui->pushButton_homing->setEnabled(true);
  ui->pushButton_calib->setEnabled(true);
  ui->groupBox_app->setEnabled(op_outcome);
  ui->frame_manualControl->setEnabled(true);
  ui->pushButton_exitReady->setEnabled(op_outcome);
  // Reset app pointers as they just delete themselves.
  joints_pvt_dialog_ = nullptr;
  man_ctrl_dialog_   = nullptr;
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

void MainGUI::updateEcStatusLED(const std::bitset<3>& ec_status_flags)
{
  switch (ec_status_flags.count())
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
    waiting_for_response_.reset();
    waiting_for_response_.set(Actuator::ST_IDLE);
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
    ui->pushButton_exitReady->setDisabled(true);
    ui->pushButton_homing->setDisabled(true);
    ui->pushButton_calib->setDisabled(true);
    ui->groupBox_app->setDisabled(true);
    DisablePosCtrlButtons(true);
    DisableVelCtrlButtons(true);
    DisableTorqueCtrlButtons(true);
    waiting_for_response_.reset();
    waiting_for_response_.set(Actuator::ST_IDLE);
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
  if (waiting_for_response_.test(Actuator::ST_ENABLED) && state != Actuator::ST_ENABLED)
    return;
  if (waiting_for_response_.test(Actuator::ST_IDLE) && state != Actuator::ST_IDLE)
    return;

  bool robot_ready     = robot_ptr_->GetCurrentState() == CableRobot::ST_READY;
  manual_ctrl_enabled_ = (state == Actuator::ST_ENABLED) && !(robot_ready || freedrive_);

  ui->pushButton_enable->setEnabled(!(robot_ready || freedrive_));
  ui->pushButton_enable->setText(
    tr(state == Actuator::ST_ENABLED ? "Disable" : "Enable"));
  ui->pushButton_freedrive->setDisabled(manual_ctrl_enabled_);
  ui->pushButton_freedrive->setText(freedrive_ ? "Exit Freedrive" : "Freedrive");
  ui->pushButton_faultReset->setDisabled(true);
  ui->comboBox_motorAxis->setDisabled(
    manual_ctrl_enabled_); // prevent switching drive during manual control

  ui->pushButton_homing->setDisabled(manual_ctrl_enabled_ || freedrive_);
  ui->pushButton_calib->setDisabled(manual_ctrl_enabled_ || freedrive_);

  if (!manual_ctrl_enabled_)
  {
    DisablePosCtrlButtons(true);
    DisableVelCtrlButtons(true);
    DisableTorqueCtrlButtons(true);
    waiting_for_response_.reset();
  }
}

void MainGUI::UpdateDriveCtrlButtons(const ControlMode ctrl_mode)
{
  if (!waiting_for_response_.any())
    return;

  if (desired_ctrl_mode_.test(ctrl_mode))
  {
    DisablePosCtrlButtons(ctrl_mode != ControlMode::CABLE_LENGTH);
    DisableVelCtrlButtons(ctrl_mode != ControlMode::MOTOR_SPEED);
    DisableTorqueCtrlButtons(ctrl_mode != ControlMode::MOTOR_TORQUE);
    waiting_for_response_.reset();
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
  QMessageBox::StandardButton reply = QMessageBox::question(
    this, "Robot ready",
    "If you proceed with direct manual motor control motors will be "
    "disabled and you will have to repeat the homing procedure "
    "before starting a new application.\nAre you sure you want to continue?",
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
    man_ctrl_ptr_->SetMotorTorqueSsErrTol(kTorqueSsErrTol_);
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
    robot_ptr_->SetController(nullptr);
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
  robot_ptr_ = new CableRobot(this, robot_params_);

  connect(robot_ptr_, SIGNAL(printToQConsole(QString)), this,
          SLOT(appendText2Browser(QString)), Qt::ConnectionType::QueuedConnection);
  connect(robot_ptr_, SIGNAL(motorStatus(id_t, grabec::GSWDriveInPdos)), this,
          SLOT(handleMotorStatusUpdate(id_t, grabec::GSWDriveInPdos)));
  connect(robot_ptr_, SIGNAL(ecStateChanged(std::bitset<3>)), this,
          SLOT(updateEcStatusLED(std::bitset<3>)), Qt::ConnectionType::QueuedConnection);
  connect(robot_ptr_, SIGNAL(rtThreadStatusChanged(bool)), this,
          SLOT(updateRtThreadStatusLED(bool)), Qt::ConnectionType::QueuedConnection);

  robot_ptr_->eventSuccess(); // pwd & config OK --> robot ENABLED
  if (robot_ptr_->GetCurrentState() == CableRobot::ST_ENABLED)
    robot_ptr_->Start(); // start rt thread (ec master)
}

void MainGUI::DeleteRobot()
{
  if (robot_ptr_ == nullptr)
    return;

  // We don't disconnect printToQConsole so we still have logs on shutdown
  disconnect(robot_ptr_, SIGNAL(motorStatus(id_t, grabec::GSWDriveInPdos)), this,
             SLOT(handleMotorStatusUpdate(id_t, grabec::GSWDriveInPdos)));
  disconnect(robot_ptr_, SIGNAL(ecStateChanged(std::bitset<3>)), this,
             SLOT(updateEcStatusLED(std::bitset<3>)));
  disconnect(robot_ptr_, SIGNAL(rtThreadStatusChanged(bool)), this,
             SLOT(updateRtThreadStatusLED(bool)));

  delete robot_ptr_;
  robot_ptr_ = nullptr;
  QCoreApplication::processEvents();
  CLOG(INFO, "event") << "Cable robot object deleted";
}

void MainGUI::CloseAllApps()
{
  if (calib_dialog_ != nullptr)
  {
    disconnect(calib_dialog_, SIGNAL(calibrationEnd()), robot_ptr_, SLOT(eventSuccess()));
    disconnect(calib_dialog_, SIGNAL(enableMainGUI(bool)), this,
               SLOT(enableInterface(bool)));
    calib_dialog_->close();
    delete calib_dialog_;
  }

  if (homing_dialog_ != nullptr)
  {
    disconnect(homing_dialog_, SIGNAL(homingSuccess()), robot_ptr_, SLOT(eventSuccess()));
    disconnect(homing_dialog_, SIGNAL(homingFailed()), robot_ptr_, SLOT(eventFailure()));
    disconnect(homing_dialog_, SIGNAL(enableMainGUI(bool)), this,
               SLOT(enableInterface(bool)));
    homing_dialog_->close();
    delete homing_dialog_;
  }

  if (joints_pvt_dialog_ != nullptr && joints_pvt_dialog_->isVisible())
  {
    disconnect(joints_pvt_dialog_, SIGNAL(destroyed()), robot_ptr_, SLOT(eventSuccess()));
    disconnect(joints_pvt_dialog_, SIGNAL(destroyed()), this, SLOT(enableInterface()));
    joints_pvt_dialog_->close();
  }

  if (man_ctrl_dialog_ != nullptr && man_ctrl_dialog_->isVisible())
  {
    disconnect(man_ctrl_dialog_, SIGNAL(destroyed()), robot_ptr_, SLOT(eventSuccess()));
    disconnect(man_ctrl_dialog_, SIGNAL(destroyed()), this, SLOT(enableInterface()));
    man_ctrl_dialog_->close();
  }
}
