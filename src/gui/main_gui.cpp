#include "gui/main_gui.h"
#include "ui_main_gui.h"

MainGUI::MainGUI(QWidget* parent, const grabcdpr::Params& config)
  : QDialog(parent), ui(new Ui::MainGUI), robot_(this, config)
{
  ui->setupUi(this);

  for (size_t i = 0; i < config.cables.size(); i++)
    ui->comboBox_motorAxis->addItem(QString(static_cast<char>(i + 1)));

  connect(&robot_, SIGNAL(printToQConsole(QString)), this,
          SLOT(AppendText2Browser(QString)));

  robot_.EventSuccess(); // pwd & config OK --> robot ENABLED
  if (robot_.GetCurrentState() == CableRobot::ST_ENABLED)
    robot_.Start();
}

MainGUI::~MainGUI()
{
  disconnect(&robot_, SIGNAL(printToQConsole(QString)), this,
             SLOT(AppendText2Browser(QString)));

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
}

////////////////////////////////////////
/// SLOTS GUI
////////////////////////////////////////

void MainGUI::on_pushButton_calib_clicked()
{
  ui->pushButton_homing->setDisabled(true);
  ui->pushButton_calib->setDisabled(true);
  ui->groupBox_app->setDisabled(true);
  ui->frame_manualControl->setDisabled(true);

  robot_.EnterCalibrationMode();

  calib_dialog_ = new CalibrationDialog(this, &robot_);
  connect(calib_dialog_, SIGNAL(enableMainGUI()), this, SLOT(EnableInterface()));
  connect(calib_dialog_, SIGNAL(calibrationEnd()), &robot_, SLOT(EventSuccess()));
  calib_dialog_->show();
}

void MainGUI::on_pushButton_homing_clicked()
{
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
}

void MainGUI::on_pushButton_startApp_clicked()
{
  //  ui->pushButton_homing->setDisabled(true);
  //  ui->pushButton_calib->setDisabled(true);
  //  ui->groupBox_app->setDisabled(true);
  //  ui->frame_manualControl->setDisabled(true);
}

void MainGUI::on_pushButton_enable_clicked()
{
  if (robot_.GetCurrentState() == CableRobot::ST_READY)
  {
    QMessageBox::StandardButton reply = QMessageBox::question(
      this, "Robot ready", "If you proceed with direct manual motor control all homing "
                           "results will be lost and you will have to repeat the "
                           "procedure before starting a new application.\nAre you sure "
                           "you want to continue?",
      QMessageBox::Yes | QMessageBox::No);
    if (reply == QMessageBox::No)
      return;
  }

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

  SetupDirectDriveCtrl();
}

void MainGUI::on_pushButton_faultReset_clicked() {}

void MainGUI::on_radioButton_posMode_clicked()
{
  ui->radioButton_posMode->setChecked(true);
  ui->radioButton_velMode->setChecked(false);
  ui->radioButton_torqueMode->setChecked(false);

  if (manual_ctrl_enabled_)
  {
    DisablePosCtrlButtons(false);
    DisableVelCtrlButtons(true);
    DisableTorqueCtrlButtons(true);
  }
}

void MainGUI::on_radioButton_velMode_clicked()
{
  ui->radioButton_posMode->setChecked(false);
  ui->radioButton_velMode->setChecked(true);
  ui->radioButton_torqueMode->setChecked(false);

  if (manual_ctrl_enabled_)
  {
    DisablePosCtrlButtons(true);
    DisableVelCtrlButtons(false);
    DisableTorqueCtrlButtons(true);
  }
}

void MainGUI::on_radioButton_torqueMode_clicked()
{
  ui->radioButton_posMode->setChecked(false);
  ui->radioButton_velMode->setChecked(false);
  ui->radioButton_torqueMode->setChecked(true);

  if (manual_ctrl_enabled_)
  {
    DisablePosCtrlButtons(true);
    DisableVelCtrlButtons(true);
    DisableTorqueCtrlButtons(false);
  }
}

////////////////////////////////////////
/// SLOTS
////////////////////////////////////////

void MainGUI::EnableInterface(const bool op_outcome /*= false*/)
{
  ui->pushButton_homing->setEnabled(true);
  ui->pushButton_calib->setEnabled(true);
  ui->groupBox_app->setEnabled(op_outcome);
  ui->frame_manualControl->setEnabled(true);
}

void MainGUI::AppendText2Browser(const QString& text)
{
  ui->textBrowser_logs->append(text);
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
}

void MainGUI::DisableVelCtrlButtons(const bool value)
{
  ui->pushButton_speedMinus->setDisabled(value);
  ui->pushButton_speedPlus->setDisabled(value);
}

void MainGUI::DisableTorqueCtrlButtons(const bool value)
{
  ui->pushButton_torqueMinus->setDisabled(value);
  ui->pushButton_torquePlus->setDisabled(value);
}

////////////////////////////////////////
/// Private methods
////////////////////////////////////////

void MainGUI::SetupDirectDriveCtrl()
{
  robot_.Stop(); // robot: READY | ENABLED --> ENABLED

  if (manual_ctrl_enabled_)
  {
    // Setup controller before enabling the motor
    man_ctrl_ptr_ = new ControllerBasic(motor_id_);
    MotorStatus current_status = robot_.GetMotorStatus(motor_id_);
    if (ui->radioButton_posMode->isChecked())
    {
      man_ctrl_ptr_->SetMode(grabec::CYCLIC_POSITION);
      man_ctrl_ptr_->SetCableLenTarget(current_status.length_target);
    }
    if (ui->radioButton_velMode->isChecked())
    {
      man_ctrl_ptr_->SetMode(grabec::CYCLIC_VELOCITY);
      man_ctrl_ptr_->SetMotorSpeedTarget(current_status.speed_target);
    }
    if (ui->radioButton_torqueMode->isChecked())
    {
      man_ctrl_ptr_->SetMode(grabec::CYCLIC_TORQUE);
      man_ctrl_ptr_->SetMotorTorqueTarget(current_status.torque_target);
    }
    robot_.SetController(man_ctrl_ptr_);

    robot_.EnableMotors(std::vector<uint8_t>(1, motor_id_));
  }
  else
  {
    delete man_ctrl_ptr_;
    robot_.DisableMotors(std::vector<uint8_t>(1, motor_id_));
  }
}
