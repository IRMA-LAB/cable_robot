#include "gui/homing/homing_interface_proprioceptive.h"
#include "ui_homing_interface_proprioceptive.h"

HomingInterfaceProprioceptive::HomingInterfaceProprioceptive(
  QWidget* parent, const grabcdpr::Params* config)
  : HomingInterface(parent, config), ui(new Ui::HomingInterfaceProprioceptive),
    app_(this, config)
{
  ui->setupUi(this);

  connect(&app_, SIGNAL(printToQConsole(QString)), this,
          SLOT(AppendText2Browser(QString)));
  connect(&app_, SIGNAL(acquisitionComplete()), this, SLOT(AcquisitionCompleteCb()));
}

HomingInterfaceProprioceptive::~HomingInterfaceProprioceptive()
{
  disconnect(&app_, SIGNAL(printToQConsole(QString)), this,
             SLOT(AppendText2Browser(QString)));
  disconnect(&app_, SIGNAL(acquisitionComplete()), this, SLOT(AcquisitionCompleteCb()));
  delete ui;
}

///////////////////////////
//// Public slots
///////////////////////////

void HomingInterfaceProprioceptive::FaultPresent(const bool value)
{
  ui->pushButton_clearFaults->setEnabled(value);
  ui->pushButton_enable->setEnabled(!value);
  ui->pushButton_start->setEnabled(!value);
  ui->pushButton_acquire->setEnabled(!value);

  if (app_.IsCollectingData())
    ui->pushButton_enable->setText(tr("Enable"));

  if (value)
    app_.FaultTrigger();
}

void HomingInterfaceProprioceptive::AcquisitionCompleteCb()
{
  ui->radioButton_internal->setEnabled(true);
  ui->pushButton_start->click(); // stop
}

///////////////////////////
//// Private slots
///////////////////////////

void HomingInterfaceProprioceptive::on_pushButton_enable_clicked()
{
  if (app_.IsCollectingData())
  {
    ui->pushButton_start->click(); // stop
    // robot.Disable();
    ui->pushButton_enable->setText(tr("Enable"));
    ui->pushButton_start->setDisabled(true);
  }
  else
  {
    // robot.Enable();
    ui->pushButton_enable->setText(tr("Disable"));
    ui->pushButton_start->setEnabled(true);
  }
}

void HomingInterfaceProprioceptive::on_pushButton_clearFaults_clicked()
{
  app_.FaultReset();
}

void HomingInterfaceProprioceptive::on_pushButton_start_clicked()
{
  if (app_.IsCollectingData())
  {
    app_.Stop();
    ui->pushButton_start->setText(tr("Start"));
    ui->pushButton_acquire->setDisabled(true);
  }
  else
  {
    app_.Start();
    ui->pushButton_start->setText(tr("Stop"));
    ui->pushButton_acquire->setEnabled(true);
  }
}

void HomingInterfaceProprioceptive::on_pushButton_acquire_clicked() { app_.Next(); }

void HomingInterfaceProprioceptive::on_radioButton_internal_clicked()
{
  ui->radioButton_external->toggled(false);
  ui->lineEdit->setDisabled(true);
  ui->pushButton_extFile->setDisabled(true);
}

void HomingInterfaceProprioceptive::on_radioButton_external_clicked()
{
  ui->radioButton_internal->toggled(false);
  ui->lineEdit->setEnabled(true);
  ui->pushButton_extFile->setEnabled(true);
}

void HomingInterfaceProprioceptive::on_pushButton_extFile_clicked()
{
  QString config_filename =
    QFileDialog::getOpenFileName(this, tr("Load Optimization Results"), tr("../.."),
                                 tr("Optimization results (*.txt)"));
  if (config_filename.isEmpty())
  {
    QMessageBox::warning(this, "File Error", "file name is empty");
    return;
  }
  ui->lineEdit->setText(config_filename);
}

void HomingInterfaceProprioceptive::on_pushButton_ok_clicked() { app_.Optimize(); }

void HomingInterfaceProprioceptive::on_pushButton_cancel_clicked()
{
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
                            "lost.\n Are you sure you want to abort the operation?",
                            QMessageBox::Yes | QMessageBox::No);
    if (reply == QMessageBox::No)
      return;
    break;
  }
  case HomingProprioceptive::ST_GO_HOME:
  {
    QMessageBox::StandardButton reply = QMessageBox::question(
      this, "Homing in progress", "The robot is moving to the homing position. If you "
                                  "quit now all progress will be lost.\n Are you sure "
                                  "you want to abort the operation?",
      QMessageBox::Yes | QMessageBox::No);
    if (reply == QMessageBox::No)
      return;
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
      this, "Acquisition in progress", "The application is still acquiring data from "
                                       "the robot for the homing procedure. If you "
                                       "quit now all progress will be lost.\n Are you "
                                       "sure you want to abort the operation?",
      QMessageBox::Yes | QMessageBox::No);
    if (reply == QMessageBox::No)
      return;
  }
  }

  ui->pushButton_enable->click();
  emit homingFailed();
  close();
}

void HomingInterfaceProprioceptive::on_pushButton_done_clicked()
{
  ui->pushButton_enable->click();
  emit homingSuccess();
  close();
}

void HomingInterfaceProprioceptive::AppendText2Browser(const QString& text)
{
  ui->textBrowser_logs->append(text);
}
