/**
 * @file calib_interface_excitation.cpp
 * @author Simone Comari
 * @date 20 May 2020
 * @brief This file includes definitions of class present in calib_interface_excitation.h.
 */

#include "gui/calib/calib_interface_excitation.h"
#include "ui_calib_interface_excitation.h"

const QString CalibInterfaceExcitation::kDefaultExcitationTrajDir_ =
  SRCDIR "resources/trajectories";

CalibInterfaceExcitation::CalibInterfaceExcitation(
  QWidget* parent, CableRobot* robot, const vect<grabcdpr::ActuatorParams>& params)
  : QDialog(parent), ui(new Ui::CalibInterfaceExcitation), robot_ptr_(robot),
    app_(this, robot, params), traj_parent_dir_(kDefaultExcitationTrajDir_),
    log_parent_dir_(QStandardPaths::writableLocation(QStandardPaths::DesktopLocation))
{
  ui->setupUi(this);
  setAttribute(Qt::WA_DeleteOnClose);

  connect(robot_ptr_, SIGNAL(printToQConsole(QString)), this,
          SLOT(appendText2Browser(QString)), Qt::ConnectionType::QueuedConnection);
  connect(&app_, SIGNAL(printToQConsole(QString)), this,
          SLOT(appendText2Browser(QString)), Qt::ConnectionType::DirectConnection);
  connect(&app_, SIGNAL(stateChanged(quint8)), this, SLOT(handleStateChanged(quint8)),
          Qt::ConnectionType::QueuedConnection);
}

CalibInterfaceExcitation::~CalibInterfaceExcitation()
{
  app_.disable();
  disconnect(robot_ptr_, SIGNAL(printToQConsole(QString)), this,
             SLOT(appendText2Browser(QString)));
  disconnect(&app_, SIGNAL(printToQConsole(QString)), this,
             SLOT(appendText2Browser(QString)));
  disconnect(&app_, SIGNAL(stateChanged(quint8)), this, SLOT(handleStateChanged(quint8)));
  delete ui;
  CLOG(INFO, "event") << "Calib interface excitation closed";
}

//--------- Private slots -----------------------------------------------------------//

void CalibInterfaceExcitation::appendText2Browser(const QString& text)
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

void CalibInterfaceExcitation::handleStateChanged(const quint8& state)
{
  switch (state)
  {
    case CalibExcitation::ST_IDLE:
      ui->pushButton_enable->setText(tr("Enable"));
      break;
    case CalibExcitation::ST_ENABLED:
      ui->pushButton_enable->setText(tr("Disable"));
      if (ui->radioButton_torque->isChecked())
        on_radioButton_torque_clicked();
      else
        on_radioButton_position_clicked();
      break;
    case CalibExcitation::ST_POS_CONTROL:
      ui->pushButton_logging->setEnabled(true);
      ui->pushButton_enable->setEnabled(true);
      ui->pushButton_return->setEnabled(true);
      ui->radioButton_torque->setEnabled(true);
      ui->radioButton_position->setEnabled(true);
      break;
    case CalibExcitation::ST_LOGGING:
      ui->pushButton_enable->setDisabled(true);
      ui->pushButton_return->setDisabled(true);
      ui->pushButton_logging->setDisabled(true);
      ui->pushButton_save->setEnabled(true);
      ui->radioButton_torque->setDisabled(true);
      ui->radioButton_position->setDisabled(true);
      break;
    case CalibExcitation::ST_TORQUE_CONTROL:
      ui->pushButton_logging->setDisabled(true);
      break;
  }
}

//--------- Private GUI slots -------------------------------------------------------//

void CalibInterfaceExcitation::on_pushButton_enable_clicked()
{
  bool robot_enabled = ui->pushButton_enable->text() == "Disable";
  CLOG(TRACE, "event") << (robot_enabled ? "DISABLE" : "ENABLE");
  ui->pushButton_enable->setChecked(false);

  if (!robot_enabled)
    app_.enable(); // IDLE --> ENABLED
  else
    app_.disable(); // any --> IDLE
}

void CalibInterfaceExcitation::on_radioButton_torque_clicked()
{
  CLOG(TRACE, "event");
  ui->radioButton_torque->setChecked(true);
  ui->radioButton_position->setChecked(false);
  CalibExcitationData* data =
    new CalibExcitationData(static_cast<short>(ui->spinBox_torque->value()));
  app_.changeControlMode(data);
}

void CalibInterfaceExcitation::on_radioButton_position_clicked()
{
  CLOG(TRACE, "event");
  ui->radioButton_torque->setChecked(false);
  ui->radioButton_position->setChecked(true);
  app_.changeControlMode();
}

void CalibInterfaceExcitation::on_pushButton_fileSelection_clicked()
{
  CLOG(TRACE, "event");
  QString filename = QFileDialog::getOpenFileName(
    this, tr("Load Excitation Trajectory"), traj_parent_dir_, tr("Trajectory file (*.txt)"));
  if (filename.isEmpty())
  {
    QMessageBox::warning(this, "File Error", "File name is empty!");
    return;
  }
  ui->lineEdit_inputFile->setText(filename);
  // Update parent directory
  QDir dir    = QFileInfo(filename).absoluteDir();
  traj_parent_dir_ = dir.absolutePath();
}

void CalibInterfaceExcitation::on_pushButton_logging_clicked()
{
  if (ui->lineEdit_inputFile->text().isEmpty())
  {
    appendText2Browser("WARNING: missing trajectory file!");
    app_.exciteAndLog();
  }
  else
  {
    CalibExcitationData* data = new CalibExcitationData(ui->lineEdit_inputFile->text());
    app_.exciteAndLog(data);
  }
}

void CalibInterfaceExcitation::on_pushButton_save_clicked()
{
  app_.stopLogging();

  CLOG(TRACE, "event");
  QString filename = QFileDialog::getSaveFileName(
    this, tr("Save Excitation Log Data"), log_parent_dir_, tr("Log Data (*.log)"));
  if (filename.isEmpty())
  {
    QMessageBox::warning(this, "File Error", "File name is empty!");
    return;
  }
  // Update parent directory
  QFileInfo file_info(filename);
  QDir dir        = file_info.absoluteDir();
  log_parent_dir_ = dir.absolutePath();
  // Make sure filenamehas correct extension
  filename = log_parent_dir_ + "/" + file_info.completeBaseName() + ".log";

  // Copy log file in given position and clear it before new acquisition
  robot_ptr_->copyDataLogs(filename);
  robot_ptr_->flushDataLogs();

  ui->pushButton_save->setDisabled(true);
}

void CalibInterfaceExcitation::on_pushButton_return_clicked()
{
  CLOG(TRACE, "event");
  close();
}
