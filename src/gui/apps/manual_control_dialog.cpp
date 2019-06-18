#include "gui/apps/manual_control_dialog.h"
#include "ui_manual_control_dialog.h"

ManualControlDialog::ManualControlDialog(QWidget* parent, CableRobot* robot)
  : QDialog(parent), ui(new Ui::ManualControlDialog), robot_ptr_(robot), app_(this, robot)
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

ManualControlDialog::~ManualControlDialog()
{
  app_.disable();
  disconnect(robot_ptr_, SIGNAL(printToQConsole(QString)), this,
             SLOT(appendText2Browser(QString)));
  disconnect(&app_, SIGNAL(printToQConsole(QString)), this,
             SLOT(appendText2Browser(QString)));
  disconnect(&app_, SIGNAL(stateChanged(quint8)), this, SLOT(handleStateChanged(quint8)));
  delete ui;
  CLOG(INFO, "event") << "Manual control dialog closed";
}

//--------- Private slots -----------------------------------------------------------//

void ManualControlDialog::appendText2Browser(const QString& text)
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

void ManualControlDialog::handleStateChanged(const quint8& state)
{
  switch (state)
  {
    case ManualControlApp::ST_IDLE:
      ui->pushButton_enable->setText(tr("Enable"));
      break;
    case ManualControlApp::ST_ENABLED:
      ui->pushButton_enable->setText(tr("Disable"));
      if (ui->radioButton_torque->isChecked())
        on_radioButton_torque_clicked();
      else
        on_radioButton_position_clicked();
      break;
    case ManualControlApp::ST_POS_CONTROL:
      ui->pushButton_logging->setEnabled(true);
      ui->pushButton_enable->setEnabled(true);
      ui->pushButton_return->setEnabled(true);
      ui->radioButton_torque->setEnabled(true);
      ui->radioButton_position->setEnabled(true);
      break;
    case ManualControlApp::ST_LOGGING:
      ui->pushButton_enable->setDisabled(true);
      ui->pushButton_return->setDisabled(true);
      ui->pushButton_logging->setDisabled(true);
      ui->radioButton_torque->setDisabled(true);
      ui->radioButton_position->setDisabled(true);
      break;
    case ManualControlApp::ST_TORQUE_CONTROL:
      ui->pushButton_logging->setDisabled(true);
      break;
  }
}

//--------- Private GUI slots -------------------------------------------------------//

void ManualControlDialog::on_pushButton_enable_clicked()
{
  bool robot_enabled = ui->pushButton_enable->text() == "Disable";
  CLOG(TRACE, "event") << (robot_enabled ? "DISABLE" : "ENABLE");
  ui->pushButton_enable->setChecked(false);

  if (!robot_enabled)
    app_.enable(); // IDLE --> ENABLED
  else
    app_.disable(); // any --> IDLE
}

void ManualControlDialog::on_radioButton_torque_clicked()
{
  CLOG(TRACE, "event");
  ui->radioButton_torque->setChecked(true);
  ui->radioButton_position->setChecked(false);
  MyData* data = new MyData(static_cast<short>(ui->spinBox_torque->value()));
  app_.changeControlMode(data);
}

void ManualControlDialog::on_radioButton_position_clicked()
{
  CLOG(TRACE, "event");
  ui->radioButton_torque->setChecked(false);
  ui->radioButton_position->setChecked(true);
  app_.changeControlMode();
}

void ManualControlDialog::on_pushButton_return_clicked()
{
  CLOG(TRACE, "event");
  close();
}

void ManualControlDialog::on_pushButton_logging_clicked() { app_.exciteAndLog(); }
