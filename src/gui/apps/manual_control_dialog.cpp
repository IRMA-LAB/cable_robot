#include "gui/apps/manual_control_dialog.h"
#include "ui_manual_control_dialog.h"

ManualControlDialog::ManualControlDialog(QWidget* parent, CableRobot* robot)
  : QDialog(parent), ui(new Ui::ManualControlDialog)
{
  ui->setupUi(this);
  setAttribute(Qt::WA_DeleteOnClose);

  app_ = new ManualControlApp(this, robot);

  connect(app_, SIGNAL(printToQConsole(QString)), this, SLOT(appendText2Browser(QString)),
          Qt::ConnectionType::QueuedConnection);
}

ManualControlDialog::~ManualControlDialog()
{
  disconnect(app_, SIGNAL(printToQConsole(QString)), this,
             SLOT(appendText2Browser(QString)));
  delete ui;
  delete app_;
}

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

void ManualControlDialog::on_pushButton_return_clicked() { close(); }

void ManualControlDialog::on_pushButton_next_clicked()
{
  app_->next(); // dummy
}
