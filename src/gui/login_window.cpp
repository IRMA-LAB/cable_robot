
#include "gui/login_window.h"
#include "ui_login_window.h"

LoginWindow::LoginWindow(QWidget* parent) : QMainWindow(parent), ui(new Ui::LoginWindow)
{
  ui->setupUi(this);
  setFixedHeight(this->geometry().height());
}

LoginWindow::~LoginWindow() { delete ui; }

LoginWindow::RetVal LoginWindow::IsValidUser(QString& username, QString& password) const
{
  std::string usb_path = "";
  foreach (const QStorageInfo& storage, QStorageInfo::mountedVolumes())
  {
    if (storage.name() == "sbb")
    {
      usb_path = storage.rootPath().toStdString();
      break;
    }
  }

  // Open file
  std::ifstream ifile(usb_path+"/.users.json");
  if (!ifile.is_open())
  {
    return ERR_IO;
  }

  // Parse JSON (generic) data
  json data;
  ifile >> data;
  ifile.close();

  // Look for existing user
  for (auto& user : data["users"])
    if (username.toStdString() == user["username"] &&
        password.toStdString() == user["password"])
      return OK;

  return ERR_INVAL;
}

// debug
// void LoginWindow::on_pushButton_login_clicked()
//{
//  ui->groupBox_config->setEnabled(true);
//  ui->pushButton_loadDefault->click();
//}

void LoginWindow::on_pushButton_login_clicked()
{
  username_ = ui->lineEdit_username->text();
  QString password = ui->lineEdit_password->text();

  RetVal ret = IsValidUser(username_, password);
  switch (ret)
  {
  case OK:
//    QMessageBox::information(this, "Login Success",
//                             "Username and password are correct.\n"
//                             "Please load a configuration file or use default one.");
    ui->groupBox_config->setEnabled(true);
    ui->groupBox_signIn->setDisabled(true);
    break;
  case ERR_IO:
    QMessageBox::warning(const_cast<LoginWindow*>(this), "I/O Error",
                         "Please insert authentication usb stick and try again");
    break;
  case ERR_INVAL:
    QMessageBox::warning(this, "Login Error", "Username and/or password is not correct");
    break;
  }
}

void LoginWindow::on_pushButton_inputFile_clicked()
{
  config_filename_ =
    QFileDialog::getOpenFileName(this, tr("Load Configuration File"), tr("../../config"),
                                 tr("Configuration File (*.json)"));
  if (config_filename_.isEmpty())
  {
    QMessageBox::warning(this, "File Error", "Configuration file is empty");
    return;
  }
  ui->lineEdit_inputFile->setText(config_filename_);
}

void LoginWindow::on_pushButton_load_clicked()
{
  config_filename_ = ui->lineEdit_inputFile->text();
  while (config_filename_.isEmpty())
    ui->pushButton_inputFile->click();
  if (!IsConfigFileValid())
  {
    QMessageBox::warning(this, "File Error", "Configuration file is not valid");
    return;
  }
  hide();
  main_gui = new MainGUI(this, config_filename_);
  main_gui->show();
}

void LoginWindow::on_pushButton_loadDefault_clicked()
{
  hide();
  QString default_filename = ":/config/default.json";
  main_gui = new MainGUI(this, default_filename);
  main_gui->show();
}
