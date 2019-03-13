/**
 * @file login_window.cpp
 * @author Simone Comari
 * @date 13 Mar 2019
 * @brief This file includes definitions of window class present in login_window.h.
 */

#include "gui/login_window.h"
#include "ui_login_window.h"

#define USB_STORAGE_NAME "2EDA-2625" /**< Authentication USB stick name. */

LoginWindow::LoginWindow(QWidget* parent) : QMainWindow(parent), ui(new Ui::LoginWindow)
{
  ui->setupUi(this);
  setFixedHeight(this->geometry().height());
}

LoginWindow::~LoginWindow()
{
  delete ui;
  CLOG(INFO, "event") << "Login window closed";
}

LoginWindow::RetVal LoginWindow::IsValidUser(QString& username, QString& password) const
{
  std::string usb_path = "";
  foreach (const QStorageInfo& storage, QStorageInfo::mountedVolumes())
  {
    if (storage.name() == USB_STORAGE_NAME ||
        storage.rootPath().mid(storage.rootPath().lastIndexOf("/") + 1) ==
          USB_STORAGE_NAME)
    {
      usb_path = storage.rootPath().toStdString();
      break;
    }
  }

  // Open file
  std::ifstream ifile(usb_path + "/.users.json");
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

void LoginWindow::on_pushButton_login_clicked()
{
  CLOG(TRACE, "event");
  username_        = ui->lineEdit_username->text();
  QString password = ui->lineEdit_password->text();

  RetVal ret = IsValidUser(username_, password);
  switch (ret)
  {
    case OK:
      CLOG(INFO, "event") << "Login in success. Current user: " << username_;
      ui->groupBox_config->setEnabled(true);
      ui->groupBox_signIn->setDisabled(true);
      break;
    case ERR_IO:
      CLOG(WARNING, "event") << "Login in failed: missing USB stick ";
      QMessageBox::warning(this, "I/O Error",
                           "Please insert authentication usb stick and try again");
      break;
    case ERR_INVAL:
      CLOG(WARNING, "event") << "Login in failed: invalid user";
      QMessageBox::warning(this, "Login Error",
                           "Username and/or password is not correct");
      break;
  }
}

void LoginWindow::on_pushButton_inputFile_clicked()
{
  CLOG(TRACE, "event");
  QString config_filename =
    QFileDialog::getOpenFileName(this, tr("Load Configuration File"), tr("../../config"),
                                 tr("Configuration File (*.json)"));
  if (config_filename.isEmpty())
  {
    CLOG(WARNING, "event") << "Configuration filename is empty";
    QMessageBox::warning(this, "File Error", "Configuration filename is empty");
    return;
  }
  ui->lineEdit_inputFile->setText(config_filename);
}

void LoginWindow::on_pushButton_load_clicked()
{
  CLOG(TRACE, "event");
  QString config_filename = ui->lineEdit_inputFile->text();
  if (config_filename.isEmpty())
  {
    ui->pushButton_inputFile->click();
    config_filename = ui->lineEdit_inputFile->text();
    if (config_filename.isEmpty())
      return;
  }
  if (!ParseConfigFile(config_filename))
  {
    CLOG(WARNING, "event") << "Configuration file is not valid";
    QMessageBox::warning(this, "File Error", "Configuration file is not valid");
    return;
  }
  CLOG(INFO, "event") << "Loaded configuration file '" << config_filename << "'";
  main_gui = new MainGUI(this, config_);
  hide();
  CLOG(INFO, "event") << "Hide login window";
  main_gui->show();
  CLOG(INFO, "event") << "Prompt main window";
}

void LoginWindow::on_pushButton_loadDefault_clicked()
{
  CLOG(TRACE, "event");
  QString default_filename(SRCDIR);
  default_filename.append("config/default.json");
  CLOG(INFO, "event") << "Loaded default configuration file '" << default_filename << "'";
  ParseConfigFile(default_filename);
  main_gui = new MainGUI(this, config_);
  hide();
  CLOG(INFO, "event") << "Hide login window";
  main_gui->show();
  CLOG(INFO, "event") << "Prompt main window";
}

bool LoginWindow::ParseConfigFile(QString& config_filename)
{
  RobotConfigJsonParser parser;
  CLOG(INFO, "event") << "Parsing configuration file '" << config_filename << "'...";
  return parser.ParseFile(config_filename, &config_);
}
