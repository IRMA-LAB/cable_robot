/**
 * @file login_window.cpp
 * @author Simone Comari
 * @date 23 Jul 2019
 * @brief This file includes definitions of window class present in login_window.h.
 */

#include "gui/login_window.h"
#include "ui_login_window.h"

#define USB_STORAGE_NAME "2EDA-2625" /**< Authentication USB stick name. */

LoginWindow::LoginWindow(QWidget* parent) : QMainWindow(parent), ui(new Ui::LoginWindow)
{
  ui->setupUi(this);
  setFixedHeight(this->geometry().height());

#if DEBUG_GUI == 1
  ui->groupBox_config->setEnabled(true);
#endif
}

LoginWindow::~LoginWindow()
{
  delete ui;
  CLOG(INFO, "event") << "Login window closed";
}

//--------- Private slots ------------------------------------------------------------//

void LoginWindow::on_pushButton_login_clicked()
{
  CLOG(TRACE, "event");
  username_        = ui->lineEdit_username->text();
  QString password = ui->lineEdit_password->text();

  RetVal ret = isValidUser(username_, password);
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

void LoginWindow::on_pushButton_robotConfig_clicked()
{
  CLOG(TRACE, "event");
  QString config_filename = QFileDialog::getOpenFileName(
    this, tr("Load Robot Configuration File"), tr(SRCDIR "/config"),
    tr("Robot Configuration File (*.json)"));
  if (config_filename.isEmpty())
  {
    CLOG(WARNING, "event") << "Robot configuration filename is empty";
    QMessageBox::warning(this, "File Error", "Robot configuration filename is empty");
    return;
  }
  ui->lineEdit_robotConfig->setText(config_filename);
}

void LoginWindow::on_pushButton_sensorsConfig_clicked()
{
  CLOG(TRACE, "event");
  QString config_filename = QFileDialog::getOpenFileName(
    this, tr("Load Sensors Configuration File"), tr(SRCDIR "/config"),
    tr("Sensors configuration File (*.json)"));
  if (config_filename.isEmpty())
  {
    CLOG(WARNING, "event") << "Sensors configuration filename is empty";
    QMessageBox::warning(this, "File Error", "Sensors configuration filename is empty");
    return;
  }
  ui->lineEdit_sensorsConfig->setText(config_filename);
}

void LoginWindow::on_pushButton_load_clicked()
{
  CLOG(TRACE, "event");
  QString robot_config_filename = ui->lineEdit_robotConfig->text();
  if (robot_config_filename.isEmpty())
  {
    on_pushButton_robotConfig_clicked();
    robot_config_filename = ui->lineEdit_robotConfig->text();
    if (robot_config_filename.isEmpty())
      return;
  }
  QString sensors_config_filename = ui->lineEdit_sensorsConfig->text();
  if (sensors_config_filename.isEmpty())
  {
    if (QMessageBox::No == QMessageBox::question(this,
                                                 "Sensors Configuration File Missing",
                                                 "Without sensors configuration some\n"
                                                 "functionalities will not be enabled.\n"
                                                 "Do you wish to continue?"))
    {
      on_pushButton_sensorsConfig_clicked();
      sensors_config_filename = ui->lineEdit_sensorsConfig->text();
      if (sensors_config_filename.isEmpty())
        return;
    }
  }
  loadConfigFiles(robot_config_filename, sensors_config_filename);
}

void LoginWindow::on_pushButton_loadDefault_clicked()
{
  CLOG(TRACE, "event");
  QString robot_default_filename(SRCDIR);
  robot_default_filename.append("config/default.json");
  QString sensors_default_filename(SRCDIR);
  sensors_default_filename.append("config/sensors_config.json");
  loadConfigFiles(robot_default_filename, sensors_default_filename);
}

//--------- Private Functions --------------------------------------------------------//

LoginWindow::RetVal LoginWindow::isValidUser(const QString& username,
                                             const QString& password) const
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

bool LoginWindow::parseRobotConfigFile(const QString& config_filename)
{
  RobotConfigJsonParser parser;
  CLOG(INFO, "event") << "Parsing robot configuration file '" << config_filename
                      << "'...";
  return parser.ParseFile(config_filename, &robot_config_);
}

bool LoginWindow::parseSensorsConfigFile(const QString& config_filename)
{
  SensorsConfigJsonParser parser;
  CLOG(INFO, "event") << "Parsing sensors configuration file '" << config_filename
                      << "'...";
  return parser.ParseFile(config_filename, &sensors_config_);
}

void LoginWindow::loadConfigFiles(const QString& robot_config_filename,
                                  const QString& sensors_config_filename)
{
  if (!parseRobotConfigFile(robot_config_filename))
  {
    CLOG(WARNING, "event") << "Robot Configuration file is not valid";
    QMessageBox::warning(this, "File Error", "Robot configuration file is not valid");
    return;
  }
  CLOG(INFO, "event") << "Loaded robot configuration file '" << robot_config_filename
                      << "'";

  // Generate file with temporary information that can be useful during the app execution
  QString tmp_info = QStandardPaths::writableLocation(QStandardPaths::TempLocation) +
                     "/cable_robot_app_tmp.txt";
  if (QFile::exists(tmp_info))
    QFile::remove(tmp_info);
  QFile file(tmp_info);
  if (file.open(QIODevice::WriteOnly | QIODevice::Text))
  {
    QTextStream out(&file);
    out << robot_config_filename << "\n";
  }

  if (sensors_config_filename.isEmpty())
    main_gui = new MainGUI(this, robot_config_);
  else
  {
    if (!parseSensorsConfigFile(sensors_config_filename))
    {
      CLOG(WARNING, "event") << "Sensors Configuration file is not valid";
      QMessageBox::warning(this, "File Error", "Sensors configuration file is not valid");
      return;
    }
    CLOG(INFO, "event") << "Loaded sensors configuration file '" << robot_config_filename
                        << "'";
    main_gui = new MainGUI(this, robot_config_, sensors_config_);
  }

  hide();
  CLOG(INFO, "event") << "Hide login window";
  main_gui->show();
  CLOG(INFO, "event") << "Prompt main window";
}
