/**
 * @file login_window.h
 * @author Simone Comari
 * @date 10 Jan 2020
 * @brief This file takes care of the implementation of the login window of cable robot
 * app.
 */

#ifndef CABLE_ROBOT_LOGIN_WINDOW_H
#define CABLE_ROBOT_LOGIN_WINDOW_H

#include <QFileDialog>
#include <QMainWindow>
#include <QMessageBox>
#include <QResource>
#include <QStorageInfo>
#include <QString>

#include <fstream>
#include <iostream>

#include "cdpr_types.h"
#include "json.hpp"
#include "robotconfigjsonparser.h"

#include "utils/sensorsconfigjsonparser.h"

#include "main_gui.h"

using json = nlohmann::json; /**< Alias for json namespace. */

namespace Ui {
class LoginWindow;
}

/**
 * @brief The login window class.
 *
 * This class manages the access to the cable robot app as well as the configuration file
 * given by the user to setup the robot at software level according to the physical
 * configuration.
 */
class LoginWindow: public QMainWindow
{
  Q_OBJECT

 public:
  /**
   * @brief LoginWindow constructor.
   * @param[in] parent The parent QWidget, in this case the QApplication of cable robot
   * app.
   */
  explicit LoginWindow(QWidget* parent = nullptr);
  ~LoginWindow();

 private slots:
  void on_pushButton_login_clicked();

  void on_pushButton_robotConfig_clicked();
  void on_pushButton_sensorsConfig_clicked();
  void on_pushButton_load_clicked();
  void on_pushButton_loadDefault_clicked();

 private:
  Ui::LoginWindow* ui;
  MainGUI* main_gui;

  QString username_;
  grabcdpr::RobotParams robot_config_;
  SensorsParams sensors_config_;

  enum RetVal
  {
    OK,
    ERR_IO,
    ERR_INVAL
  };

  RetVal isValidUser(const QString& username, const QString& password) const;
  bool parseRobotConfigFile(const QString& config_filename);
  bool parseSensorsConfigFile(const QString& config_filename);
  void loadConfigFiles(const QString& robot_config_filename,
                       const QString& sensors_config_filename);
};

#endif // CABLE_ROBOT_LOGIN_WINDOW_H
