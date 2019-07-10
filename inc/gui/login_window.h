/**
 * @file login_window.h
 * @author Simone Comari
 * @date 09 Jul 2019
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

#include <fstream>
#include <iostream>

#include "json.hpp"
#include "libcdpr/inc/cdpr_types.h"
#include "main_gui.h"
#include "robotconfigjsonparser.h"

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

  void on_pushButton_inputFile_clicked();
  void on_pushButton_load_clicked();
  void on_pushButton_loadDefault_clicked();

 private:
  Ui::LoginWindow* ui;
  MainGUI* main_gui;

  QString username_;
  grabcdpr::Params config_;

  enum RetVal
  {
    OK,
    ERR_IO,
    ERR_INVAL
  };

  RetVal IsValidUser(QString& username, QString& password) const;
  bool ParseConfigFile(QString& config_filename);
};

#endif // CABLE_ROBOT_LOGIN_WINDOW_H
