#ifndef CABLE_ROBOT_LOGIN_WINDOW_H
#define CABLE_ROBOT_LOGIN_WINDOW_H

#include <QMainWindow>
#include <QMessageBox>
#include <QFileDialog>
#include <QResource>
#include <QStorageInfo>

#include <iostream>
#include <fstream>

#include "main_gui.h"
#include "robotconfigjsonparser.h"
#include "json.hpp"
#include "libcdpr/inc/types.h"

using json = nlohmann::json;

namespace Ui
{
class LoginWindow;
}

class LoginWindow : public QMainWindow
{
  Q_OBJECT

public:
  explicit LoginWindow(QWidget* parent = 0);
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
