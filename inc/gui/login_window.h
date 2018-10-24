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
#include "json.hpp"

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
  QString config_filename_;

  enum RetVal
  {
    OK,
    ERR_IO,
    ERR_INVAL
  };

  RetVal IsValidUser(QString& username, QString& password) const;
  bool IsConfigFileValid() const { return true; }
};

#endif // CABLE_ROBOT_LOGIN_WINDOW_H
