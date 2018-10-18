#include <QMessageBox>
#include <QFileDialog>
#include <QResource>

#include "gui/login_window.h"
#include "ui_login_window.h"

LoginWindow::LoginWindow(QWidget* parent) : QMainWindow(parent), ui(new Ui::LoginWindow)
{
  ui->setupUi(this);
  setFixedHeight(this->geometry().height());
}

LoginWindow::~LoginWindow() { delete ui; }

bool LoginWindow::IsValidUser(QString& username, QString& password) const
{
  return username == "test" && password == "test";
}

//debug
void LoginWindow::on_pushButton_login_clicked()
{
  ui->groupBox_config->setEnabled(true);
  ui->pushButton_loadDefault->click();
}

// void LoginWindow::on_pushButton_login_clicked()
//{
//  username_ = ui->lineEdit_username->text();
//  QString password = ui->lineEdit_password->text();

//  if (IsValidUser(username_, password))
//  {
//    QMessageBox::information(this, "Login Success",
//                             "Username and password are correct.\n"
//                             "Please load a configuration file or use default one.");
//    ui->groupBox_config->setEnabled(true);
//    ui->groupBox_signIn->setDisabled(true);
//  }
//  else
//  {
//    QMessageBox::warning(this, "Login Error", "Username and/or password is not
//    correct");
//  }
//}

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
