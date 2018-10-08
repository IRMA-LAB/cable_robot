#include "gui/main_gui.h"
#include "ui_main_gui.h"

MainGUI::MainGUI(QWidget *parent, QString &config_filename) :
    QDialog(parent),
    ui(new Ui::MainGUI),
    config_filename_(config_filename)
{
    ui->setupUi(this);
}

MainGUI::~MainGUI()
{
    delete ui;
}

void MainGUI::on_pushButton_calib_clicked()
{
  ui->pushButton_homing->setDisabled(true);
  ui->pushButton_calib->setDisabled(true);
  ui->groupBox_app->setDisabled(true);
  ui->frame_manualControl->setDisabled(true);

  calib_dialog = new CalibrationDialog(this);
  calib_dialog->show();
}

void MainGUI::on_pushButton_homing_clicked()
{
  ui->pushButton_homing->setDisabled(true);
  ui->pushButton_calib->setDisabled(true);
  ui->groupBox_app->setDisabled(true);
  ui->frame_manualControl->setDisabled(true);

  homing_dialog = new HomingDialog(this);
  homing_dialog->show();
}

void MainGUI::on_pushButton_startApp_clicked()
{
  ui->pushButton_homing->setDisabled(true);
  ui->pushButton_calib->setDisabled(true);
  ui->groupBox_app->setDisabled(true);
  ui->frame_manualControl->setDisabled(true);
}

void MainGUI::on_pushButton_enable_clicked()
{
  ui->pushButton_homing->setDisabled(true);
  ui->pushButton_calib->setDisabled(true);
  ui->groupBox_app->setDisabled(true);
}

void MainGUI::on_pushButton_faultReset_clicked()
{

}
