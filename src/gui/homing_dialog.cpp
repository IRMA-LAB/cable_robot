#include "gui/homing_dialog.h"
#include "ui_homing_dialog.h"

HomingDialog::HomingDialog(QWidget* parent, QString& config_filename)
  : QDialog(parent), ui(new Ui::HomingDialog), config_filename_(config_filename)
{
  ui->setupUi(this);
}

HomingDialog::~HomingDialog() { delete ui; }

void HomingDialog::on_buttonBox_accepted()
{
  emit homingSuccess();
  emit enableMainGUI(true);
  close();
}

void HomingDialog::on_buttonBox_rejected()
{
  emit homingFailed();
  emit enableMainGUI(false);
  close();
}
