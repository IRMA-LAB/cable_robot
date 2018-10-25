#include "gui/homing_dialog.h"
#include "ui_homing_dialog.h"

HomingDialog::HomingDialog(QWidget* parent, const grabcdpr::Params& config)
  : QDialog(parent), ui(new Ui::HomingDialog), config_(config)
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
