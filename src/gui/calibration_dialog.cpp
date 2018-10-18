#include "gui/calibration_dialog.h"
#include "ui_calibration_dialog.h"

CalibrationDialog::CalibrationDialog(QWidget* parent, QString& config_filename)
  : QDialog(parent), ui(new Ui::CalibrationDialog), config_filename_(config_filename)
{
  ui->setupUi(this);
}

CalibrationDialog::~CalibrationDialog() { delete ui; }

void CalibrationDialog::on_buttonBox_accepted()
{
  emit calibrationEnd();
  emit enableMainGUI();
  close();
}

void CalibrationDialog::on_buttonBox_rejected()
{
  emit calibrationEnd();
  emit enableMainGUI();
  close();
}
