#include "gui/homing_dialog.h"
#include "ui_homing_dialog.h"

HomingDialog::HomingDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::HomingDialog)
{
    ui->setupUi(this);
}

HomingDialog::~HomingDialog()
{
    delete ui;
}
