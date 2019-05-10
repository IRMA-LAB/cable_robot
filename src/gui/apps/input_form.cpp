#include "gui/apps/input_form.h"
#include "ui_input_form.h"

InputForm::InputForm(QWidget *parent) :
  QWidget(parent),
  ui(new Ui::InputForm)
{
  ui->setupUi(this);
}

InputForm::~InputForm()
{
  delete ui;
}

bool InputForm::isInputEmpty() const
{
  return ui->lineEdit_inputFile->text().isEmpty();
}

QString InputForm::getFilepath() const
{
  return ui->lineEdit_inputFile->text();
}

void InputForm::on_pushButton_fileSelection_clicked()
{
  CLOG(TRACE, "event");
  QString config_filename = QFileDialog::getOpenFileName(
    this, tr("Load Trajectory"), tr("../.."), tr("Trajectory file (*.txt)"));
  if (config_filename.isEmpty())
  {
    QMessageBox::warning(this, "File Error", "File name is empty!");
    return;
  }
  ui->lineEdit_inputFile->setText(config_filename);
}
