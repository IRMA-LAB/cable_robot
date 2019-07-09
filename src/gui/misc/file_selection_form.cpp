/**
 * @file file_selection_form.cpp
 * @author Simone Comari
 * @date 08 Jul 2019
 * @brief This file includes definitions of class present in file_selection_form.h.
 */

#include "gui/misc/file_selection_form.h"
#include "ui_file_selection_form.h"

FileSelectionForm::FileSelectionForm(QWidget* parent)
  : QWidget(parent), ui(new Ui::FileSelectionForm)
{
  ui->setupUi(this);
}

FileSelectionForm::~FileSelectionForm() { delete ui; }

//--------- Public functions --------------------------------------------------------//

bool FileSelectionForm::isInputEmpty() const
{
  return ui->lineEdit_inputFile->text().isEmpty();
}

QString FileSelectionForm::getFilepath() const { return ui->lineEdit_inputFile->text(); }

//--------- Private GUI slots -------------------------------------------------------//

void FileSelectionForm::on_pushButton_fileSelection_clicked()
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
