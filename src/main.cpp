#include <QApplication>

#include "gui/login_window.h"

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    qRegisterMetaType<grabec::GSWDriveInPdos>("grabec::GSWDriveInPdos");

    LoginWindow w;
    w.show();

    return a.exec();
}
