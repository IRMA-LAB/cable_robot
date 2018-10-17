#include <QApplication>

#include "gui/login_window.h"

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);

    LoginWindow w;
    w.show();

    return a.exec();
}
