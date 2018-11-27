#include <QApplication>
#include <QtDebug>

#include "lib/easyloggingpp/src/easylogging++.h"
#include "gui/login_window.h"
#include "utils/easylog_wrapper.h"

INITIALIZE_EASYLOGGINGPP

//static el::Logger* logger = el::Loggers::getLogger("default");
//static LogBuffer buffer(logger);

int main(int argc, char *argv[])
{
    START_EASYLOGGINGPP(argc, argv);
    // Configure all loggers
    el::Loggers::configureFromGlobal("../../config/logs.conf");

    QApplication a(argc, argv);
    qRegisterMetaType<grabec::GSWDriveInPdos>("grabec::GSWDriveInPdos");
    CLOG(INFO, "event") << "App START";

    LoginWindow w;
    w.show();
    CLOG(INFO, "event") << "Prompt login in window";

    return a.exec();
}
