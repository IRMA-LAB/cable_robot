#include <QApplication>

#include "lib/easyloggingpp/src/easylogging++.h"
#include "gui/login_window.h"
#include "utils/easylog_wrapper.h"

INITIALIZE_EASYLOGGINGPP

//static el::Logger* logger = el::Loggers::getLogger("default");
//static LogBuffer buffer(logger);

int main(int argc, char *argv[])
{
    START_EASYLOGGINGPP(argc, argv);
    // Load configuration from file
    el::Configurations conf("config/log_conf.conf");
    // Reconfigure single logger
    el::Loggers::reconfigureLogger("default", conf);

    QApplication a(argc, argv);
    qRegisterMetaType<grabec::GSWDriveInPdos>("grabec::GSWDriveInPdos");

    LoginWindow w;
    w.show();

    return a.exec();
}
