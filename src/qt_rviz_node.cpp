
#include <QtWidgets/QApplication>
#include <QMainWindow>

#include "display/mainwindow.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "qt_rviz");

  FLAGS_log_dir = std::string(getenv("HOME")) + "/logs/qt_rviz";
  FLAGS_colorlogtostderr = true;
  FLAGS_alsologtostderr = true;

  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InitGoogleLogging(argv[0]);

  QApplication a(argc, argv);

  MainWindow w;
  w.show();

  return a.exec();

  // ros::spin();
  // return 0;
}
