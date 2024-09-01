#include <iostream>
#include <deque>
#include <thread>

#include <QMainWindow>
#include <QFileDialog>
#include <QDir>
#include <QtWidgets/QMessageBox>

#include "display/ui_mainwindow.h"

#include "publishers.h"

namespace Ui
{
class MainWindow;
}

class MainWindow : public QMainWindow
{
  // a define macro, it's necessary to import it as a private member for the class that contains signal and slot
  Q_OBJECT

 public:
  explicit MainWindow(QWidget *parent = 0);

  QString default_dir; // defaule directory to read file

  Publishers pubers = Publishers::getInstance();
  std::string root_path; // the directory of executable file
  ~MainWindow();

 private:
  Ui::MainWindow *ui; // UI and mainwindow layout

 private slots:
  void on_select_trace_clicked();
  void on_viewport_clicked();     // unused
  bool on_select_cloud_clicked(); // slot to select a new folder
  bool openAbout();               // oen "About" form

 private:
  // display
  void addDisplays();                  // add topics and messages and add display properties in embeded rviz.
  void closeEvent(QCloseEvent *event); // this function will perform when a close event occurs
 protected:
  void resizeEvent(QResizeEvent *event) override
  {
    QMainWindow::resizeEvent(event);
    resizeWidgets();
  }

 private:
  void resizeWidgets();
};