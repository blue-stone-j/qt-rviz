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
  // 本质是一个已定义好的宏，所有需要“信号和槽”功能的组件都必须将 Q_OBJECT 作为 private 属性成员引入到类中
  Q_OBJECT

public:
  explicit MainWindow(QWidget *parent = 0);

  QString default_dir; // 默认的文件读取路径

  Publishers pubers = Publishers::getInstance();
  std::string root_path;     // 程序编译产生的可执行文件的路径
  ~MainWindow();

private:
  Ui::MainWindow *ui; // UI界面

private slots:
  void on_select_trace_clicked();
  void on_viewport_clicked();     // 未使用的槽函数
  bool on_select_cloud_clicked(); // 用来选取新的文件夹
  bool openAbout();               // 打开About窗口

private:
  // display
  void addDisplays();                  // 为内嵌的rviz添加话题、消息并设置显示的属性
  void closeEvent(QCloseEvent *event); // 发生关闭窗口事件后需要执行的函数
protected:
  void resizeEvent(QResizeEvent *event) override
  {
    QMainWindow::resizeEvent(event); 
    resizeWidgets();
  }
private:
  void resizeWidgets();
};