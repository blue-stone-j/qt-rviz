#include "display/mainwindow.h"
#include "utlqr.h"
// #define MY_PRINT_VAR(var) #var

MainWindow::MainWindow(QWidget *parent) :
  QMainWindow(parent), ui(new Ui::MainWindow)
{
  root_path   = utlqr::get_root_path();
  default_dir = QString::fromStdString(root_path) + QString("/assets");
  ui->setupUi(this, QString().fromStdString(root_path + "/assets/")); // create window layout
  addDisplays();

  //(action,trigger signal,this,slot function)
  connect(ui->about, SIGNAL(triggered()), this, SLOT(openAbout()));

  // std::cout << MY_PRINT_VAR(mode) << std::endl;
}

MainWindow::~MainWindow()
{
  delete ui;
}

void MainWindow::resizeWidgets()
{
  // ui->cloud_size1->setGeometry(QRect(20, 10, 70, 30));
  // ui->render_panel->setGeometry(QRect(10, 50, width()-20, height()-85));
}

bool MainWindow::on_select_cloud_clicked()
{
  QString file_path      = QFileDialog::getOpenFileName(this, "select cloud", default_dir, "*.pcd");
  std::string cloud_file = file_path.toStdString();
  pcl::PointCloud<pcl::PointXYZI> cloud;
  if (std::filesystem::exists(cloud_file))
  {
    pcl::io::loadPCDFile(cloud_file, cloud);
  }
  else
  {
    return false;
  }
  pubers.publishCloud(cloud);
  ui->cloud_size2->setText(QString::number(cloud.size() / 10000, 10) + QString(",") + QString::number(cloud.size() % 10000, 10));

  return true;
}


// add topics and messages and add display properties in embeded rviz.
void MainWindow::addDisplays()
{
  std::pair<std::string, std::string> name;
  std::map<std::string, std::string> proper;
  QColor color;
  /*--------*/
  name.first  = "rviz/Path";
  name.second = "trace";
  proper.clear();
  proper.insert(std::pair<std::string, std::string>("Topic", "/trace"));
  proper.insert(std::pair<std::string, std::string>("Line Style", "Lines"));
  proper.insert(std::pair<std::string, std::string>("Alpha", "1"));
  color = QColor(255, 85, 255);
  ui->addCloud(name, proper, color);
  /*--------*/
  name.first  = "rviz/PointCloud2";
  name.second = "cloud";
  proper.clear();
  proper.insert(std::pair<std::string, std::string>("Topic", "/cloud"));
  proper.insert(std::pair<std::string, std::string>("Style", "Points"));
  proper.insert(std::pair<std::string, std::string>("Size (Pixels)", "1"));
  proper.insert(std::pair<std::string, std::string>("Alpha", "1"));
  proper.insert(std::pair<std::string, std::string>("Color Transformer", "AxisColor"));
  // color = QColor(164, 0, 0);
  color = QColor(250, 250, 250);
  ui->addCloud(name, proper, color); // addCloud
}

void MainWindow::on_viewport_clicked()
{
  // float x, y, z;
  // QString qs;

  // x = (float)(ui->render_panel->getCamera()->getRealPosition().x);
  // y = (float)(ui->render_panel->getCamera()->getRealPosition().y);
  // z = (float)(ui->render_panel->getCamera()->getRealPosition().z);
  // qs = "p: " + QString("%1").arg(x) + ", " + QString("%1").arg(y) + ", " + QString("%1").arg(z);
  // ulog(qs);

  // x = (float)(ui->render_panel->getCamera()->getRealOrientation().x);
  // y = (float)(ui->render_panel->getCamera()->getRealOrientation().y);
  // z = (float)(ui->render_panel->getCamera()->getRealOrientation().z);
  // qs = "o: " + QString("%1").arg(x) + ", " + QString("%1").arg(y) + ", " + QString("%1").arg(z);
  // ulog(qs);
}

// select file and publish refer curve line
void MainWindow::on_select_trace_clicked()
{
  // select file
  QString filepath = QFileDialog::getOpenFileName(this, "select trace", default_dir, "*.csv");
  pubers.updateTrace(filepath.toStdString());
}

// this function will perform first before window is closed when a close event occurs
void MainWindow::closeEvent(QCloseEvent *event)
{
  std::cout << "closing main window" << std::endl;
}

bool MainWindow::openAbout()
{
  QString text = "This is an example about how to merge rviz and qt UI.";
  QMessageBox::about(this, "about qt rviz", text);
  return true;
}
