#ifndef UI_MAINWINDOW_H
#define UI_MAINWINDOW_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QLabel>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenu>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QWidget>

#include <QDebug>

#include <rviz/visualization_manager.h>
#include <rviz/render_panel.h>
#include <rviz/display.h>
#include <rviz/tool_manager.h>
#include <rviz/tool.h>
#include <rviz/default_plugin/view_controllers/orbit_view_controller.h>
#include <rviz/view_manager.h>

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
 public:
  QMenuBar *menuBar;
  QAction *about;

  QWidget *centralWidget;

  QLabel *cloud_size2;
  QLabel *cloud_size1;

  QPushButton *select_cloud;
  QPushButton *select_trace;


  // create a rviz container, which is QWidget
  rviz::RenderPanel *render_panel;
  // initialize rviz manager
  rviz::VisualizationManager *visual_manager;

  rviz::Display *display;


  void setupUi(QMainWindow *MainWindow, QString root_path = "../")
  {
    if (MainWindow->objectName().isEmpty())
    {
      MainWindow->setObjectName(QStringLiteral("MainWindow"));
    }
    MainWindow->setWindowModality(Qt::WindowModal);
    MainWindow->resize(1620, 890);
    MainWindow->showNormal();
    MainWindow->setWindowIcon(QIcon(root_path + "logo.ico"));


    QSize min_size(400, 500);
    MainWindow->setMinimumSize(min_size);

    centralWidget = new QWidget(MainWindow);
    centralWidget->setObjectName(QStringLiteral("centralWidget"));

    MainWindow->setCentralWidget(centralWidget);
    menuBar = new QMenuBar(MainWindow);
    menuBar->setObjectName(QStringLiteral("menuBar"));
    MainWindow->setMenuBar(menuBar);

    about = new QAction(MainWindow);
    about->setObjectName(QStringLiteral("about"));
    menuBar->addAction(about);

    select_trace = new QPushButton(centralWidget);
    select_trace->setObjectName(QStringLiteral("select_trace"));
    select_trace->setToolTip("select a refer border from cloud end");
    select_trace->setGeometry(QRect(10, 10, 90, 30));

    select_cloud = new QPushButton(centralWidget);
    select_cloud->setObjectName(QStringLiteral("select_cloud"));
    select_cloud->setGeometry(QRect(110, 10, 100, 30));

    cloud_size1 = new QLabel(centralWidget);
    cloud_size1->setObjectName(QStringLiteral("cloud_size1"));
    cloud_size1->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
    cloud_size1->setGeometry(QRect(220, 10, 70, 30));
    cloud_size2 = new QLabel(centralWidget);
    cloud_size2->setObjectName(QStringLiteral("cloud_size2"));
    cloud_size2->setStyleSheet(QLatin1String("border: 1px solid;\n"
                                             "border-color: rgb(52, 101, 164);"));
    cloud_size2->setAlignment(Qt::AlignRight | Qt::AlignTrailing | Qt::AlignVCenter);
    cloud_size2->setGeometry(QRect(300, 10, 70, 30));


    render_panel = new rviz::RenderPanel(centralWidget);
    render_panel->setGeometry(QRect(10, 50, MainWindow->width() - 20, MainWindow->height() - 85));

    visual_manager = new rviz::VisualizationManager(render_panel);
    /*
      set camera
    */
    // Ogre::Camera *camera = visual_manager->getSceneManager()->createCamera("cam");
    // camera->setPosition(67, 34, 113);
    // Ogre::Vector3 *ovec3;
    // ovec3 = new Ogre::Vector3(0.06, 0.08, 0.8);
    // camera->setOrientation(Ogre::Quaternion(ovec3));
    /**/

    // initialize camera. This statement will import zoom and translation to rviz
    render_panel->initialize(visual_manager->getSceneManager(), visual_manager);

    visual_manager->initialize();
    visual_manager->removeAllDisplays();
    visual_manager->startUpdate();

    // render_panel->setCamera(camera);

    rviz::Display *grid = visual_manager->createDisplay("rviz/Grid", "adjustable grid", true);
    // to prevent crash
    // ROS_ASSERT(grid != NULL);
    // set properties
    grid->subProp("Line Style")->setValue("Lines");
    grid->subProp("Color")->setValue(QColor(160, 160, 164));
    grid->subProp("Cell Size")->setValue("5");
    grid->subProp("Plane Cell Count")->setValue("100");

    rviz::Display *axis = visual_manager->createDisplay("rviz/Axes", "axis", true);
    // to prevent crash
    // ROS_ASSERT(axis != NULL);
    // set properties
    axis->subProp("Length")->setValue("5");
    axis->subProp("Radius")->setValue("1");
    axis->subProp("Alpha")->setValue("1");

    // set properties in leftside of rviz
    visual_manager->setFixedFrame("/map");


    retranslateUi(MainWindow);

    QMetaObject::connectSlotsByName(MainWindow);

    render_panel->setGeometry(QRect(10, 50, MainWindow->width() - 20, MainWindow->height() - 85));
    ;
  } // setupUi

  void retranslateUi(QMainWindow *MainWindow)
  {
    MainWindow->setWindowTitle(QApplication::translate("MainWindow", "qt rviz", Q_NULLPTR));
    about->setText(QApplication::translate("MainWindow", "about", Q_NULLPTR));

    cloud_size1->setText(QApplication::translate("MainWindow", "cloud size", Q_NULLPTR));
    cloud_size2->setText(QApplication::translate("MainWindow", "0,0", Q_NULLPTR));

    select_cloud->setText(QApplication::translate("MainWindow", "select cloud", Q_NULLPTR));
    select_trace->setText(QApplication::translate("MainWindow", "select trace", Q_NULLPTR));
  } // retranslateUi

  void addCloud(std::pair<std::string, std::string> name, std::map<std::string, std::string> proper, QColor color)
  {
    rviz::Display *dis = visual_manager->createDisplay(QString().fromStdString(name.first),
                                                       QString().fromStdString(name.second),
                                                       true);
    for (auto item : proper)
    {
      dis->subProp(QString().fromStdString(item.first))->setValue(QString().fromStdString(item.second));
    }

    dis->subProp("Color")->setValue(color);
    //   dis->subProp("Axis")->setValue("Z");
  }
};

namespace Ui
{
class MainWindow : public Ui_MainWindow
{
};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
