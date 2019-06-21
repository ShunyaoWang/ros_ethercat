#ifndef rqt_ethercat_test_plugin_WIDGET
#define rqt_ethercat_test_plugin_WIDGET

#include <QWidget>
#include <QCheckBox>
#include <QLabel>

#include "ros/ros.h"
#include "ros_ethercat_driver/hardware_interface/ros_ethercat_hardware_interface.hpp"

#include "string"
#include "eigen3/Eigen/Eigen"

namespace Ui {
  class rqt_ethercat_test_plugin_widget;
}

class rqt_ethercat_test_plugin_widget : public QWidget
{
  Q_OBJECT

public:
  explicit rqt_ethercat_test_plugin_widget(QWidget *parent = nullptr);
  ~rqt_ethercat_test_plugin_widget();
private slots:

  void on_info_display_customContextMenuRequested(const QPoint &pos);

  void on_connect_clicked();

  void on_disconnect_clicked();

  void on_set_mode_clicked();

  void on_send_clicked();

  void on_stop_clicked();

  void on_readSDO_clicked();

  void on_writeSDO_clicked();

  void on_mode_of_operation_currentIndexChanged(const QString &arg1);

private:
  Ui::rqt_ethercat_test_plugin_widget *ui;

  std::vector<QCheckBox*> motor_chooser;

  std::vector<QLabel*> status_label, mode_label;

  ros_ethercat_driver::RobotStateEtherCATHardwareInterfacePtr ethercat_driver_ptr_;

private:
  void displayOutputInfos(const std::string &color, const QString &context);

  void updateFeedback();

  boost::thread updateFeedbackThread_;

  bool is_connected;

};

#endif // TEST_PLUGIN_WIDGET
