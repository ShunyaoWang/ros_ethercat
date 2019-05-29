#include <rqt_ethercat_test_plugin/rqt_ethercat_test_plugin_widget.h>
#include "ui_rqt_ethercat_test_plugin_widget.h"
#include "iostream"

using namespace ros_ethercat_driver;

rqt_ethercat_test_plugin_widget::rqt_ethercat_test_plugin_widget(QWidget *parent) :
  QWidget(parent),
  ui(new Ui::rqt_ethercat_test_plugin_widget)
{
  ui->setupUi(this);
//  ui->info_display->
  ethercat_driver_ptr_.reset(new ros_ethercat_driver::RobotStateEtherCATHardwareInterface);
  ui->desired_position->setRange(-100, 100);
  ui->desired_velocity->setRange(-100, 100);
  ui->desired_torque->setRange(-30, 30);
}

rqt_ethercat_test_plugin_widget::~rqt_ethercat_test_plugin_widget()
{
  delete ui;
}

void rqt_ethercat_test_plugin_widget::updateFeedback()
{
  ROS_INFO("In update Feeedback Loop");
  ros::Rate rate(50);
  Eigen::Vector3d joint_feedback;
  while (ros::ok()&&is_connected) {

      for(int i = 0;i<motor_chooser.size();i++)
        {
          QString device_name = ui->device_table->item(i,1)->text();
//          ROS_INFO("update NO.%d Motor named '%s'.",i,device_name.toStdString().c_str());
          if(motor_chooser[i]->checkState() == Qt::CheckState::Checked)
            {
              joint_feedback = ethercat_driver_ptr_->getJointFeedback(i, device_name.toStdString());
              ui->actual_position->setText(QString::number(joint_feedback(0)));
              ui->actual_velocity->setText(QString::number(joint_feedback(1)));
              ui->actual_torque->setText(QString::number(joint_feedback(2)));
            }

            status_label[i]->setText(ethercat_driver_ptr_->getSlaveStatus(i, device_name.toStdString()));
            mode_label[i]->setText(ethercat_driver_ptr_->getSlaveMode(i, device_name.toStdString()));
//          ui->device_table->item(i,3)->setText(QString("Status"));//ethercat_driver_ptr_->getSlaveStatus(i, device_name.toStdString()));
//          ui->device_table->item(i,4)->setText(QString("Mode"));//ethercat_driver_ptr_->getSlaveMode(i, device_name.toStdString()));
//          ui->device_table->setItem(i,3, new QTableWidgetItem(ethercat_driver_ptr_->getSlaveStatus(i, device_name.toStdString())));
//          ui->device_table->setItem(i,4, new QTableWidgetItem(ethercat_driver_ptr_->getSlaveMode(i, device_name.toStdString())));
//          ROS_INFO("update NO.%d Motor named '%s'.",i,device_name.toStdString().c_str());

        }
      rate.sleep();
    }

}

void rqt_ethercat_test_plugin_widget::on_Mode_activated(int index)
{
  std::cout<<"Try to change mode index"<<index<<std::endl;
}

void rqt_ethercat_test_plugin_widget::on_info_display_customContextMenuRequested(const QPoint &pos)
{

}

void rqt_ethercat_test_plugin_widget::on_Mode_activated(const QString &arg1)
{

//  std::cout<<"Try to change mode to"<<*arg1.data()<<std::endl;
}


void rqt_ethercat_test_plugin_widget::on_connect_clicked()
{
  QString PDOType = ui->pdo_type->currentText();
  ethercat_driver_ptr_->setPDOType("ANYdrive", PDOType.toStdString());
  displayOutputInfos("green", "Choose PDO TYPE :"+PDOType);
  QString ifname = ui->ethernet_port_input->text();
  if(ifname.size()==0)
    {
      displayOutputInfos("red", "Please Input the name of Etherner Port");
      return;
    }
  displayOutputInfos("green", "Choose Port "+ifname+" for EtherCAT");
  displayOutputInfos("green", "Try to Connect ......");

  std::string ifnamestr = ifname.toStdString();
  ethercat_driver_ptr_->ifname = (char *)ifnamestr.c_str();

  if(!ethercat_driver_ptr_->EtherCATInit())
    {
      displayOutputInfos("red", "Failed to initialize EtherCAT Conmunication !!!!!!!!!");
      return;
    }
  displayOutputInfos("green", "Connected Successfully!");
  ethercat_driver_ptr_->createEtherCATLoopThread();

  ui->device_table->setItem(0,0, new QTableWidgetItem(QString::number(ethercat_driver_ptr_->getSlaveAddress(1), 16)));
  ui->device_table->setItem(0,1, new QTableWidgetItem(ethercat_driver_ptr_->getSlaveName(1)));
  ui->device_table->setItem(0,2, new QTableWidgetItem(ethercat_driver_ptr_->getSlaveECState(1)));
  status_label.push_back(new QLabel);
  mode_label.push_back(new QLabel);
  ui->device_table->setCellWidget(0,3, status_label[0]);
  ui->device_table->setCellWidget(0,4, mode_label[0]);

  motor_chooser.push_back(new QCheckBox());
  ui->device_table->setCellWidget(0,5,motor_chooser[0]);

  for(int i = 1;i<ethercat_driver_ptr_->getNumberOfSlaves();i++)
    {
      ui->device_table->insertRow(1);
      ui->device_table->setItem(i,0, new QTableWidgetItem(QString::number(ethercat_driver_ptr_->getSlaveAddress(i+1), 16)));
      ui->device_table->setItem(i,1, new QTableWidgetItem(ethercat_driver_ptr_->getSlaveName(i+1)));
      ui->device_table->setItem(i,2, new QTableWidgetItem(ethercat_driver_ptr_->getSlaveECState(i+1)));

      status_label.push_back(new QLabel);
      mode_label.push_back(new QLabel);
      ui->device_table->setCellWidget(i,3, status_label[i]);
      ui->device_table->setCellWidget(i,4, mode_label[i]);
      motor_chooser.push_back(new QCheckBox());
      ui->device_table->setCellWidget(i,5,motor_chooser[i]);
    }

//  ui->device_table->resize(1,3);
    is_connected = true;
    updateFeedbackThread_ = boost::thread(boost::bind(&rqt_ethercat_test_plugin_widget::updateFeedback, this));
//    updateFeedbackThread_.join();


}

void rqt_ethercat_test_plugin_widget::displayOutputInfos(const std::string &color,
                                                const QString &context)
{

  if(color == "red")
    {
      ui->info_display->setTextColor(QColor(255,0,0));
      ui->info_display->insertPlainText(QString("ERROR : ") + context + QString("\n"));
    }
  if(color == "green")
    {
      ui->info_display->setTextColor(QColor(0,255,0));
      ui->info_display->insertPlainText(QString("INFO : ") + context + QString("\n"));
    }
  if(color == "yellow")
    {
      ui->info_display->setTextColor(QColor(255,255,0));
      ui->info_display->insertPlainText(QString("WARN : ") + context + QString("\n"));
    }

//  ui->info_display->insertPlainText(context + QString("\n"));
}

void rqt_ethercat_test_plugin_widget::on_disconnect_clicked()
{
  is_connected = false;
  displayOutputInfos("green", "Try to disconnect ......");
  motor_chooser.clear();
  status_label.clear();
  mode_label.clear();

  ethercat_driver_ptr_->shutdown();

  displayOutputInfos("green", "Disconnected Successfully");
}

void rqt_ethercat_test_plugin_widget::on_set_mode_clicked()
{
  QString cw = ui->control_word->currentText();

  for(int i = 0;i<motor_chooser.size();i++)
    {
      if(motor_chooser[i]->checkState() == Qt::CheckState::Checked)
        {
          QString device_name = ui->device_table->item(i,1)->text();
          ethercat_driver_ptr_->setSlaveCW(i, device_name.toStdString(),
                                             cw.toStdString());
          displayOutputInfos("green", "Set "+cw+" for "+device_name);
        }
    }


}

void rqt_ethercat_test_plugin_widget::on_send_clicked()
{
  QString mode = ui->mode_of_operation->currentText();
  double command = 0;
  for(int i = 0;i<motor_chooser.size();i++)
    {
      if(motor_chooser[i]->checkState() == Qt::CheckState::Checked)
        {
          if(mode == "Position")
            {
              command = ui->desired_position->value();
            }
          else if (mode == "Velocity") {
              command = ui->desired_velocity->value();
            }
          else if (mode == "Torque") {
              command = ui->desired_torque->value();
            }
          ethercat_driver_ptr_->setCommand(i, mode.toStdString(), command);
          displayOutputInfos("green", "Set "+mode+" commannd : " + QString::number(command));
        }
    }
}

void rqt_ethercat_test_plugin_widget::on_stop_clicked()
{
  for(int i = 0;i<motor_chooser.size();i++)
    {
      if(motor_chooser[i]->checkState() == Qt::CheckState::Checked)
        {
          ethercat_driver_ptr_->setCommand(i, "Freeze", 0);
          QString device_name = ui->device_table->item(i,1)->text();
          ethercat_driver_ptr_->setSlaveCW(i, device_name.toStdString(),
                                             "ControlOp to MotorOp");

          displayOutputInfos("green", "STOP The Motor!");
        }
    }
}
