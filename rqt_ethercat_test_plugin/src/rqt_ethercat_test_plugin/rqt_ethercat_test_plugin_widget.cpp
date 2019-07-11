#include <rqt_ethercat_test_plugin/rqt_ethercat_test_plugin_widget.h>
#include "ui_rqt_ethercat_test_plugin_widget.h"
#include "iostream"

using namespace ros_ethercat_driver;

rqt_ethercat_test_plugin_widget::rqt_ethercat_test_plugin_widget(ros::NodeHandle& nh, QWidget *parent) :
  QWidget(parent),
  ui(new Ui::rqt_ethercat_test_plugin_widget),
  nh_(nh)
{
  ui->setupUi(this);
//  ui->info_display->
  ethercat_driver_ptr_.reset(new ros_ethercat_driver::RobotStateEtherCATHardwareInterface);
  ethercat_driver_ptr_->loadParameters(nh_);
  //  ui->desired_position->setRange(-100, 100);
//  ui->desired_velocity->setRange(-100, 100);
//  ui->desired_torque->setRange(-100, 100);
  ui->desired_position->setEnabled(true);
  ui->desired_velocity->setEnabled(false);
  ui->desired_torque->setEnabled(false);
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
  int last_index = 0;
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

              if(device_name == "Twitter" && last_index != i)
                {
                  ui->control_word->clear();
                  QStringList cws;
                  cws << "Shut Down"<< "Switch On" << "Switch On and Enable" << "Quick Stop"
                      <<"Fault Reset" << "Disable Operation" << "Enable Operation" << "Disable Voltage";
                  ui->control_word->addItems(cws);
                }
              if(device_name == "ANYdrive" && last_index != i)
                {
                  ui->control_word->clear();
                  QStringList cws;
                  cws << "Warm Reset"<< "Clear Errors to MotorOp" << "Standby to Configure" << "Configure to Standby"
                      << "MotorOp to Standby" << "Standby to MotorPreOp" << "ControlOp to MotorOp" << "MotorOp to ControlOp"
                      << "Clear Errors to Standby";
                  ui->control_word->addItems(cws);
                }
              last_index = i;
            }

            status_label[i]->setText(ethercat_driver_ptr_->getSlaveStatus(i, device_name.toStdString()));
            mode_label[i]->setText(ethercat_driver_ptr_->getSlaveMode(i, device_name.toStdString()));
//          ui->device_table->item(i,3)->setText(QString("Status"));//ethercat_driver_ptr_->getSlaveStatus(i, device_name.toStdString()));
//          ui->device_table->item(i,4)->setText(QString("Mode"));//ethercat_driver_ptr_->getSlaveMode(i, device_name.toStdString()));
//          ui->device_table->setItem(i,3, new QTableWidgetItem(ethercat_driver_ptr_->getSlaveStatus(i, device_name.toStdString())));
//          ui->device_table->setItem(i,4, new QTableWidgetItem(ethercat_driver_ptr_->getSlaveMode(i, device_name.toStdString())));
//          ROS_INFO("updated NO.%d Motor named '%s'.",i,device_name.toStdString().c_str());

        }
      rate.sleep();
    }

}



void rqt_ethercat_test_plugin_widget::on_info_display_customContextMenuRequested(const QPoint &pos)
{

}




void rqt_ethercat_test_plugin_widget::on_connect_clicked()
{
  QString PDOType = ui->pdo_type->currentText();
  ethercat_driver_ptr_->setPDOType("ANYdrive", PDOType.toStdString());
  ethercat_driver_ptr_->setPDOType("Twitter", PDOType.toStdString());
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
  ethercat_driver_ptr_->createEtherCATCheckThread();
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
      ui->device_table->insertRow(i);
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
  displayOutputInfos("green", "Connected " + QString::number(ethercat_driver_ptr_->getNumberOfSlaves()) + " Slaves");

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
  ui->device_table->clear();

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
              ethercat_driver_ptr_->setCommand(i, mode.toStdString(), command);
              ros::Duration delay(0.1);
              delay.sleep();
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

void rqt_ethercat_test_plugin_widget::on_readSDO_clicked()
{
  uint8 *u8;
  int8 *i8;
  uint16 *u16;
  int16 *i16;
  uint32 *u32;
  int32 *i32;
  uint64 *u64;
  int64 *i64;
  float *sr;
  double *dr;
  char es[32];
  ui->SDOResult->clear();
  for(int i = 0;i<motor_chooser.size();i++)
    {
      if(motor_chooser[i]->checkState() == Qt::CheckState::Checked)
        {
          QString number_system = ui->number_system->currentText();
          int base;
          if(number_system == "Binary")
            base = 2;
          if(number_system == "Hex")
            base = 16;
          if(number_system == "Dec")
            base = 10;
          QString data_type = ui->SDOType->currentText();
          bool ok;
          uint16 id = i + 1;
          int index = ui->CoEIndex->text().toShort(&ok, 16);
          int subindex = ui->CoESubIndex->text().toShort(&ok, 16);
          std::cout<<"index: "<<index<<"subindex: "<<subindex<<std::endl;
          char result[128]  = " ";
          ethercat_driver_ptr_->readSDO(id, index, subindex, result);
          if(data_type == "STRING")
            ui->SDOResult->setText(QString(result));
          if(data_type == "UINT8")
            {
              u8 = (uint8*)&result[0];
              ui->SDOResult->setText(QString::number(*u8, base));
            }
          if(data_type == "UINT16")
            {
              u16 = (uint16*)&result[0];
              ui->SDOResult->setText(QString::number(*u16, base));
            }
          if(data_type == "UINT32")
            {
              u32 = (uint32*)&result[0];
              ui->SDOResult->setText(QString::number(*u32, base));
            }
          if(data_type == "INT8")
            {
              i8 = (int8*)&result[0];
              ui->SDOResult->setText(QString::number(*i8, base));
            }
          if(data_type == "INT16")
            {
              i16 = (int16*)&result[0];
              ui->SDOResult->setText(QString::number(*i16, base));
            }
          if(data_type == "INT32")
            {
              i32 = (int32*)&result[0];
              ui->SDOResult->setText(QString::number(*i32, base));
            }



        }
    }
}

void rqt_ethercat_test_plugin_widget::on_writeSDO_clicked()
{
  uint8 u8;
  int8 i8;
  uint16 u16;
  int16 i16;
  uint32 u32;
  int32 i32;
  uint64 u64;
  int64 i64;
  float sr;
  double dr;
  const char* es;
  for(int i = 0;i<motor_chooser.size();i++)
    {
      if(motor_chooser[i]->checkState() == Qt::CheckState::Checked)
        {
          QString number_system = ui->number_system->currentText();
          int base;
          if(number_system == "Binary")
            base = 2;
          if(number_system == "Hex")
            base = 16;
          if(number_system == "Dec")
            base = 10;
          QString data_type = ui->SDOType->currentText();
          bool ok;
          uint16 id = i + 1;
          int index = ui->CoEIndex->text().toShort(&ok, 16);
          int subindex = ui->CoESubIndex->text().toShort(&ok, 16);
          std::cout<<"index: "<<index<<" subindex: "<<subindex<<std::endl;

          if(data_type == "STRING")
            {
              es = ui->SDOResult->text().toStdString().c_str();
              std::cout<<"value : "<<es<<std::endl;
              ethercat_driver_ptr_->writeSDO(id, index, subindex, &es, sizeof(es));
            }
          if(data_type == "UINT8")
            {
              u8 = (uint8)ui->SDOResult->text().toShort(&ok, base);
              std::cout<<"value : "<<u8<<std::endl;
              ethercat_driver_ptr_->writeSDO(id, index, subindex, &u8, sizeof(u8));
            }
          if(data_type == "UINT16")
            {
              u16 = ui->SDOResult->text().toUShort(&ok, base);
              std::cout<<"value : "<<u16<<std::endl;
              ethercat_driver_ptr_->writeSDO(id, index, subindex, &u16, sizeof(u16));
            }
          if(data_type == "UINT32")
            {
              u32 = ui->SDOResult->text().toUInt(&ok, base);
              std::cout<<"value : "<<u32<<std::endl;
              ethercat_driver_ptr_->writeSDO(id, index, subindex, &u32, sizeof(u32));
            }
          if(data_type == "INT8")
            {
              i8 = (int8)ui->SDOResult->text().toShort(&ok, base);
              std::cout<<"value : "<<i8<<std::endl;
              ethercat_driver_ptr_->writeSDO(id, index, subindex, &i8, sizeof(i8));
            }
          if(data_type == "INT16")
            {
              i16 = ui->SDOResult->text().toShort(&ok, base);
              std::cout<<"value : "<<i16<<std::endl;
              ethercat_driver_ptr_->writeSDO(id, index, subindex, &i16, sizeof(i16));
            }
          if(data_type == "INT32")
            {
              i32 = ui->SDOResult->text().toInt(&ok, base);
              std::cout<<"value : "<<i32<<std::endl;
              ethercat_driver_ptr_->writeSDO(id, index, subindex, &i32, sizeof(i32));
            }

        }
    }
}

void rqt_ethercat_test_plugin_widget::on_mode_of_operation_currentIndexChanged(const QString &arg1)
{
    if(arg1 == "Position")
      {
        ui->desired_position->setEnabled(true);
        ui->desired_velocity->setEnabled(false);
        ui->desired_torque->setEnabled(false);
      }
    if(arg1 == "Velocity")
      {
        ui->desired_position->setEnabled(false);
        ui->desired_velocity->setEnabled(true);
        ui->desired_torque->setEnabled(false);
      }
    if(arg1 == "Torque")
      {
        ui->desired_position->setEnabled(false);
        ui->desired_velocity->setEnabled(false);
        ui->desired_torque->setEnabled(true);
      }
}
