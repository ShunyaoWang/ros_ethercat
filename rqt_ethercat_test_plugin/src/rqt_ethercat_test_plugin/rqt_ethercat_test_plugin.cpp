#include <rqt_ethercat_test_plugin/rqt_ethercat_test_plugin.h>

#include <pluginlib/class_list_macros.h>
#include <ros/master.h>

namespace rqt_ethercat_test_plugin {


  EtherCATTestPlugin::EtherCATTestPlugin() :
    rqt_gui_cpp::Plugin()
  {
    setObjectName("EtherCATTestPlugin");
  }

  void rqt_ethercat_test_plugin::EtherCATTestPlugin::initPlugin(qt_gui_cpp::PluginContext &context)
  {
    widget = new rqt_ethercat_test_plugin_widget(getNodeHandle());
    context.addWidget(widget);
  }

  void rqt_ethercat_test_plugin::EtherCATTestPlugin::shutdownPlugin()
  {

  }

  void rqt_ethercat_test_plugin::EtherCATTestPlugin::saveSettings(qt_gui_cpp::Settings &plugin_settings, qt_gui_cpp::Settings &instance_settings) const
  {

  }

  void rqt_ethercat_test_plugin::EtherCATTestPlugin::restoreSettings(const qt_gui_cpp::Settings &plugin_settings, const qt_gui_cpp::Settings &instance_settings)
  {

  }

} // end namespace rqt_ethercat_test_plugin

PLUGINLIB_EXPORT_CLASS(rqt_ethercat_test_plugin::EtherCATTestPlugin, rqt_gui_cpp::Plugin)
