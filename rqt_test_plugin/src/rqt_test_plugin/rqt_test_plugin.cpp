#include <rqt_test_plugin/rqt_test_plugin.h>

#include <pluginlib/class_list_macros.h>
#include <ros/master.h>

namespace rqt_test_plugin {


  test_plugin::test_plugin() :
    rqt_gui_cpp::Plugin()
  {
    setObjectName("test_plugin");
  }

  void rqt_test_plugin::test_plugin::initPlugin(qt_gui_cpp::PluginContext &context)
  {
    widget = new rqt_test_plugin_widget();
    context.addWidget(widget);
  }

  void rqt_test_plugin::test_plugin::shutdownPlugin()
  {

  }

  void rqt_test_plugin::test_plugin::saveSettings(qt_gui_cpp::Settings &plugin_settings, qt_gui_cpp::Settings &instance_settings) const
  {

  }

  void rqt_test_plugin::test_plugin::restoreSettings(const qt_gui_cpp::Settings &plugin_settings, const qt_gui_cpp::Settings &instance_settings)
  {

  }

} // end namespace rqt_test_plugin

PLUGINLIB_EXPORT_CLASS(rqt_test_plugin::test_plugin, rqt_gui_cpp::Plugin)
