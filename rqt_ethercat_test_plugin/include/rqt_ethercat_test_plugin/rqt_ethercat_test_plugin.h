#ifndef TEST_PLUGIN
#define TEST_PLUGIN

#include <rqt_gui_cpp/plugin.h>
#include <std_msgs/String.h>

#include <rqt_ethercat_test_plugin/rqt_ethercat_test_plugin_widget.h>

namespace rqt_ethercat_test_plugin {

  class EtherCATTestPlugin : public rqt_gui_cpp::Plugin
  {
  public:
    EtherCATTestPlugin();

    void initPlugin(qt_gui_cpp::PluginContext& context) override;
    void shutdownPlugin() override;

    void saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const override;
    void restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings) override;

  private:
    rqt_ethercat_test_plugin_widget *widget = nullptr;
  };

}

#endif // TEST_PLUGIN
