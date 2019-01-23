/*
  Copyright 2016 Lucas Walter
*/
#ifndef ACTUATOR_BRIDGE_ACTUATOR_ESTOP_H
#define ACTUATOR_BRIDGE_ACTUATOR_ESTOP_H

#include <xr1controllerol/plugin.h>
#include <xr1controllerol/ui_actuator_estop.h>
#include <QWidget>

namespace rqt_example_cpp
{

class MyPlugin
  : public rqt_gui_cpp::Plugin
{
  Q_OBJECT
public:
  MyPlugin();
  virtual void initPlugin(qt_gui_cpp::PluginContext& context);
  virtual void shutdownPlugin();
  virtual void saveSettings(qt_gui_cpp::Settings& plugin_settings,
      qt_gui_cpp::Settings& instance_settings) const;
  virtual void restoreSettings(const qt_gui_cpp::Settings& plugin_settings,
      const qt_gui_cpp::Settings& instance_settings);

  // Comment in to signal that the plugin has a way to configure it
  // bool hasConfiguration() const;
  // void triggerConfiguration();
private:
  Ui::MyPluginWidget ui_;
  QWidget* widget_;
};
}  // namespace rqt_example_cpp
#endif  // RQT_EXAMPLE_CPP_MY_PLUGIN_H
