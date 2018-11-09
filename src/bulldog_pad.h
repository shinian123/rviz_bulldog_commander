#ifndef BULLDOG_PAD_H
#define BULLDOG_PAD_H

//所需要包含的头文件
#include <ros/ros.h>
#include "std_msgs/String.h"
#include <ros/console.h>
#include <rviz/panel.h>   //plugin基类的头文件

#include <QPushButton>
#include <QLabel>
#include <QTabWidget>
#include <QString>

#include <string>
#ifndef Q_MOC_RUN
#include <moveit/macros/class_forward.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_interaction/robot_interaction.h>
#include <moveit/robot_interaction/interaction_handler.h>
#include <moveit/semantic_world/semantic_world.h>
#include <interactive_markers/interactive_marker_server.h>
#include <rviz/default_plugin/interactive_markers/interactive_marker.h>
#include <moveit_msgs/MotionPlanRequest.h>
#include <actionlib/client/simple_action_client.h>
#include <object_recognition_msgs/ObjectRecognitionAction.h>
#endif

#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <map>


namespace rviz_bulldog_commander
{
#define STATE_WAIT 0
#define STATE_DETECTED 1
#define STATE_PICKED 2

// 所有的plugin都必须是rviz::Panel的子类
class BulldogPanel: public rviz::Panel
{
// 后边需要用到Qt的信号和槽，都是QObject的子类，所以需要声明Q_OBJECT宏
Q_OBJECT
public:
  BulldogPanel( QWidget* parent = 0 );

protected:
  QTabWidget *tab;

  //Tab 1
  QPushButton *button_auto;

  // Tab 2
  QPushButton *button_detect;
  QPushButton *button_navigation1;
  QPushButton *button_water;
  QPushButton *button_coke;
  QPushButton *button_arrive;
  QPushButton *button_arrive_plan;
  QPushButton *button_pick;
  QPushButton *button_reset;
  QPushButton *button_pick_plan;
  QPushButton *button_place_plan;
  QPushButton *button_place;
  QPushButton *button_power;
  QPushButton *button_say_hello;
  QPushButton *button_wave;
  QPushButton *button_gripper_activate;
  QPushButton *button_left_gripper_close;
  QPushButton *button_right_gripper_close;
  QPushButton *button_left_gripper_open;
  QPushButton *button_right_gripper_open;

  QPushButton *button_speech_wave;
  QPushButton *button_speech_pick;
  QPushButton *button_speech_place;
  QLabel *label_display;
  QLabel *label_pick;
  QLabel *label_place;
  QLabel *label_arrive;
  QLabel *label_detect;
  QLabel *label_navigation1;
  QLabel *label_pick_object;
  QLabel *label_reset;
  QLabel *label_say_hello;
  QString display;

  // Tab 3

  // ROS
  ros::NodeHandle n;
  ros::Publisher pub;
  ros::Publisher left_gripper_pub;
  ros::Publisher right_gripper_pub;
  ros::Subscriber sub;




  void callback(const std_msgs::String::ConstPtr& msg);

  // Variables
  int state; //State Machine: 0-STATE_WAIT, 1-STATE_DETECTED, 2-STATE_PICKED

  

private Q_SLOTS:
  void button_auto_click();
  void button_detect_click();
  void button_navigation1_click();
  void button_water_click();
  void button_coke_click();
  void button_arrive_click();
  void button_arrive_plan_click();
  void button_pick_click();
  void button_pick_plan_click();
  void button_reset_click();
  void button_place_click();
  void button_place_plan_click();
  void button_power_click();
  void button_wave_click();
  void button_say_hello_click();
  void button_speech_wave_click();
  void button_speech_pick_click();
  void button_speech_place_click();
  void button_gripper_activate_click();
  void button_left_gripper_close_click();
  void button_right_gripper_close_click();
  void button_left_gripper_open_click();
  void button_right_gripper_open_click();
  void pub_gripper(ros::Publisher *pub, std::string str);
};



} // end namespace rviz_bulldog_commander

#endif // BULLDOG_PANEL_H
