#include "bulldog_pad.h"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QTimer>
#include <QDebug>

#include <sstream>

namespace rviz_bulldog_commander
{

// 构造函数，初始化变量
BulldogPanel::BulldogPanel( QWidget* parent )
  : rviz::Panel( parent )
  , state( STATE_WAIT )
{
  tab = new QTabWidget();
  // Tab 1
  QWidget *tab_1 = new QWidget();
  button_auto = new QPushButton("Execute");
  QHBoxLayout* layout_1 = new QHBoxLayout;
  layout_1->addWidget(button_auto);
  tab_1->setLayout(layout_1);

  // Tab 2
  QWidget *tab_2 = new QWidget();
  // Button
  label_detect = new QLabel(QObject::tr("Detect"));
  label_navigation1 = new QLabel(QObject::tr("Navigation1"));
  label_pick_object = new QLabel(QObject::tr("Object"));
  label_arrive = new QLabel(QObject::tr("Arrive"));
  label_pick = new QLabel(QObject::tr("Pick"));
  label_place = new QLabel(QObject::tr("Place"));
  label_reset = new QLabel(QObject::tr("Reset"));
  label_say_hello = new QLabel(QObject::tr("Welcome"));
  button_detect = new QPushButton("Detect");
  button_navigation1 = new QPushButton("Navigation1");
  button_water = new QPushButton("Water");
  button_coke = new QPushButton("Coke");
  button_arrive_plan = new QPushButton("Plan");
  button_arrive = new QPushButton("Execute");
  button_pick_plan = new QPushButton("Plan");
  button_place_plan = new QPushButton("Plan");
  button_pick = new QPushButton("Execute");
  button_place = new QPushButton("Execute");
  button_reset = new QPushButton("Reset");
  button_power = new QPushButton("Power");
  button_say_hello = new QPushButton("Clear Scene");
  button_wave = new QPushButton("Shutdown");
  button_gripper_activate = new QPushButton("Gripper Activate");
  button_left_gripper_close = new QPushButton("Close Left Gripper");
  button_right_gripper_close = new QPushButton("Close Right Gripper");
  button_left_gripper_open = new QPushButton("Open Left Gripper");
  button_right_gripper_open = new QPushButton("Open Right Gripper");

  button_speech_wave = new QPushButton("Wave");
  button_speech_pick = new QPushButton("Pick");
  button_speech_place = new QPushButton("Place");

  label_display = new QLabel();

  // Button Layout
  //QVBoxLayout* button_layout = new QVBoxLayout;
  QGridLayout *button_layout = new QGridLayout();
  button_layout->addWidget(label_detect,1,0,1,1);
  button_layout->addWidget(button_detect,1,1,1,2);
  button_layout->addWidget(label_navigation1,0,0,1,1);
  button_layout->addWidget(button_navigation1,0,1,1,2);
  button_layout->addWidget(label_arrive,3,0,1,1);
  button_layout->addWidget(button_arrive_plan,3,1,1,1);
  button_layout->addWidget(button_arrive,3,2,1,1);
  button_layout->addWidget(label_pick,4,0,1,1);
  button_layout->addWidget(button_pick_plan,4,1,1,1);
  button_layout->addWidget(button_pick,4,2,1,1);
  button_layout->addWidget(label_pick_object,2,0,1,1);
  button_layout->addWidget(button_water,2,1,1,1);
  button_layout->addWidget(button_coke,2,2,1,1);
  button_layout->addWidget(label_place,5,0,1,1);
  button_layout->addWidget(button_place_plan,5,1,1,1);
  button_layout->addWidget(button_place,5,2,1,1);
  button_layout->addWidget(label_reset,6,0,1,1);
  button_layout->addWidget(button_reset,6,1,1,1);
  button_layout->addWidget(button_power,6,2,1,1);
  button_layout->addWidget(label_say_hello,7,0,1,1);
  button_layout->addWidget(button_say_hello,7,1,1,1);
  button_layout->addWidget(button_wave,7,2,1,1);
  button_layout->addWidget(label_display,8,0,1,3);

  // Display Label
  
  // Layout
  tab_2 -> setLayout(button_layout);
  
  QGridLayout *manual_layout = new QGridLayout();
  manual_layout->addWidget(button_gripper_activate,1,0,1,2);
  manual_layout->addWidget(button_left_gripper_close,2,0,1,1);
  manual_layout->addWidget(button_right_gripper_close,2,1,1,1);
  manual_layout->addWidget(button_left_gripper_open,3,0,1,1);
  manual_layout->addWidget(button_right_gripper_open,3,1,1,1);
  // Tab 3
  QWidget *tab_3 = new QWidget();
  tab_3->setLayout(manual_layout);

  QWidget *tab_4 = new QWidget();
  QGridLayout *speech_layout = new QGridLayout();

  tab_4->setLayout(speech_layout);
  speech_layout->addWidget(button_speech_wave,1,0,1,1);
  speech_layout->addWidget(button_speech_pick,2,0,1,1);
  speech_layout->addWidget(button_speech_place,3,0,1,1);

  // Add Tab Pages
  tab->addTab(tab_1, "Auto");
  tab->addTab(tab_2, "Semi-Auto");
  tab->addTab(tab_3, "Manual");
  tab->addTab(tab_4,"Speech Control");

  // Add Tab
  QHBoxLayout *layout = new QHBoxLayout();
  layout->addWidget(tab);
  setLayout(layout);

  // connect signal and slots, sender, signal, receiver, slots
  connect(button_auto, SIGNAL(clicked()),
          this, SLOT(button_auto_click()));
  connect(button_detect, SIGNAL(clicked()),
          this, SLOT(button_detect_click()));
  connect(button_pick_plan, SIGNAL(clicked()),
          this, SLOT(button_pick_plan_click()));
  connect(button_navigation1, SIGNAL(clicked()),
          this, SLOT(button_navigation1_click()));
  connect(button_pick, SIGNAL(clicked()),
          this, SLOT(button_pick_click()));
  connect(button_arrive, SIGNAL(clicked()),
          this, SLOT(button_arrive_click()));
  connect(button_arrive_plan, SIGNAL(clicked()),
          this, SLOT(button_arrive_plan_click()));
  connect(button_place, SIGNAL(clicked()),
          this, SLOT(button_place_click()));
  connect(button_place_plan, SIGNAL(clicked()),
          this, SLOT(button_place_plan_click()));
  connect(button_water, SIGNAL(clicked()),
          this, SLOT(button_water_click()));
  connect(button_coke, SIGNAL(clicked()),
          this, SLOT(button_coke_click()));
  connect(button_reset, SIGNAL(clicked()),
          this, SLOT(button_reset_click()));
  connect(button_power, SIGNAL(clicked()),
          this, SLOT(button_power_click()));
  connect(button_gripper_activate, SIGNAL(clicked()),
          this, SLOT(button_gripper_activate_click()));
  connect(button_left_gripper_open, SIGNAL(clicked()),
          this, SLOT(button_left_gripper_open_click()));
  connect(button_right_gripper_open, SIGNAL(clicked()),
          this, SLOT(button_right_gripper_open_click()));
  connect(button_left_gripper_close, SIGNAL(clicked()),
          this, SLOT(button_left_gripper_close_click()));
  connect(button_right_gripper_close, SIGNAL(clicked()),
          this, SLOT(button_right_gripper_close_click()));
  connect(button_say_hello, SIGNAL(clicked()),
          this, SLOT(button_say_hello_click()));
  connect(button_wave, SIGNAL(clicked()),
          this, SLOT(button_wave_click()));
  connect(button_speech_wave, SIGNAL(clicked()),
          this, SLOT(button_speech_wave_click()));
  connect(button_speech_pick, SIGNAL(clicked()),
          this, SLOT(button_speech_pick_click()));
  connect(button_speech_place, SIGNAL(clicked()),
          this, SLOT(button_speech_place_click()));

  // ROS
  pub = n.advertise<std_msgs::String>("plugin_command", 1);
  sub = n.subscribe("plugin_return", 1, &BulldogPanel::callback, this);
  left_gripper_pub = n.advertise<std_msgs::String>("left_gripper_signal", 1000);
  right_gripper_pub = n.advertise<std_msgs::String>("right_gripper_signal", 1000);
}
void BulldogPanel::pub_gripper(ros::Publisher *pub, std::string str){

  std_msgs::String msg;
  std::stringstream ss;
    ss << str;
    msg.data = ss.str();
    pub->publish(msg);
  ROS_INFO("gripper signal has been published!");
}
void BulldogPanel::callback(const std_msgs::String::ConstPtr& msg){
    ROS_INFO("%s", msg->data.c_str());
    std::string rec = msg->data;
    display = QString::fromStdString(rec);
    label_display->clear();
    label_display->setText(display);
    if(rec == "Detect succeed!")  state = STATE_DETECTED;
    else if(rec == "Navagation succeed");
    else if(rec == "Execute succeed!")  state = STATE_PICKED;
}

void BulldogPanel::button_auto_click(){
  // if(state == STATE_WAIT)
  // {
  //   //robot->main();
  // }
  if(ros::ok())
  {
    std_msgs::String msg;

    std::stringstream ss;
    ss << "AUTO";
    msg.data = ss.str();

    ROS_INFO("%s", msg.data.c_str());
    pub.publish(msg);
  }
}

void BulldogPanel::button_detect_click(){
  // if(state == STATE_WAIT)
  // {
  //   QString str;
  //   if(robot->detect(x,y,z)==true)
  //   {
  //     str = QString("x:%1\ny:%2\nz:%3\n").arg(x).arg(y).arg(z);
  //     state = STATE_DETECTED;
  //   }
  //   else
  //   {
  //     str = QString("Detected Failed!");
  //     state = WAIT;
  //   }
  //   label_display->setText(str);
  // }
  if(ros::ok())
  {
    label_display->clear();
    label_display->setText("Detecting");
    std_msgs::String msg;

    std::stringstream ss;
    ss << "DETECT";
    msg.data = ss.str();

    ROS_INFO("%s", msg.data.c_str());
    pub.publish(msg);

    ros::spinOnce();
  }
}
void BulldogPanel::button_pick_plan_click(){
  if(ros::ok())
  {
    label_display->clear();
    label_display->setText("Pick planning");
    std_msgs::String msg;

    std::stringstream ss;
    ss << "PICKPLAN";
    msg.data = ss.str();

    ROS_INFO("%s", msg.data.c_str());
    pub.publish(msg);
  }
}
void BulldogPanel::button_navigation1_click(){
  if(ros::ok())
  {
    label_display->clear();
    label_display->setText("Navigating");
    std_msgs::String msg;

    std::stringstream ss;
    ss << "NAVIGATION1";
    msg.data = ss.str();

    ROS_INFO("%s", msg.data.c_str());
    pub.publish(msg);
  }
}

void BulldogPanel::button_pick_click(){
  // if(state == STATE_DETECTED)
  // {
  //   QString str;
  //   if(robot->execute()==true)
  //   {
  //     str = QString("Pick Succeed!");
  //     state = PICKED;
  //   }
  //   else
  //   {
  //     str = QString("Pick Failed!");
  //     state = DETECTED;
  //   }
  //   label_display->setText(str);
  // }
  if(ros::ok())
  {
    label_display->clear();
    label_display->setText("Picking");
    std_msgs::String msg;

    std::stringstream ss;
    ss << "PICKEXECUTE";
    msg.data = ss.str();

    ROS_INFO("%s", msg.data.c_str());
    pub.publish(msg);
  }
}
void BulldogPanel::button_reset_click(){
  if(ros::ok())
  {
    label_display->clear();
    label_display->setText("Reseting");
    std_msgs::String msg;

    std::stringstream ss;
    ss << "RESET";
    msg.data = ss.str();

    ROS_INFO("%s", msg.data.c_str());
    pub.publish(msg);
  }
}
void BulldogPanel::button_power_click(){
  if(ros::ok())
  {
    label_display->clear();
    label_display->setText("Powering on");
    std_msgs::String msg;

    std::stringstream ss;
    ss << "POWER";
    msg.data = ss.str();

    ROS_INFO("%s", msg.data.c_str());
    pub.publish(msg);
  }
}
void BulldogPanel::button_arrive_click(){
  if(ros::ok())
  {
    label_display->clear();
    label_display->setText("Arriving");
    std_msgs::String msg;

    std::stringstream ss;
    ss << "ARRIVEEXECUTE";
    msg.data = ss.str();

    ROS_INFO("%s", msg.data.c_str());
    pub.publish(msg);
  }
}
void BulldogPanel::button_arrive_plan_click(){
  if(ros::ok())
  {
    label_display->clear();
    label_display->setText("Arrive Planning");
    std_msgs::String msg;

    std::stringstream ss;
    ss << "ARRIVEPLAN";
    msg.data = ss.str();

    ROS_INFO("%s", msg.data.c_str());
    pub.publish(msg);
  }
}
void BulldogPanel::button_place_click(){
  if(ros::ok())
  {
    label_display->clear();
    label_display->setText("Placing");
    std_msgs::String msg;

    std::stringstream ss;
    ss << "PLACEEXECUTE";
    msg.data = ss.str();

    ROS_INFO("%s", msg.data.c_str());
    pub.publish(msg);
  }
}
void BulldogPanel::button_place_plan_click(){
  if(ros::ok())
  {
    label_display->clear();
    label_display->setText("Place planning");
    std_msgs::String msg;

    std::stringstream ss;
    ss << "PLACEPLAN";
    msg.data = ss.str();

    ROS_INFO("%s", msg.data.c_str());
    pub.publish(msg);
  }
}
void BulldogPanel::button_water_click(){
  if(ros::ok())
  {
    label_display->clear();
    label_display->setText("Picking water bottle.");
    std_msgs::String msg;

    std::stringstream ss;
    ss << "WATER";
    msg.data = ss.str();

    ROS_INFO("%s", msg.data.c_str());
    pub.publish(msg);
  }
}

void BulldogPanel::button_coke_click(){
  if(ros::ok())
  {
    label_display->clear();
    label_display->setText("Picking coke.");
    std_msgs::String msg;

    std::stringstream ss;
    ss << "COKE";
    msg.data = ss.str();

    ROS_INFO("%s", msg.data.c_str());
    pub.publish(msg);
  }
}

void BulldogPanel::button_wave_click(){
  if(ros::ok())
  {
    label_display->clear();
    label_display->setText("Waving");
    std_msgs::String msg;

    std::stringstream ss;
    ss << "WAVE";
    msg.data = ss.str();

    ROS_INFO("%s", msg.data.c_str());
    pub.publish(msg);
  }
}

void BulldogPanel::button_say_hello_click(){
  if(ros::ok())
  {
    label_display->clear();
    label_display->setText("Clear_Scene");
    std_msgs::String msg;

    std::stringstream ss;
    ss << "SAY_HELLO";
    msg.data = ss.str();

    ROS_INFO("%s", msg.data.c_str());
    pub.publish(msg);
  }
}

void BulldogPanel::button_speech_wave_click(){
  if(ros::ok())
{
    label_display->clear();
    label_display->setText("Waving");
    std_msgs::String msg;

    std::stringstream ss;
    ss << "SPEECH_WAVE";
    msg.data = ss.str();

    ROS_INFO("%s", msg.data.c_str());
    pub.publish(msg);
  }
}

void BulldogPanel::button_speech_pick_click(){
  if(ros::ok())
  {
    label_display->clear();
    label_display->setText("Picking");
    std_msgs::String msg;

    std::stringstream ss;
    ss << "SPEECH_PICK";
    msg.data = ss.str();

    ROS_INFO("%s", msg.data.c_str());
    pub.publish(msg);
  }
}

void BulldogPanel::button_speech_place_click(){
  if(ros::ok())
  {
    label_display->clear();
    label_display->setText("Placing");
    std_msgs::String msg;

    std::stringstream ss;
    ss << "SPEECH_PLACE";
    msg.data = ss.str();

    ROS_INFO("%s", msg.data.c_str());
    pub.publish(msg);
  }
}

void BulldogPanel::button_gripper_activate_click(){
  pub_gripper(&left_gripper_pub,"a");
  pub_gripper(&right_gripper_pub,"a");
  ros::spinOnce();
}
void BulldogPanel::button_left_gripper_open_click(){
  pub_gripper(&left_gripper_pub,"o");
  ros::spinOnce();
}
void BulldogPanel::button_right_gripper_open_click(){
  pub_gripper(&right_gripper_pub,"o");
  ros::spinOnce();
}
void BulldogPanel::button_left_gripper_close_click(){
  pub_gripper(&left_gripper_pub,"K");
  ros::spinOnce();
}
void BulldogPanel::button_right_gripper_close_click(){
  pub_gripper(&right_gripper_pub,"K");
  ros::spinOnce();
}


} // end namespace rviz_bulldog_commander

// 声明此类是一个rviz的插件
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz_bulldog_commander::BulldogPanel,rviz::Panel)
// END_TUTORIAL
