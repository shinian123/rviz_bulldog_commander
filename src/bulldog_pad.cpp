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
  label_pick = new QLabel(QObject::tr("     Pick"));
  label_place = new QLabel(QObject::tr("    Place"));
  button_detect = new QPushButton("Detect");
  button_navigation1 = new QPushButton("Navigation");
  button_navigation2 = new QPushButton("Navigation");
  button_pick_plan = new QPushButton("Plan");
  button_place_plan = new QPushButton("Plan");
  button_pick = new QPushButton("Pick");
  button_place = new QPushButton("Place");
  button_reset = new QPushButton("Reset");

  label_display = new QLabel();

  // Button Layout
  //QVBoxLayout* button_layout = new QVBoxLayout;
  QGridLayout *button_layout = new QGridLayout();
  button_layout->addWidget(label_pick,0,0,1,1);
  button_layout->addWidget(button_detect,1,0,1,1);
  button_layout->addWidget(button_navigation1,2,0,1,1);
  button_layout->addWidget(button_pick_plan,3,0,1,1);
  button_layout->addWidget(button_pick,4,0,1,1);
  button_layout->addWidget(label_place,0,1,1,1);
  button_layout->addWidget(button_reset,1,1,1,1);
  button_layout->addWidget(button_navigation2,2,1,1,1);
  button_layout->addWidget(button_place_plan,3,1,1,1);
  button_layout->addWidget(button_place,4,1,1,1);

  button_layout->addWidget(label_display,5,0,1,2);

  // Display Label
  
  // Layout
  tab_2 -> setLayout(button_layout);

  // Tab 3
  QWidget *tab_3 = new QWidget();

  // Add Tab Pages
  tab->addTab(tab_1, "Auto");
  tab->addTab(tab_2, "Semi-Auto");
  tab->addTab(tab_3, "Manual");

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
  connect(button_place, SIGNAL(clicked()),
          this, SLOT(button_place_click()));
  connect(button_place_plan, SIGNAL(clicked()),
          this, SLOT(button_place_plan_click()));
  connect(button_navigation2, SIGNAL(clicked()),
          this, SLOT(button_navigation2_click()));
  connect(button_reset, SIGNAL(clicked()),
          this, SLOT(button_reset_click()));

  // ROS
  pub = n.advertise<std_msgs::String>("plugin_command", 1);
  sub = n.subscribe("plugin_return", 1, &BulldogPanel::callback, this);
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
    std_msgs::String msg;

    std::stringstream ss;
    ss << "PLAN";
    msg.data = ss.str();

    ROS_INFO("%s", msg.data.c_str());
    pub.publish(msg);
  }
}
void BulldogPanel::button_navigation1_click(){
  if(ros::ok())
  {
    std_msgs::String msg;

    std::stringstream ss;
    ss << "NAVIGATION";
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
    std_msgs::String msg;

    std::stringstream ss;
    ss << "EXECUTE";
    msg.data = ss.str();

    ROS_INFO("%s", msg.data.c_str());
    pub.publish(msg);
  }
}
void BulldogPanel::button_reset_click(){
  if(ros::ok())
  {
    std_msgs::String msg;

    std::stringstream ss;
    ss << "RESET";
    msg.data = ss.str();

    ROS_INFO("%s", msg.data.c_str());
    pub.publish(msg);
  }
}
void BulldogPanel::button_place_click(){}
void BulldogPanel::button_place_plan_click(){}
void BulldogPanel::button_navigation2_click(){}

} // end namespace rviz_bulldog_commander

// 声明此类是一个rviz的插件
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz_bulldog_commander::BulldogPanel,rviz::Panel)
// END_TUTORIAL
