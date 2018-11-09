#ifndef GRASP_PAD_H
#define GRASP_PAD_H

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

namespace rviz_bulldog_commander
{
#define STATE_WAIT 0
#define STATE_DETECTED 1
#define STATE_PICKED 2

// 所有的plugin都必须是rviz::Panel的子类
class GraspPanel: public rviz::Panel
{
// 后边需要用到Qt的信号和槽，都是QObject的子类，所以需要声明Q_OBJECT宏
Q_OBJECT
public:
  GraspPanel( QWidget* parent = 0 );

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

  static const int ITEM_TYPE_SCENE = 1;
  static const int ITEM_TYPE_QUERY = 2;

  void constructPlanningRequest(moveit_msgs::MotionPlanRequest& mreq);

  void updateSceneMarkers(float wall_dt, float ros_dt);

  void updateExternalCommunication();

  MotionPlanningDisplay* planning_display_;
  rviz::DisplayContext* context_;

  moveit::planning_interface::MoveGroupPtr move_group_;
  moveit::planning_interface::PlanningSceneInterfacePtr planning_scene_interface_;
  moveit::semantic_world::SemanticWorldPtr semantic_world_;

  moveit::planning_interface::MoveGroup::PlanPtr current_plan_;
  moveit_warehouse::PlanningSceneStoragePtr planning_scene_storage_;
  moveit_warehouse::ConstraintsStoragePtr constraints_storage_;
  moveit_warehouse::RobotStateStoragePtr robot_state_storage_;

  boost::shared_ptr<rviz::InteractiveMarker> scene_marker_;

  typedef std::map<std::string, moveit_msgs::RobotState> RobotStateMap;
  typedef std::pair<std::string, moveit_msgs::RobotState> RobotStatePair;
  RobotStateMap robot_states_;

  ros::Subscriber plan_subscriber_;
  ros::Subscriber execute_subscriber_;
  ros::Subscriber update_start_state_subscriber_;
  ros::Subscriber update_goal_state_subscriber_;
  // General
  void changePlanningGroupHelper();
  void importResource(const std::string& path);
  void loadStoredStates(const std::string& pattern);

  void remotePlanCallback(const std_msgs::EmptyConstPtr& msg);
  void remoteExecuteCallback(const std_msgs::EmptyConstPtr& msg);
  void remoteUpdateStartStateCallback(const std_msgs::EmptyConstPtr& msg);
  void remoteUpdateGoalStateCallback(const std_msgs::EmptyConstPtr& msg);

  /* Selects or unselects a item in a list by the item name */
  void setItemSelectionInList(const std::string& item_name, bool selection, QListWidget* list);

  ros::NodeHandle nh_;
  ros::Publisher planning_scene_publisher_;
  ros::Publisher planning_scene_world_publisher_;

  collision_detection::CollisionWorld::ObjectConstPtr scaled_object_;

  std::vector<std::pair<std::string, bool> > known_collision_objects_;
  long unsigned int known_collision_objects_version_;
  bool first_time_;
  ros::ServiceClient clear_octomap_service_client_;


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
  void computeArrivePlanAndExecute(geometry_msgs::Pose pose,double tolerance);
  void computeJointPlanAndExecute(std::vector<double> v);
  void computeArrivePlanButtonClicked(geometry_msgs::Pose pose,double tolerance);
  void computeArriveExecuteButtonClicked();
  void GraspPanel::configuerForJointPlanning(std::vector<double> v);
  void GraspPanel::configureForPlanning(geometry_msgs::Pose pose,double tolerance);
  void GraspPanel::configureWorkspace();
};

// \todo THIS IS REALLY BAD. NEED TO MOVE THIS AND RELATED FUNCTIONALITY OUT OF HERE
template <typename T>
void GraspPanel::waitForAction(const T& action, const ros::NodeHandle& node_handle,
                                        const ros::Duration& wait_for_server, const std::string& name)
{
  ROS_DEBUG("Waiting for MoveGroup action server (%s)...", name.c_str());

  // in case ROS time is published, wait for the time data to arrive
  ros::Time start_time = ros::Time::now();
  while (start_time == ros::Time::now())
  {
    ros::WallDuration(0.01).sleep();
    ros::spinOnce();
  }

  // wait for the server (and spin as needed)
  if (wait_for_server == ros::Duration(0, 0))
  {
    // wait forever until action server connects
    while (node_handle.ok() && !action->isServerConnected())
    {
      ros::WallDuration(0.02).sleep();
      ros::spinOnce();
    }
  }
  else
  {
    // wait for a limited amount of non-simulated time
    ros::WallTime final_time = ros::WallTime::now() + ros::WallDuration(wait_for_server.toSec());
    while (node_handle.ok() && !action->isServerConnected() && final_time > ros::WallTime::now())
    {
      ros::WallDuration(0.02).sleep();
      ros::spinOnce();
    }
  }

  if (!action->isServerConnected())
    throw std::runtime_error("Unable to connect to move_group action server within allotted time");
  else
    ROS_DEBUG("Connected to '%s'", name.c_str());
};

} // end namespace rviz_bulldog_commander

#endif // BULLDOG_PANEL_H
