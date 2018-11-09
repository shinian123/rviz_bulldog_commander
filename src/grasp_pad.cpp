#include "grasp_pad.h"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QTimer>
#include <QDebug>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/robot_state/conversions.h>

#include <std_srvs/Empty.h>
#include <sstream>

namespace rviz_bulldog_commander
{

// 构造函数，初始化变量
GraspPanel::GraspPanel( QWidget* parent )
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
  button_wave = new QPushButton("Stop");
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
  sub = n.subscribe("plugin_return", 1, &GraspPanel::callback, this);
  left_gripper_pub = n.advertise<std_msgs::String>("left_gripper_signal", 1000);
  right_gripper_pub = n.advertise<std_msgs::String>("right_gripper_signal", 1000);

  known_collision_objects_version_ = 0;

  planning_scene_publisher_ = nh_.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
  planning_scene_world_publisher_ = nh_.advertise<moveit_msgs::PlanningSceneWorld>("planning_scene_world", 1);

  //  object_recognition_trigger_publisher_ = nh_.advertise<std_msgs::Bool>("recognize_objects_switch", 1);
  object_recognition_client_.reset(new actionlib::SimpleActionClient<object_recognition_msgs::ObjectRecognitionAction>(
      OBJECT_RECOGNITION_ACTION, false));
  object_recognition_subscriber_ =
      nh_.subscribe("recognized_object_array", 1, &GraspPanel::listenDetectedObjects, this);

  try
  {
    move_group_.reset(new moveit::planning_interface::MoveGroup(opt, context_->getFrameManager()->getTFClientPtr(),
                                                                ros::WallDuration(30, 0)));
    if (planning_scene_storage_)
      move_group_->setConstraintsDatabase(ui_->database_host->text().toStdString(), ui_->database_port->value());
  }
  catch (std::runtime_error& ex)
  {
    ROS_ERROR("%s", ex.what());
  }
  if (move_group_)
  {
    move_group_->allowLooking(ui_->allow_looking->isChecked());
    move_group_->allowReplanning(ui_->allow_replanning->isChecked());
    moveit_msgs::PlannerInterfaceDescription desc;
    if (move_group_->getInterfaceDescription(desc))
      planning_display_->addMainLoopJob(boost::bind(&MotionPlanningFrame::populatePlannersList, this, desc));
    planning_display_->addBackgroundJob(boost::bind(&MotionPlanningFrame::populateConstraintsList, this),
                                        "populateConstraintsList");

    if (first_time_)
    {
      first_time_ = false;
      const planning_scene_monitor::LockedPlanningSceneRO& ps = planning_display_->getPlanningSceneRO();
      if (ps)
      {
        planning_display_->setQueryStartState(ps->getCurrentState());
        planning_display_->setQueryGoalState(ps->getCurrentState());
      }
    }
  }

  if (object_recognition_client_)
  {
    try
    {
      waitForAction(object_recognition_client_, nh_, ros::Duration(3.0), OBJECT_RECOGNITION_ACTION);
    }
    catch (std::runtime_error& ex)
    {
      //      ROS_ERROR("Object recognition action: %s", ex.what());
      object_recognition_client_.reset();
    }
  }
  try
  {
    planning_scene_interface_.reset(new moveit::planning_interface::PlanningSceneInterface());
  }
  catch (std::runtime_error& ex)
  {
    ROS_ERROR("%s", ex.what());
  }

  try
  {
    const planning_scene_monitor::LockedPlanningSceneRO& ps = planning_display_->getPlanningSceneRO();
    if (ps)
    {
      semantic_world_.reset(new moveit::semantic_world::SemanticWorld(ps));
    }
    else
      semantic_world_.reset();
    if (semantic_world_)
    {
      semantic_world_->addTableCallback(boost::bind(&GraspPanel::updateTables, this));
    }
  }
  catch (std::runtime_error& ex)
  {
    ROS_ERROR("%s", ex.what());
  }
}
void GraspPanel::pub_gripper(ros::Publisher *pub, std::string str){

  std_msgs::String msg;
  std::stringstream ss;
    ss << str;
    msg.data = ss.str();
    pub->publish(msg);
  ROS_INFO("gripper signal has been published!");
}
void GraspPanel::callback(const std_msgs::String::ConstPtr& msg){
    ROS_INFO("%s", msg->data.c_str());
    std::string rec = msg->data;
    display = QString::fromStdString(rec);
    label_display->clear();
    label_display->setText(display);
    if(rec == "Detect succeed!")  state = STATE_DETECTED;
    else if(rec == "Navagation succeed");
    else if(rec == "Execute succeed!")  state = STATE_PICKED;
}

void GraspPanel::button_auto_click(){
  // if(state == STATE_WAIT)
  // {
  //   //robot->main();
  // }
  int whichobject;
  float water_x,water_y,water_z,coke_x,coke_y,coke_z;
  nh_.getParam("/grasp_demo/object",whichobject);
  nh_.getParam("/grasp_demo/water_x",water_x);
  nh_.getParam("/grasp_demo/water_y",water_y);
  nh_.getParam("/grasp_demo/water_z",water_z);
  nh_.getParam("/grasp_demo/coke_x",coke_x);
  nh_.getParam("/grasp_demo/coke_y",coke_y);
  nh_.getParam("/grasp_demo/coke_z",coke_z);
  if(whichobject==1){
    target_pose2.position.x=water_x;
    target_pose2.position.z=water_z+0.05;
    target_pose2.position.y=water_y+0.18;
    target_pose2.orientation.x=1;
    target_pose2.orientation.y=0;
    target_pose2.orientation.z=0;
    target_pose2.orientation.w=0;
  }else{
    target_pose2.position.x=coke_x;
    target_pose2.position.z=coke_z+0.2;
    target_pose2.position.y=coke_y;
    target_pose2.orientation.x=0;
    target_pose2.orientation.y=-sqrt(2)/2;
    target_pose2.orientation.z=sqrt(2)/2;
    target_pose2.orientation.w=0;
  }
  
  ROS_INFO("Begin Planning!");
  int planning_times = 10;
  for(int i=0;i<planning_times;i++){
    bool success = computeArrivePlanAndExecute(target_pose2,0.1);
    ROS_INFO("Visualizing plan 1 (pose goal) %s",success?"SUCCEED":"FAILED");
    if(success)
      break;
  }

  target_pose2.position.y -= 0.05;
  for(int i=0;i<planning_times;i++){
    bool success = computeArrivePlanAndExecute(target_pose2,0.02);
    ROS_INFO("Visualizing plan 1 (pose goal) %s",success?"SUCCEED":"FAILED");
    if(success)
      break;
  }

  pub_gripper(&left_gripper_signal_pub,"o"); 
  sleep(2.0);



  target_pose2.position.z += 0.18;
  target_pose2.position.y += 0.18;
  target_pose2.position.x -=0.10;
  
  moveit_msgs::CollisionObject attached_object;
  
  /* The header must contain a valid TF frame*/
  attached_object.header.frame_id = move_group_->getPlanningFrame();
  /* The id of the object */
  attached_object.id = "bottle";

  geometry_msgs::Pose pose;
  pose.orientation.w = 1.0;
  if(whichobject==1){
    pose.position.x = water_x;
    pose.position.y = water_y;
  }else{
    pose.position.x = coke_x;
    pose.position.y = coke_y;
  }
  pose.position.z =0.325;

  /* Define a box to be attached */
  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.CYLINDER;
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = 0.235;
  primitive.dimensions[1] = 0.035;

  attached_object.primitives.push_back(primitive);
  attached_object.primitive_poses.push_back(pose);  
  attached_object.operation = attached_object.ADD;
  std::vector<moveit_msgs::CollisionObject> collision_objects;  
  collision_objects.push_back(attached_object);  
  planning_scene_interface_->addCollisionObjects(collision_objects);
  moveit_msgs::CollisionObject remove_object;
  remove_object.id = "cyl2";
  //remove_object.header.frame_id = "left_gripper_palm_link";
  remove_object.operation = remove_object.REMOVE;

  ROS_INFO("Attaching the object to the hand and removing it from the world.");
  move_group_->attachObject(attached_object.id);

  for(int i=0;i<planning_times;i++){
    bool success = computeArrivePlanAndExecute(target_pose2,0.08);
    ROS_INFO("Visualizing plan 1 (pose goal) %s",success?"SUCCEED":"FAILED");
    if(success)
      break;
  }

  std::vector<double> group_variable_values;
  group_variable_values.push_back(2.999290);
  group_variable_values.push_back(-2.236700);
  group_variable_values.push_back(-0.559131);
  group_variable_values.push_back(-2.830370);
  group_variable_values.push_back(-1.319326);
  group_variable_values.push_back(-1.608036);
  ROS_INFO("Start planning picking!");
  computeJointPlanAndExecute(group_variable_values);

  pub_gripper(&left_gripper_signal_pub,"o");


  move_group_->detachObject("botttle");
  group_variable_values.clear();
  group_variable_values.push_back(-0.802153889);
  group_variable_values.push_back(-1.344944779);
  group_variable_values.push_back(1.707468509674);
  group_variable_values.push_back(-3.133113686);
  group_variable_values.push_back(-0.5110691);
  group_variable_values.push_back(1.346010);

  ROS_INFO("Start reseting!");
  computeJointPlanAndExecute(group_variable_values);

}

void GraspPanel::button_detect_click(){
  
}
void GraspPanel::button_pick_plan_click(){
  
}
void GraspPanel::button_navigation1_click(){
  
}

void GraspPanel::button_pick_click(){
  
}
void GraspPanel::button_reset_click(){
  
}
void GraspPanel::button_power_click(){
  
}
void GraspPanel::button_arrive_click(){
  
}
void GraspPanel::button_arrive_plan_click(){
  
}
void GraspPanel::button_place_click(){
  
}
void GraspPanel::button_place_plan_click(){
  
}
void GraspPanel::button_water_click(){
  
}

void GraspPanel::button_coke_click(){
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

void GraspPanel::button_wave_click(){
  if (move_group_)
    move_group_->stop();
}

void GraspPanel::button_say_hello_click(){
  
}

void GraspPanel::button_speech_wave_click(){
  
}

void GraspPanel::button_speech_pick_click(){
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

void GraspPanel::button_speech_place_click(){
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

void GraspPanel::button_gripper_activate_click(){
  pub_gripper(&left_gripper_pub,"a");
  pub_gripper(&right_gripper_pub,"a");
  ros::spinOnce();
}
void GraspPanel::button_left_gripper_open_click(){
  pub_gripper(&left_gripper_pub,"o");
  ros::spinOnce();
}
void GraspPanel::button_right_gripper_open_click(){
  pub_gripper(&right_gripper_pub,"o");
  ros::spinOnce();
}
void GraspPanel::button_left_gripper_close_click(){
  pub_gripper(&left_gripper_pub,"K");
  ros::spinOnce();
}
void GraspPanel::button_right_gripper_close_click(){
  pub_gripper(&right_gripper_pub,"K");
  ros::spinOnce();
}

void GraspPanel::computeArrivePlanButtonClicked(geometry_msgs::Pose pose,double tolerance){
  if (!move_group_)
    return;


  configureForPlanning(pose,tolerance);
  current_plan_.reset(new moveit::planning_interface::MoveGroup::Plan());
  if (move_group_->plan(*current_plan_))
  {
    button_arrive_plan->setEnabled(true);

    // Success
    label_display->setText(QString("Time: ").append(QString::number(current_plan_->planning_time_, 'f', 3)));
  }
  else
  {
    current_plan_.reset();

    // Failure
    label_display->setText("Failed");
  }
  Q_EMIT planningFinished();
}

void GraspPanel::computeArriveExecuteButtonClicked(){
  if (move_group_ && current_plan_)
  {
    button_arrive->setEnabled(true);  // enable stopping
    bool success = move_group_->execute(*current_plan_);
  }
}

void GraspPanel::computeArrivePlanAndExecute(geometry_msgs::Pose pose,double tolerance){
  if (!move_group_)
    return;
  configureForPlanning(pose,tolerance);
  // move_group::move() on the server side, will always start from the current state
  // to suppress a warning, we pass an empty state (which encodes "start from current state")
  move_group_->setStartStateToCurrentState();
  bool success = move_group_->move();
}

void GraspPanel::computeJointPlanAndExecute(std::vector<double> v){
  if (!move_group_)
    return;
  configuerForJointPlanning(v);
  // move_group::move() on the server side, will always start from the current state
  // to suppress a warning, we pass an empty state (which encodes "start from current state")
  move_group_->setStartStateToCurrentState();
  bool success = move_group_->move();
}

void GraspPanel::configureWorkspace()
{
  robot_model::VariableBounds bx, by, bz;
  bx.position_bounded_ = by.position_bounded_ = bz.position_bounded_ = true;

  robot_model::JointModel::Bounds b(3);
  // bx.min_position_ = ui_->wcenter_x->value() - ui_->wsize_x->value() / 2.0;
  // bx.max_position_ = ui_->wcenter_x->value() + ui_->wsize_x->value() / 2.0;
  // by.min_position_ = ui_->wcenter_y->value() - ui_->wsize_y->value() / 2.0;
  // by.max_position_ = ui_->wcenter_y->value() + ui_->wsize_y->value() / 2.0;
  // bz.min_position_ = ui_->wcenter_z->value() - ui_->wsize_z->value() / 2.0;
  // bz.max_position_ = ui_->wcenter_z->value() + ui_->wsize_z->value() / 2.0;

  if (move_group_)
    // move_group_->setWorkspace(bx.min_position_, by.min_position_, bz.min_position_, bx.max_position_, by.max_position_,
    //                           bz.max_position_);
  planning_scene_monitor::PlanningSceneMonitorPtr psm = planning_display_->getPlanningSceneMonitor();
  // get non-const access to the kmodel and update planar & floating joints as indicated by the workspace settings
  if (psm && psm->getRobotModelLoader() && psm->getRobotModelLoader()->getModel())
  {
    const robot_model::RobotModelPtr& kmodel = psm->getRobotModelLoader()->getModel();
    const std::vector<robot_model::JointModel*>& jm = kmodel->getJointModels();
    for (std::size_t i = 0; i < jm.size(); ++i)
      if (jm[i]->getType() == robot_model::JointModel::PLANAR)
      {
        jm[i]->setVariableBounds(jm[i]->getName() + "/" + jm[i]->getLocalVariableNames()[0], bx);
        jm[i]->setVariableBounds(jm[i]->getName() + "/" + jm[i]->getLocalVariableNames()[1], by);
      }
      else if (jm[i]->getType() == robot_model::JointModel::FLOATING)
      {
        jm[i]->setVariableBounds(jm[i]->getName() + "/" + jm[i]->getLocalVariableNames()[0], bx);
        jm[i]->setVariableBounds(jm[i]->getName() + "/" + jm[i]->getLocalVariableNames()[1], by);
        jm[i]->setVariableBounds(jm[i]->getName() + "/" + jm[i]->getLocalVariableNames()[2], bz);
      }
  }
}

void GraspPanel::configureForPlanning(geometry_msgs::Pose pose,double tolerance)
{
  move_group_->setStartStateToCurrentState();
  move_group_->setPoseTarget(pose);
  move_group_->setPlanningTime(1.0);
  move_group_->setGoalTolerance(tolerance);
  move_group_->setNumPlanningAttempts(20);
  move_group_->setMaxVelocityScalingFactor(1.0);
  move_group_->setMaxAccelerationScalingFactor(1.0);
  configureWorkspace();
}

void GraspPanel::configuerForJointPlanning(std::vector<double> v){
  move_group_->setStartStateToCurrentState();
  move_group_->setJointValueTarget(v);
  move_group_->setPlanningTime(1.0);
  move_group_->setNumPlanningAttempts(20);
  move_group_->setMaxVelocityScalingFactor(1.0);
  move_group_->setMaxAccelerationScalingFactor(1.0);
  configureWorkspace();
}




} // end namespace rviz_bulldog_commander

// 声明此类是一个rviz的插件
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz_bulldog_commander::GraspPanel,rviz::Panel)
// END_TUTORIAL

