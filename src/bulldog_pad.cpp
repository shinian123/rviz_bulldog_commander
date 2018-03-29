#include "bulldog_pad.h"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QTimer>
#include <QDebug>

namespace rviz_bulldog_commander
{

// 构造函数，初始化变量
BulldogPanel::BulldogPanel( QWidget* parent )
  : rviz::Panel( parent )
  // , is_detect( false )
  // , is_execute( false)
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
  button_detect = new QPushButton("Detect");
  button_move = new QPushButton("Move");
  button_pick = new QPushButton("Pick");
  // Button Layout
  QVBoxLayout* button_layout = new QVBoxLayout;
  button_layout->addWidget(button_detect);
  button_layout->addWidget(button_move);
  button_layout->addWidget(button_pick);
  // Display Label
  label_display = new QLabel();
  // Layout
  QHBoxLayout* layout_2 = new QHBoxLayout;
  layout_2->addLayout(button_layout);
  layout_2->addWidget(label_display);
  tab_2 -> setLayout(layout_2);

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
  connect(button_move, SIGNAL(clicked()),
          this, SLOT(button_move_click()));
  connect(button_pick, SIGNAL(clicked()),
          this, SLOT(button_pick_click()));
}

void BulldogPanel::button_auto_click(){
  label_display->setText("123");
}

void BulldogPanel::button_detect_click(){
  label_display->setText("456");
}

void BulldogPanel::button_move_click(){
  label_display->setText("789");
}

void BulldogPanel::button_pick_click(){
  label_display->setText("jqk");
}

} // end namespace rviz_bulldog_commander

// 声明此类是一个rviz的插件
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz_bulldog_commander::BulldogPanel,rviz::Panel)
// END_TUTORIAL
