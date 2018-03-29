#ifndef BULLDOG_PAD_H
#define BULLDOG_PAD_H

//所需要包含的头文件
#include <ros/ros.h>
#include <ros/console.h>
#include <rviz/panel.h>   //plugin基类的头文件

#include <QPushButton>
#include <QLabel>
#include <QTabWidget>

namespace rviz_bulldog_commander
{
// 所有的plugin都必须是rviz::Panel的子类
class BulldogPanel: public rviz::Panel
{
// 后边需要用到Qt的信号和槽，都是QObject的子类，所以需要声明Q_OBJECT宏
Q_OBJECT
public:
  BulldogPanel( QWidget* parent = 0 );

private:
  QTabWidget *tab;

  //Tab 1
  QPushButton *button_auto;

  // Tab 2
  QPushButton *button_detect;
  QPushButton *button_move;
  QPushButton *button_pick;
  QLabel *label_display;

private Q_SLOTS:
  void button_auto_click();
  void button_detect_click();
  void button_move_click();
  void button_pick_click();

//   // Variables
//
//
//   double &x, &y, &z, &rx, &ry, &rz, &rw;
};

} // end namespace rviz_bulldog_commander

#endif // BULLDOG_PANEL_H

// void init();
// bool auto_execute();
// bool detect(double &x, double &y, double &z, double &rx, double &ry, double &rz, double &rw);
// bool move();
// bool pick(double &x, double &y, double &z, double &rx, double &ry, double &rz, double &rw);
// void reset()
// void stop()
