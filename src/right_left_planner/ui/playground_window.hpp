//
// Created by plaba on 4/23/23.
//

#ifndef RIGHT_LEFT_PLANNER_PLAYGROUND_WINDOW_HPP
#define RIGHT_LEFT_PLANNER_PLAYGROUND_WINDOW_HPP

#include <QWidget>

namespace right_left_planner {
QT_BEGIN_NAMESPACE
namespace Ui { class PlaygroundWindow; }
QT_END_NAMESPACE

class PlaygroundWindow : public QWidget {
  Q_OBJECT

public:
  explicit PlaygroundWindow(QWidget *parent = nullptr);

  ~PlaygroundWindow() override;

private:
  Ui::PlaygroundWindow *ui;

};

} // right_left_planner

#endif //RIGHT_LEFT_PLANNER_PLAYGROUND_WINDOW_HPP
