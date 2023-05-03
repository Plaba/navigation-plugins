//
// Created by plaba on 4/23/23.
//

#ifndef RIGHT_LEFT_PLANNER_PLAYGROUND_WIDGET_HPP
#define RIGHT_LEFT_PLANNER_PLAYGROUND_WIDGET_HPP

#include <QWidget>
#include <QOpenGLWidget>
#include <QPainter>
#include "right_left_planner/right_left_algorithm.hpp"
#include "right_left_planner/path_segment.hpp"

namespace right_left_planner {

class PlaygroundWidget : public QOpenGLWidget {

Q_OBJECT

public:

  explicit PlaygroundWidget(QWidget *parent = nullptr);

public slots:
  void setnumcellswide(int cellwidth);
  void setnumcellshigh(int cellheight);

  void setobstacle();
  void setstart();
  void setgoal();
  void setfree();

  void plan();
  void step();
  void resetplanning();

protected:

  ~PlaygroundWidget() override;

  void mousePressEvent(QMouseEvent *event) override;

  void mouseMoveEvent(QMouseEvent *event) override;

  void initializeGL() override;

  void resizeGL(int w, int h) override;

  void paintGL() override;

private:

  std::unique_ptr<LineString> result = nullptr;
  std::shared_ptr<PathStep> step_result = nullptr;

  void drawPathStep(QPainter &painter, const PathStep &step);

  bool setup = false;
  bool highlight_unvisited = false;

  enum class SetMode {
    SET_OBSTACLE,
    SET_START,
    SET_GOAL,
    SET_FREE
  };

  SetMode setmode = SetMode::SET_OBSTACLE;

  int start[2] = {0, 0};
  int goal[2] = {0, 0};

  int numcellswide = 50;
  int numcellshigh = 50;

  RightLeftAlgorithm planner;

    void drawSegment(QPainter &painter, const Segment &segment, int xoffset, int yoffset, int squaresize) const;
};


} // right_left_planner

#endif //RIGHT_LEFT_PLANNER_PLAYGROUND_WIDGET_HPP
