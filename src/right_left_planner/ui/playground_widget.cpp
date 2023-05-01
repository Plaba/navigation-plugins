#include "playground_widget.hpp"
#include <QOpenGLWidget>
#include <QOpenGLFunctions>
#include <QWidget>
#include <QPainter>
#include <QMouseEvent>
#include "right_left_planner/right_left_algorithm.hpp"

namespace right_left_planner {

PlaygroundWidget::PlaygroundWidget(QWidget *parent) 
: QOpenGLWidget(parent) 
, planner(numcellswide, numcellshigh)
{ }

PlaygroundWidget::~PlaygroundWidget() = default;

void PlaygroundWidget::initializeGL() {
    QOpenGLFunctions *f = QOpenGLContext::currentContext()->functions();
    f->glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
}

void PlaygroundWidget::resizeGL(int w, int h) {
    QOpenGLFunctions *f = QOpenGLContext::currentContext()->functions();
    f->glViewport(0, 0, w, h);
}

void PlaygroundWidget::paintGL() {
    QOpenGLFunctions *f = QOpenGLContext::currentContext()->functions();
    f->glClear(GL_COLOR_BUFFER_BIT);
    QPainter painter(this);

    int squaresize = std::min(width() / numcellswide, height() / numcellshigh);
    int xoffset = (width() - squaresize * numcellswide) / 2;
    int yoffset = (height() - squaresize * numcellshigh) / 2;

    painter.setPen(QPen(Qt::darkGray, 1));

    for (int i = 0; i < numcellswide; i++) {
        for (int j = 0; j < numcellshigh; j++) {
            if (isObstacle(planner.getCell(i, j))) {
                if (highlight_unvisited && !isVisited(planner.getCell(i, j))) {
                    painter.setBrush(QBrush(Qt::darkRed));
                } else {
                    painter.setBrush(QBrush(Qt::gray));
                }
            } else if (i == start[0] && j == start[1]) {
                painter.setBrush(QBrush(Qt::blue));
            } else if (i == goal[0] && j == goal[1]) {
                painter.setBrush(QBrush(Qt::green));
            } else {
                if (highlight_unvisited && !isVisited(planner.getCell(i, j))) {
                    painter.setBrush(QBrush(Qt::yellow));
                } else {
                    painter.setBrush(QBrush(Qt::white));
                }
            }
            painter.drawRect(xoffset + i * squaresize, yoffset + j * squaresize, squaresize, squaresize);
        }
    }

    painter.setPen(QPen(Qt::black, 2));

    for (auto line: planner.getSegments())
    {
        painter.drawLine(xoffset + (line.first.first.x() + 1)/2 * squaresize,
                         yoffset + (line.first.first.y() + 1)/2  * squaresize,
                         xoffset + (line.first.second.x() + 1)/2 * squaresize,
                         yoffset + (line.first.second.y() + 1)/2 * squaresize);
    }

    if (result != nullptr) {
        painter.setPen(QPen(Qt::red, 2));
        for (size_t i = 0; i < result->size() - 1; i++) {
            painter.drawLine(xoffset + result->at(i).x() * squaresize + squaresize / 2,
                             yoffset + result->at(i).y() * squaresize + squaresize / 2,
                             xoffset + result->at(i + 1).x() * squaresize + squaresize / 2,
                             yoffset + result->at(i + 1).y() * squaresize + squaresize / 2);
        }
    } else if (step_result != nullptr) {
        drawPathStep(painter, *step_result);
        step_result = nullptr;
    }
}

void PlaygroundWidget::drawPathStep(QPainter &painter, const PathStep &step){
    int squaresize = std::min(width() / numcellswide, height() / numcellshigh);
    int xoffset = (width() - squaresize * numcellswide) / 2;
    int yoffset = (height() - squaresize * numcellshigh) / 2;

    if (step.choice != nullptr) {
        drawPathStep(painter, step.choice->clockwise_choice);
        drawPathStep(painter, step.choice->counter_clockwise_choice);
    } else {
        if (step.next->completed) {
            painter.setPen(QPen(Qt::green, 2));
        } else {
            painter.setPen(QPen(Qt::blue, 2, Qt::DashLine));
        }

        auto segment = step.next->segment;
        painter.drawLine(xoffset + segment.first.x()/2 * squaresize + squaresize / 2,
                         yoffset + segment.first.y()/2 * squaresize + squaresize / 2,
                         xoffset + segment.second.x()/2 * squaresize + squaresize / 2,
                         yoffset + segment.second.y()/2 * squaresize + squaresize / 2);

        if(step.next->next != nullptr) {
            drawPathStep(painter, *step.next->next);
        }
    }
}

void PlaygroundWidget::setnumcellswide(int cellwidth) {
    numcellswide = cellwidth;
    planner.setNavArr(numcellswide, numcellshigh);
    update();
}

void PlaygroundWidget::setnumcellshigh(int cellheight) {
    numcellshigh = cellheight;
    planner.setNavArr(numcellswide, numcellshigh);
    update();
}

void PlaygroundWidget::setobstacle() {
    setmode = SetMode::SET_OBSTACLE;
}

void PlaygroundWidget::setstart() {
    setmode = SetMode::SET_START;
}

void PlaygroundWidget::setgoal() {
    setmode = SetMode::SET_GOAL;
}

void PlaygroundWidget::setfree() {
    setmode = SetMode::SET_FREE;
}

void PlaygroundWidget::plan() {
    planner.setStart(start);
    planner.setGoal(goal);

    highlight_unvisited = true;

    result = std::make_unique<LineString>(planner.calculatePlan());
    std::cout << "Plan calculated with length: " << result->size() << std::endl; 
    update();
}

void PlaygroundWidget::step() {
    planner.setStart(start);
    planner.setGoal(goal);


    if (!setup) {
        planner.prepare();
        setup = true;
        highlight_unvisited = true;
    }

    step_result = planner.calculateStep();
    update();
}

void PlaygroundWidget::mousePressEvent(QMouseEvent * event) {
    mouseMoveEvent(event);
}

void PlaygroundWidget::mouseMoveEvent(QMouseEvent * event) {
    int squaresize = std::min(width() / numcellswide, height() / numcellshigh);
    int xoffset = (width() - squaresize * numcellswide) / 2;
    int yoffset = (height() - squaresize * numcellshigh) / 2;

    int x = (event->x() - xoffset) / squaresize;
    int y = (event->y() - yoffset) / squaresize;

    if (x >= 0 && x < numcellswide && y >= 0 && y < numcellshigh) {
        switch (setmode) {
            case SetMode::SET_OBSTACLE:
                planner.setObstacle(x, y);
                break;
            case SetMode::SET_START:
                start[0] = x;
                start[1] = y;
                std::cout << "Start set to: " << start[0] << ", " << start[1] << std::endl;
                break;
            case SetMode::SET_GOAL:
                goal[0] = x;
                goal[1] = y;
                std::cout << "Goal set to: " << goal[0] << ", " << goal[1] << std::endl;
                break;
            case SetMode::SET_FREE:
                planner.setFree(x, y);
                break;
        }
        update();
    }
}

void PlaygroundWidget::reset() {
    planner.reset();
    result = nullptr;
    step_result = nullptr;
    setup = false;
    highlight_unvisited = false;
    update();
}

} // right_left_planner