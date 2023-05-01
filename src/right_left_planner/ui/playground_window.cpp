//
// Created by plaba on 4/23/23.
//

// You may need to build the project (run Qt uic code generator) to get "ui_playgroundwindow.h" resolved

#include "playground_window.hpp"
#include "ui_playgroundwindow.h"
#include <QOpenGLFunctions>
#include <QPainter>

namespace right_left_planner {
PlaygroundWindow::PlaygroundWindow(QWidget *parent) :
        QWidget(parent), ui(new Ui::PlaygroundWindow) {
    ui->setupUi(this);
}

PlaygroundWindow::~PlaygroundWindow() {
    delete ui;
}
} // right_left_planner
