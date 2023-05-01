#include <QtWidgets>
#include "playground_window.hpp"

int main(int argc, char *argv[])
{
    QApplication app(argc, argv);

    right_left_planner::PlaygroundWindow window;

    window.resize(1600, 1200);
    window.show();
    return app.exec();
}