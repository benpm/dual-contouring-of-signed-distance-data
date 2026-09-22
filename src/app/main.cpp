#include "application.h"

#include <polyscope/options.h>
#include <polyscope/polyscope.h>

#include <string>

int main(int argc, char** argv) {
    polyscope::options::programName = "Dual Contouring of Signed Distance Data";
    polyscope::options::rightGuiPaneWidth = 460;
    polyscope::options::maxFPS = 60;
    polyscope::options::alwaysRedraw = true;
    polyscope::init();

    Application application;
    application.initialize(argc > 1 ? std::string(argv[1]) : std::string{});
    polyscope::state::userCallback = [&application]() { application.draw_ui(); };
    polyscope::state::filesDroppedCallback = [&application](const std::vector<std::string>& paths) {
        application.handle_dropped_files(paths);
    };
    polyscope::show();
    return 0;
}
