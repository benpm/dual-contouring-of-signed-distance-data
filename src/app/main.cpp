#include "application.h"
#include "gl_api.h"
#include "renderer.h"

#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>
#include <imgui.h>
#include <backends/imgui_impl_glfw.h>
#include <backends/imgui_impl_opengl3.h>

#include <cstdio>
#include <exception>

namespace {

void glfw_error_callback(int, const char* description) {
    std::fprintf(stderr, "GLFW error: %s\n", description);
}

} // namespace

int main() {
    glfwSetErrorCallback(glfw_error_callback);
    if (!glfwInit()) return 1;
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 2);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
#ifdef __APPLE__
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GLFW_TRUE);
#endif
    GLFWwindow* window = glfwCreateWindow(1280, 800, "Dual Contouring Gallery", nullptr, nullptr);
    if (!window) {
        glfwTerminate();
        return 1;
    }
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    try {
        gl::load();
        IMGUI_CHECKVERSION();
        ImGui::CreateContext();
        ImGui::StyleColorsDark();
        ImGui_ImplGlfw_InitForOpenGL(window, true);
        ImGui_ImplOpenGL3_Init("#version 150");

        Renderer renderer;
        Application application(renderer);
        double previous_x = 0.0;
        double previous_y = 0.0;
        bool was_orbiting = false;
        while (!glfwWindowShouldClose(window)) {
            glfwPollEvents();
            ImGui_ImplOpenGL3_NewFrame();
            ImGui_ImplGlfw_NewFrame();
            ImGui::NewFrame();

            application.update();
            double mouse_x = 0.0;
            double mouse_y = 0.0;
            glfwGetCursorPos(window, &mouse_x, &mouse_y);
            const bool orbiting = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS && !ImGui::GetIO().WantCaptureMouse;
            if (orbiting && was_orbiting) application.orbit(static_cast<float>(mouse_x - previous_x), static_cast<float>(mouse_y - previous_y));
            previous_x = mouse_x;
            previous_y = mouse_y;
            was_orbiting = orbiting;

            const double scroll = ImGui::GetIO().MouseWheel;
            if (!ImGui::GetIO().WantCaptureMouse && scroll != 0.0) application.zoom(static_cast<float>(scroll));

            int framebuffer_width = 0;
            int framebuffer_height = 0;
            int window_width = 0;
            int window_height = 0;
            glfwGetFramebufferSize(window, &framebuffer_width, &framebuffer_height);
            glfwGetWindowSize(window, &window_width, &window_height);
            const Eigen::Matrix4f view_projection = application.view_projection(
                framebuffer_height > 0 ? static_cast<float>(framebuffer_width) / framebuffer_height : 1.0f);
            renderer.draw(application.render_items(), view_projection, framebuffer_width, framebuffer_height);

            if (ImGui::IsMouseClicked(ImGuiMouseButton_Left) && !ImGui::GetIO().WantCaptureMouse && window_width > 0 && window_height > 0) {
                const int pixel_x = static_cast<int>(mouse_x * framebuffer_width / window_width);
                const int pixel_y = static_cast<int>(mouse_y * framebuffer_height / window_height);
                const unsigned int picked = renderer.pick(
                    application.render_items(), view_projection, framebuffer_width, framebuffer_height, pixel_x, pixel_y);
                if (picked != 0) application.select(picked);
            }

            application.draw_ui();
            ImGui::Render();
            ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
            glfwSwapBuffers(window);
        }

        ImGui_ImplOpenGL3_Shutdown();
        ImGui_ImplGlfw_Shutdown();
        ImGui::DestroyContext();
    } catch (const std::exception& exception) {
        std::fprintf(stderr, "%s\n", exception.what());
        glfwDestroyWindow(window);
        glfwTerminate();
        return 1;
    }

    glfwDestroyWindow(window);
    glfwTerminate();
    return 0;
}
