#include "planar_quadrotor_visualizer.h"

PlanarQuadrotorVisualizer::PlanarQuadrotorVisualizer(PlanarQuadrotor *quadrotor_ptr): quadrotor_ptr(quadrotor_ptr) {}

/**
 * TODO: Improve visualizetion
 * X 1. Transform coordinates from quadrotor frame to image frame so the circle is in the middle of the screen
 * 2. Use more shapes to represent quadrotor (e.x. try replicate http://underactuated.mit.edu/acrobot.html#section3 or do something prettier)
 * 3. Animate proppelers (extra points)
 */
void PlanarQuadrotorVisualizer::render(std::shared_ptr<SDL_Renderer> &gRenderer) {
    Eigen::VectorXf state = quadrotor_ptr->GetState();
    float q_x, q_y, q_theta;

    /* x, y, theta coordinates */
    q_x = state[0];
    q_y = state[1];
    q_theta = state[2];

    /*
        Transform coordinates from quadrotor frame to image frame so the circle is in the middle of the screen
    */
    float screen_x, screen_y;
    int renderer_w, renderer_h;
    SDL_GetRendererOutputSize(gRenderer.get(), &renderer_w, &renderer_h);
    screen_x = q_x + renderer_w/2;
    screen_y = q_y + renderer_h/2;

    SDL_SetRenderDrawColor(gRenderer.get(), 0xFF, 0x00, 0x00, 0xFF);
    filledCircleColor(gRenderer.get(), screen_x, screen_y, 30, 0xFF0000FF);
}