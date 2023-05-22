#include "planar_quadrotor_visualizer.h"

PlanarQuadrotorVisualizer::PlanarQuadrotorVisualizer(PlanarQuadrotor *quadrotor_ptr): quadrotor_ptr(quadrotor_ptr) {}

/**
 * TODO: Improve visualizetion
 * X 1. Transform coordinates from quadrotor frame to image frame so the circle is in the middle of the screen
 * X 2. Use more shapes to represent quadrotor (e.x. try replicate http://underactuated.mit.edu/acrobot.html#section3 or do something prettier)
 * X 3. Animate proppelers (extra points)
 */
void PlanarQuadrotorVisualizer::render(std::shared_ptr<SDL_Renderer> &gRenderer) {

    Vector2 propeller_leg_sz = {5, 15};
    Vector2 quadrotor_sz = {90, 10};
    Vector2 big_propeller_sz = {15,7};
    Vector2 small_propeller_sz = {10,5};
    float propeller_leg_quadrocoptor_dist = quadrotor_sz.x*0.9f;

    /* Color structure: AARRGGBB*/
    int quadrotor_color = 0xFF34403A;
    int propeller_leg_color = 0xFF285238;
    int propeller_color = 0xFF138A36;

    /* x, y, theta coordinates */
    Eigen::VectorXf state = quadrotor_ptr->GetState();
    
    int renderer_w, renderer_h;
    SDL_GetRendererOutputSize(gRenderer.get(), &renderer_w, &renderer_h);
    Vector2 quadrotor_pos = {state[0] + renderer_w/2, state[1] + renderer_h/2};
    float q_theta = state[2];

    static bool propeller_anim_state = false;
    static int propeller_anim_tick_last = 0;
    static int propeller_multiplier = 1;

    if (SDL_GetTicks() - propeller_anim_tick_last > 100) // One tick should be 1ms
    {
        propeller_anim_state = !propeller_anim_state;
        propeller_anim_tick_last = SDL_GetTicks();
        propeller_multiplier = propeller_anim_state ? 1 : -1;
    }

    /* Distance from quadrotor middle point to propellers */
    Vector2 quadrotor_propeller_dist  =  { propeller_leg_quadrocoptor_dist * std::cos(q_theta), propeller_leg_quadrocoptor_dist * std::sin(q_theta)};

    /* Propeller position from their legs */
    Vector2 propeller = { propeller_leg_sz.y * std::sin(q_theta), propeller_leg_sz.y * std::cos(q_theta)};

    /* Quadrotor extremities */
    Vector2 quad_left_top     = {quadrotor_pos.x - std::cos(q_theta) * (quadrotor_sz.x / 2), quadrotor_pos.y + std::sin(q_theta) * (quadrotor_sz.x / 2)};
    Vector2 quad_right_bottom = {quadrotor_pos.x + (quadrotor_sz.x / 2) * std::cos(q_theta), quadrotor_pos.y - std::sin(q_theta) * (quadrotor_sz.x / 2)};
    
    /* Propellers leg beginning points (quadrotor) */
    Vector2 quadrotor_propeller_leg_left  = {quad_right_bottom.x - quadrotor_propeller_dist.x, quad_right_bottom.y + quadrotor_propeller_dist.y};
    Vector2 quadrotor_propeller_leg_right = {quad_left_top.x + quadrotor_propeller_dist.x, quad_left_top.y - quadrotor_propeller_dist.y};

    /* Propellers leg ending points (propeller) */
    Vector2 propeller_propeller_leg_left  = {quadrotor_propeller_leg_left.x - propeller.x, quadrotor_propeller_leg_left.y - propeller.y};
    Vector2 propeller_propeller_leg_right = {quadrotor_propeller_leg_right.x - propeller.x, quadrotor_propeller_leg_right.y - propeller.y};

    /* Propellers */
    Vector2 propeller_left_big = {propeller_propeller_leg_left.x + (big_propeller_sz.x * propeller_multiplier), propeller_propeller_leg_left.y};
    Vector2 propeller_left_small = {propeller_propeller_leg_left.x - (small_propeller_sz.x * propeller_multiplier), propeller_propeller_leg_left.y};

    Vector2 propeller_right_big = {propeller_propeller_leg_right.x - (big_propeller_sz.x * propeller_multiplier), propeller_propeller_leg_right.y};
    Vector2 propeller_right_small = {propeller_propeller_leg_right.x + (small_propeller_sz.x * propeller_multiplier), propeller_propeller_leg_right.y};

    /* Render */
    thickLineColor(gRenderer.get(), quadrotor_propeller_leg_right.x, quadrotor_propeller_leg_right.y, propeller_propeller_leg_right.x, propeller_propeller_leg_right.y, propeller_leg_sz.x, propeller_leg_color);
    thickLineColor(gRenderer.get(), quadrotor_propeller_leg_left.x, quadrotor_propeller_leg_left.y, propeller_propeller_leg_left.x, propeller_propeller_leg_left.y, propeller_leg_sz.x, propeller_leg_color);
    
    filledEllipseColor(gRenderer.get(), propeller_left_big.x, propeller_left_big.y, big_propeller_sz.x, big_propeller_sz.y, propeller_color);
    filledEllipseColor(gRenderer.get(), propeller_left_small.x, propeller_left_small.y, small_propeller_sz.x, small_propeller_sz.y, propeller_color);
    filledEllipseColor(gRenderer.get(), propeller_right_big.x, propeller_right_big.y, big_propeller_sz.x, big_propeller_sz.y, propeller_color);
    filledEllipseColor(gRenderer.get(), propeller_right_small.x, propeller_right_small.y, small_propeller_sz.x, small_propeller_sz.y, propeller_color);

    thickLineColor(gRenderer.get(), quad_left_top.x, quad_left_top.y, quad_right_bottom.x, quad_right_bottom.y, quadrotor_sz.y, quadrotor_color);
}