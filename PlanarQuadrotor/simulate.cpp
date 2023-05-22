/**
 * SDL window creation adapted from https://github.com/isJuhn/DoublePendulum
*/
#include "simulate.h"

#include <matplot/matplot.h>
#include <thread>

Eigen::MatrixXf LQR(PlanarQuadrotor &quadrotor, float dt) {
    /* Calculate LQR gain matrix */
    Eigen::MatrixXf Eye = Eigen::MatrixXf::Identity(6, 6);
    Eigen::MatrixXf A = Eigen::MatrixXf::Zero(6, 6);
    Eigen::MatrixXf A_discrete = Eigen::MatrixXf::Zero(6, 6);
    Eigen::MatrixXf B(6, 2);
    Eigen::MatrixXf B_discrete(6, 2);
    Eigen::MatrixXf Q = Eigen::MatrixXf::Identity(6, 6);
    Eigen::MatrixXf R = Eigen::MatrixXf::Identity(2, 2);
    Eigen::MatrixXf K = Eigen::MatrixXf::Zero(6, 6);
    Eigen::Vector2f input = quadrotor.GravityCompInput();

    Q.diagonal() << 4e-3, 4e-3, 4e2, 8e-3, 4.5e-2, 2 / 2 / M_PI;
    R.row(0) << 3e1, 7;
    R.row(1) << 7, 3e1;

    std::tie(A, B) = quadrotor.Linearize();
    A_discrete = Eye + dt * A;
    B_discrete = dt * B;
    
    return LQR(A_discrete, B_discrete, Q, R);
}

void control(PlanarQuadrotor &quadrotor, const Eigen::MatrixXf &K) {
    Eigen::Vector2f input = quadrotor.GravityCompInput();
    quadrotor.SetInput(input - K * quadrotor.GetControlState());
}

void show_history(std::vector<float> x_history, std::vector<float> y_history, std::vector<float> theta_history, std::atomic<bool>& history_active)
{
    matplot::plot(x_history, y_history);
    matplot::show();
    history_active = false;
}

int main(int argc, char* args[])
{
    std::shared_ptr<SDL_Window> gWindow = nullptr;
    std::shared_ptr<SDL_Renderer> gRenderer = nullptr;
    const int SCREEN_WIDTH = 1280;
    const int SCREEN_HEIGHT = 720;

    /**
     * TODO: Extend simulation
     * X 1. Set goal state of the mouse when clicking left mouse button (transform the coordinates to the quadrotor world! see visualizer TODO list)
     *    [x, y, 0, 0, 0, 0]
     * X 2. Update PlanarQuadrotor from simulation when goal is changed
    */

    Eigen::VectorXf initial_state = Eigen::VectorXf::Zero(6);
    srand(time(NULL));
    int initial_x = rand() % (SCREEN_WIDTH/2 - (-SCREEN_WIDTH/2) +1) - SCREEN_WIDTH/2;
    int initial_y = rand() % (SCREEN_HEIGHT/2 - (-SCREEN_HEIGHT/2) + 1) - SCREEN_HEIGHT/2;
    std::cout << "Initial state: " << initial_x << " " << initial_y << std::endl;
    
    initial_state << initial_x, initial_y, 0, 0, 0, 0;

    PlanarQuadrotor quadrotor(initial_state);
    PlanarQuadrotorVisualizer quadrotor_visualizer(&quadrotor);
    /**
     * Goal pose for the quadrotor
     * [x, y, theta, x_dot, y_dot, theta_dot]
     * For implemented LQR controller, it has to be [x, y, 0, 0, 0, 0]
    */
    Eigen::VectorXf goal_state = Eigen::VectorXf::Zero(6);
    quadrotor.SetGoal(goal_state);
    /* Timestep for the simulation */
    const float dt = 0.001;
    Eigen::MatrixXf K = LQR(quadrotor, dt);
    Eigen::Vector2f input = Eigen::Vector2f::Zero(2);

    /**
     * TODO: Plot x, y, theta over time
     * X 1. Update x, y, theta history vectors to store trajectory of the quadrotor
     * X 2. Plot trajectory using matplot++ when key 'p' is clicked
    */
    std::vector<float> x_history;
    std::vector<float> y_history;
    std::vector<float> theta_history;

    if (init(gWindow, gRenderer, SCREEN_WIDTH, SCREEN_HEIGHT) >= 0)
    {
        SDL_Event e;
        bool quit = false;
        Eigen::VectorXf state = Eigen::VectorXf::Zero(6);

        while (!quit)
        {
            //events
            while (SDL_PollEvent(&e) != 0)
            {
                static std::atomic<bool> history_active = false;

                if (e.type == SDL_QUIT)
                {
                    quit = true;
                }
                else if (e.type == SDL_MOUSEBUTTONDOWN)
                {
                    int mouse_x, mouse_y;
                    SDL_GetMouseState(&mouse_x, &mouse_y);
                    mouse_x = (mouse_x - SCREEN_WIDTH / 2);
                    mouse_y = (mouse_y - SCREEN_HEIGHT / 2);
                    std::cout << "Setting new position: (" << mouse_x << ", " << mouse_y << ")" << std::endl;
                    goal_state << mouse_x, mouse_y, 0, 0, 0, 0;
                    quadrotor.SetGoal(goal_state);
                }
                else if(e.type == SDL_KEYDOWN && e.key.keysym.sym == SDL_KeyCode::SDLK_p && history_active == false)
                {
                    std::cout << "Plotting" << std::endl;
                    history_active = true;

                    std::thread history_thread(show_history, x_history, y_history, theta_history, std::ref(history_active));
                    history_thread.detach();
                }
            }

            SDL_Delay((int) dt * 1000);

            SDL_SetRenderDrawColor(gRenderer.get(), 0x3E, 0x78, 0xB2, 0xFF);
            SDL_RenderClear(gRenderer.get());

            /* Quadrotor rendering step */
            quadrotor_visualizer.render(gRenderer);

            SDL_RenderPresent(gRenderer.get());

            /* Simulate quadrotor forward in time */
            control(quadrotor, K);
            quadrotor.Update(dt);

            Eigen::VectorXf current_state = quadrotor.GetState();
            x_history.push_back(current_state[0]);
            y_history.push_back(current_state[1]);
            theta_history.push_back(current_state[2]);
        }
    }
    SDL_Quit();
    return 0;
}

int init(std::shared_ptr<SDL_Window>& gWindow, std::shared_ptr<SDL_Renderer>& gRenderer, const int SCREEN_WIDTH, const int SCREEN_HEIGHT)
{
    SDL_RendererFlags sdl_renderer = SDL_RENDERER_ACCELERATED;
    #ifndef _MSC_VER
    sdl_renderer = SDL_RENDERER_SOFTWARE;
    #endif

    if (SDL_Init(SDL_INIT_VIDEO) >= 0)
    {
        SDL_SetHint(SDL_HINT_RENDER_SCALE_QUALITY, "1");
        gWindow = std::shared_ptr<SDL_Window>(SDL_CreateWindow("Planar Quadrotor", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, SCREEN_WIDTH, SCREEN_HEIGHT, SDL_WINDOW_SHOWN), SDL_DestroyWindow);
        gRenderer = std::shared_ptr<SDL_Renderer>(SDL_CreateRenderer(gWindow.get(), -1, sdl_renderer), SDL_DestroyRenderer);
    }
    else
    {
        std::cout << "SDL_ERROR: " << SDL_GetError() << std::endl;
        return -1;
    }
    return 0;
}