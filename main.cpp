#include "DronAltitudeController.hpp"
#include <cmath>
#include <iostream>

int main (){
    DronConfig config;
    DronAltitudeController dron(config);

    double constexpr t_simulation = 20.0;
    double constexpr dt = 0.01;
    double t = 0.0;
    double const one_m = 1.0 / config.mass; // mass
    bool start_cmd = false;
    double constexpr step = 2.0;
    double z = 0.0;
    double z_dot = 0.0;

    // simulation
    int const total_steps = static_cast<int>(t_simulation / dt); 

    for(int step_k = 0; step_k <= total_steps; ++step_k){
        t = step_k * dt;
        FlightState previous_state = dron.getState();
        if(t >= step) start_cmd = true;
        double force_u = dron.update(start_cmd, z, z_dot, t, dt);
        double z_ddot = one_m * (force_u - config.mass * config.g); // 1/m * (u - m * g) = acceleration
        z_dot += z_ddot * dt;
        z += z_dot * dt;

        if(z <= 0.0){
            z = 0.0;
            z_dot = 0.0;
        }


        FlightState current_state = dron.getState();
        if(step_k % 50 == 0 || current_state != previous_state){
            std::cout << t << "," << static_cast<int>(current_state) << "," << z << "," << z_dot << "\n";
        }

    }
}