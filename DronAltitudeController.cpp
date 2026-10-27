#include "DronAltitudeController.hpp"
#include <cmath>
#include <algorithm>

DronAltitudeController::DronAltitudeController(const DronConfig& config) :
config_(config), kp_(config.mass * (config.wn * config.wn)), 
kd_(2.0 * config.zeta * config.wn * config.mass){}

double DronAltitudeController::update(bool start_cmd, double z, double z_dot, double t, double dt){
    bool pid_enable = false;
    double z_ref = 0.0;

    switch(state_){
        case FlightState::Idle:
            pid_enable = false;

            if(start_cmd)
            {
                state_ = FlightState::TakeOff;
            }
        break;

        case FlightState::TakeOff:
            pid_enable = true;
            z_ref = config_.z_takeoff;
            
            if(std::abs(z-config_.z_takeoff) < config_.pos_tol_hover && std::abs(z_dot) < config_.vel_tol_hover){
                state_ = FlightState::Hover;
                t_hover_start_ = t;
            }
        break;

        case FlightState::Hover:
            pid_enable = true;
            z_ref = config_.z_takeoff;

            if(t >= t_hover_start_ + config_.hover_time) state_ = FlightState::Land;
        break;    

        case FlightState::Land:
            pid_enable = true;
            z_ref = 0.0;

            if(std::abs(z) < config_.pos_tol_land && std::abs(z_dot) < config_.vel_tol_land) state_ = FlightState::Completed;
        break;

        case FlightState::Completed:
        pid_enable = false;

        break;

        default:
        pid_enable = false;
        
        break;
    }

    if(!pid_enable){
        integral_force_ = 0.0;
        z_ref = 0.0;
        return 0.0;
    }

    double const error = z_ref - z;
    double const p_term = kp_ * error;
    double const d_term = kd_ * z_dot;
    integral_force_ += (config_.ki * error) * dt;
    integral_force_ = std::clamp(integral_force_, config_.i_min, config_.i_max);
    double total_force = (config_.mass*config_.g) + integral_force_ + p_term - d_term;
    total_force = std::clamp(total_force, config_.u_min, config_.u_max);

    return total_force;
}

double DronAltitudeController::getKd() const{
    return kd_;
}

double DronAltitudeController::getKp() const{
    return kp_;
}

FlightState DronAltitudeController::getState() const{
    return state_;
}