#ifndef DRONALTITUDECONTROLLER_HPP
#define DRONALTITUDECONTROLLER_HPP
#include <cstdint>

struct DronConfig
{
    double g = 9.81;
    double mass = 1.5;

    double wn = 4.0;
    double zeta = 1.2;
    double ki = 2.0;

    double u_min = 0.0;
    double u_max = 35.0;

    double i_min = -3.0;
    double i_max = 3.0;

    double z_takeoff = 10.0;

    double pos_tol_hover = 0.15;
    double vel_tol_hover = 0.2;
    double hover_time = 5.0;

    double pos_tol_land = 0.05;
    double vel_tol_land = 0.1;
};

enum class FlightState : std::uint8_t {
    Idle,
    TakeOff,
    Hover,
    Land,
    Completed
};

class DronAltitudeController{
    public:
    DronAltitudeController(const DronConfig& config);
    double update(bool start_cmd, double z, double z_dot, double t, double dt);
    
    double getKd() const;
    double getKp() const;
    FlightState getState() const;

    private:
    DronConfig config_;
    double kd_;
    double kp_;

    FlightState state_ = FlightState::Idle;

    double integral_force_ = 0.0;

    double t_hover_start_ = 0.0;
};

#endif