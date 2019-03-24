
#pragma once
#include <odrive_main.h>

class MotionController {
   public:
    MotionController(Axis* X, Axis* Y) : xAxis(X), yAxis(Y){};

    void coordinated_move(float Xf, float Yf) {
        xAxis->trap_.planTrapezoidal(Xf,
                                     xAxis->controller_.pos_setpoint_,
                                     xAxis->controller_.vel_setpoint_,
                                     xAxis->trap_.config_.vel_limit,
                                     xAxis->trap_.config_.accel_limit,
                                     xAxis->trap_.config_.decel_limit);

        yAxis->trap_.planTrapezoidal(Yf,
                                     yAxis->controller_.pos_setpoint_,
                                     yAxis->controller_.vel_setpoint_,
                                     yAxis->trap_.config_.vel_limit,
                                     yAxis->trap_.config_.accel_limit,
                                     yAxis->trap_.config_.decel_limit);

        if (xAxis->trap_.Tf_ > yAxis->trap_.Tf_ && yAxis->trap_.Td_ != 0.0f) {
            auto tDiff = std::abs(xAxis->trap_.Tf_ - yAxis->trap_.Tf_);
            auto Dmax = yAxis->trap_.config_.decel_limit*((yAxis->trap_.Td_ + tDiff)/yAxis->trap_.Td_);

            yAxis->trap_.planTrapezoidal(Yf,
                                         yAxis->controller_.pos_setpoint_,
                                         yAxis->controller_.vel_setpoint_,
                                         yAxis->trap_.config_.vel_limit,
                                         yAxis->trap_.config_.accel_limit,
                                         Dmax);
        } else if(yAxis->trap_.Tf_ > xAxis->trap_.Tf_ && xAxis->trap_.Tf_ != 0){
            auto tDiff = std::abs(xAxis->trap_.Tf_ - yAxis->trap_.Tf_);
            auto Dmax = xAxis->trap_.config_.decel_limit*((xAxis->trap_.Td_ + tDiff)/xAxis->trap_.Td_);

            xAxis->trap_.planTrapezoidal(Xf,
                                         xAxis->controller_.pos_setpoint_,
                                         xAxis->controller_.vel_setpoint_,
                                         xAxis->trap_.config_.vel_limit,
                                         xAxis->trap_.config_.accel_limit,
                                         Dmax);
        }
        xAxis->controller_.traj_start_loop_count_ = xAxis->loop_counter_;
        yAxis->controller_.traj_start_loop_count_ = yAxis->loop_counter_;
        xAxis->controller_.config_.control_mode = Controller::CTRL_MODE_TRAJECTORY_CONTROL;
        yAxis->controller_.config_.control_mode = Controller::CTRL_MODE_TRAJECTORY_CONTROL;
        xAxis->controller_.goal_point_ = Xf;
        yAxis->controller_.goal_point_ = Yf;
    }

    void set_x_axis(uint32_t axis_num) {
        if(axis_num < AXIS_COUNT)
            xAxis = axes[axis_num];
    }
    void set_y_axis(uint32_t axis_num) {
        if(axis_num < AXIS_COUNT)
            yAxis = axes[axis_num];
    };

    auto getXAxis() {
        return xAxis;
    }

    auto getYAxis() {
        return yAxis;
    }

    auto make_protocol_definitions(){
        return make_protocol_member_list(
            make_protocol_function("coordinated_move", *this, &MotionController::coordinated_move, "X_final", "Y_final"),
            make_protocol_function("set_x_axis", *this, &MotionController::set_x_axis, "Axis Num"),
            make_protocol_function("set_y_axis", *this, &MotionController::set_y_axis, "Axis Num")
        );
    }

   private:
    Axis* xAxis;
    Axis* yAxis;
};
