
#pragma once
#include <odrive_main.h>

class MotionController {
   public:
    MotionController(Axis* X, Axis* Y) : xAxis(X), yAxis(Y){};

    void coordinated_move(float Xf, float Yf) {
        // Not Implemented
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
