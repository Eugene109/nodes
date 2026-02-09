#include "driverheader.h"

#include <units/angular_velocity.h>
#include <units/time.h>
#include <units/angle.h>
#include <units/length.h>
#include <units/velocity.h>
#include <units/voltage.h>
#include <frc/ct_matrix.h>
#include <units/base.h>
#include <Eigen/Dense>
#include <iostream>
extern "C" {
void c_doThing() {
}
struct trajectorySolution{
    double v_s;
    double theta;
};
trajectorySolution a(){
    return trajectorySolution{1.0, 1.0};
}
}
