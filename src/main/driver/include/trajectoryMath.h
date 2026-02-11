#pragma once
#include "glm/glm.hpp"

class TrajectorySolution {
public:
    double shooterVelocity;
    double azimuth;

    TrajectorySolution() {
        shooterVelocity = 0;
        azimuth = 0;
    }

    TrajectorySolution(glm::vec2 in) {
        shooterVelocity = in.x;
        azimuth = in.y;
    }
};
TrajectorySolution newtonRhapsonSolveAirResistance(
    double botVelocityX,
    double botVelocityY,
    glm::vec3 targetPos,
    double shooterAltitude,
    TrajectorySolution guessSolution);

TrajectorySolution calcFiringSolution(
    double botVelocityX,
    double botVelocityY,
    glm::vec3 targetPos,
    double shooterAltitude);
glm::vec2 f_airResistance_RK4(
    glm::vec2 in, double v_x, double v_y, double z, double alpha);