#include "trajectoryMath.h"

#include <Eigen/Dense>
#include <iostream>
#include <limits>

#define g 9.81 // Gravity in m/s^2

glm::mat2 calcJacobian(glm::vec2 in, double v_x, double v_y, double z, double alpha) {
    double v_s = in[0];
    double theta = in[1];
    glm::mat2 J = glm::mat2();
    J[0][0] = (v_x * sin(alpha) / g) + ((v_x * pow(sin(alpha), 2) * v_s) / (g * sqrt(pow(sin(alpha), 2) * pow(v_s, 2) - 2 * g * z))) + 2 * cos(alpha) * cos(theta) * sin(alpha) * v_s / g + (cos(alpha) * cos(theta) / g) * (pow(sin(alpha), 2) * pow(v_s, 3) - 2 * g * z * v_s) / (sqrt(pow(sin(alpha), 2) * pow(v_s, 4) - 2 * g * z * pow(v_s, 2)));
    J[1][0] = ((v_s * sin(alpha) + sqrt(pow(v_s, 2) * pow(sin(alpha), 2) - 2 * g * z)) / g) * cos(alpha) * v_s * (-sin(theta));
    J[0][1] = (v_y * sin(alpha) / g) + ((v_y * pow(sin(alpha), 2) * v_s) / (g * sqrt(pow(sin(alpha), 2) * pow(v_s, 2) - 2 * g * z))) + 2 * cos(alpha) * sin(theta) * sin(alpha) * v_s / g + (cos(alpha) * sin(theta) / g) * (pow(sin(alpha), 2) * pow(v_s, 3) - 2 * g * z * v_s) / (sqrt(pow(sin(alpha), 2) * pow(v_s, 4) - 2 * g * z * pow(v_s, 2)));
    J[1][1] = ((v_s * sin(alpha) + sqrt(pow(v_s, 2) * pow(sin(alpha), 2) - 2 * g * z)) / g) * cos(alpha) * v_s * (cos(theta));
    return J;
}

double dragAccel(double vMag) {
    double Cd = 0.47; // Drag coefficient for a sphere / fuel
    Cd = 0.75; // Drag coefficient for a lunite
    double A =
        0.15 * 0.15 / 4 * 3.14159265358979323846; // Cross-sectional area in m^2 of a Fuel piece
    A = 0.0072; // Approximate cross-sectional area of a lunite
    double rho = 1.225; // Air density in kg/m^3
    double m = 0.23; // Mass in kg of a Fuel piece
    m = 0.069; // Mass in kg of a lunite
    double F_d = 0.5 * Cd * rho * A * vMag * vMag; // Drag force
    double a_d = F_d / m; // Acceleration due to drag
    return a_d;
}

class State {
public:
    double x, y, z;
    double vx, vy, vz;

    State() {
        x = 0;
        y = 0;
        z = 0;
        vx = 0;
        vy = 0;
        vz = 0;
    }

    State(double x_in, double y_in, double z_in, double vx_in, double vy_in, double vz_in) {
        x = x_in;
        y = y_in;
        z = z_in;
        vx = vx_in;
        vy = vy_in;
        vz = vz_in;
    }
};

/** Derivative function: returns dS/dt */
State get_derivative(State s) {
    double speed = sqrt(s.vx * s.vx + s.vy * s.vy + s.vz * s.vz);

    State deriv = State();
    deriv.x = s.vx;
    deriv.y = s.vy;
    deriv.z = s.vz;

    // Acceleration: Gravity + Drag (opposite to velocity direction)
    dragAccel(speed);
    deriv.vx = -(dragAccel(speed) * (s.vx / speed));
    deriv.vy = -(dragAccel(speed) * (s.vy / speed));
    deriv.vz = -g - (dragAccel(speed) * (s.vz / speed));

    return deriv;
}

// RK4 Step function
State rk4_step(State s, double dt) {
    State k1, k2, k3, k4, next_s;

    // k1 = f(t, s)
    k1 = get_derivative(s);

    // k2 = f(t + dt/2, s + k1 * dt/2)
    State s2 =
        State(
            s.x + k1.x * dt / 2,
            s.y + k1.y * dt / 2,
            s.z + k1.z * dt / 2,
            s.vx + k1.vx * dt / 2,
            s.vy + k1.vy * dt / 2,
            s.vz + k1.vz * dt / 2);
    k2 = get_derivative(s2);

    // k3 = f(t + dt/2, s + k2 * dt/2)
    State s3 =
        State(
            s.x + k2.x * dt / 2,
            s.y + k2.y * dt / 2,
            s.z + k2.z * dt / 2,
            s.vx + k2.vx * dt / 2,
            s.vy + k2.vy * dt / 2,
            s.vz + k2.vz * dt / 2);
    k3 = get_derivative(s3);

    // k4 = f(t + dt, s + k3 * dt)
    State s4 =
        State(
            s.x + k3.x * dt,
            s.y + k3.y * dt,
            s.z + k3.z * dt,
            s.vx + k3.vx * dt,
            s.vy + k3.vy * dt,
            s.vz + k3.vz * dt);
    k4 = get_derivative(s4);

    next_s = State();
    // Combine steps: s_next = s + (dt/6) * (k1 + 2k2 + 2k3 + k4)
    next_s.x = s.x + (dt / 6.0) * (k1.x + 2 * k2.x + 2 * k3.x + k4.x);
    next_s.y = s.y + (dt / 6.0) * (k1.y + 2 * k2.y + 2 * k3.y + k4.y);
    next_s.z = s.z + (dt / 6.0) * (k1.z + 2 * k2.z + 2 * k3.z + k4.z);
    next_s.vx = s.vx + (dt / 6.0) * (k1.vx + 2 * k2.vx + 2 * k3.vx + k4.vx);
    next_s.vy = s.vy + (dt / 6.0) * (k1.vy + 2 * k2.vy + 2 * k3.vy + k4.vy);
    next_s.vz = s.vz + (dt / 6.0) * (k1.vz + 2 * k2.vz + 2 * k3.vz + k4.vz);

    return next_s;
}

#define maxItersBisection  15
#define toleranceBisect    1e-3

glm::vec2 bisectionMethodRK4(State low, State high, double dt, double z) {
    for (int i = 0; i < maxItersBisection; i++) {
    dt /= 2.0;
    State mid = rk4_step(low, dt);
    if (mid.z > z) {
        low = mid;
    } else {
        high = mid;
    }
    // if (low.z - high.z < toleranceBisect) {
    //   break;
    // }
    }
    return glm::vec2(low.x, low.y);
}

#define simStepsRK4 60

glm::vec2 f_airResistance_RK4(
    glm::vec2 in, double v_x, double v_y, double z, double alpha) {
    double v_s = in.x;
    double theta = in.y;
    State state = State();
    state.x = 0;
    state.y = 0;
    state.z = 0;
    state.vx = v_x + v_s * cos(theta) * cos(alpha);
    state.vy = v_y + v_s * sin(theta) * cos(alpha);
    state.vz = v_s * sin(alpha);

    double totalTime = 4.0; // Simulate up to 4 seconds
    double dt = totalTime / simStepsRK4;

    for (int i = 0; i < simStepsRK4; i++) {
    State nextState = rk4_step(state, dt);

    if (nextState.vz < 0 && nextState.z < z) {
        return bisectionMethodRK4(state, nextState, dt, z);
    }

    state = nextState;
    }
    return glm::vec2(0, 0); // Did not hit target height within simulation time
}

#include <iostream>
using namespace std;

#define MAX_ITERS 7
TrajectorySolution newtonRhapsonSolveAirResistance(
    double botVelocityX,
    double botVelocityY,
    glm::vec3 targetPos,
    double shooterAltitude,
    TrajectorySolution guessSolution) {
    glm::vec2 guess =
        glm::vec2(
            guessSolution.shooterVelocity, guessSolution.azimuth
            );
    glm::vec2 x = guess;
    for (int count = 0; count < MAX_ITERS; count++) {
        glm::vec2 f_x =
            f_airResistance_RK4(
                    x,
                    botVelocityX,
                    botVelocityY,
                    targetPos.z,
                    shooterAltitude)
                - glm::vec2(targetPos.x, targetPos.y);
        if (count % 1 == 0) {
            // cout << "\niteration: " << (count + 1) << "  f_x:" << sqrt(f_x.x * f_x.x + f_x.y * f_x.y) << "\n";
            // cout << x.x << " " << x.y << "\n";
            // cout << f_x.x << " " << f_x.y << "\n";
        }
        // if (dist(f_x) < tolerance) return TrajectorySolution(x);
        glm::mat2 J =
            calcJacobian(
                x,
                botVelocityX,
                botVelocityY,
                targetPos.z,
                shooterAltitude);
        x = x -( (inverse(J)) * f_x);
    }
    return TrajectorySolution(x);
}

double calcTargetTime(double v_x, double v_y, glm::vec3 targetPos, double shooterAltitude);
double calcShooterVelocity(double targetTime, glm::vec3 targetPos, double shooterAltitude);
double calcRobotHeadingAzimuth(double v_x, double v_y, double targetTime, glm::vec3 targetPos, double shooterAltitude);

TrajectorySolution calcFiringSolution(double v_x,double v_y,glm::vec3 targetPos,double shooterAltitude){
    double targetTime = calcTargetTime(v_x, v_y, targetPos, shooterAltitude);
    double shooterVel = calcShooterVelocity(targetTime, targetPos, shooterAltitude);
    double azimuth = calcRobotHeadingAzimuth(v_x, v_y, targetTime, targetPos, shooterAltitude);
    return TrajectorySolution(glm::vec2(shooterVel, azimuth));
}


double calcRobotHeadingAzimuth(double v_x, double v_y, double targetTime, glm::vec3 targetPos, double shooterAltitude){
    double sinTheta =
        targetPos.y - targetTime * v_y;
    double cosTheta =
        targetPos.x - targetTime * v_x;
    return std::atan2(sinTheta, cosTheta);
}
double calcShooterVelocity(double timeOfFlight, glm::vec3 targetPos, double shooterAltitude){
    double numerator = targetPos.z + 0.5 * g * pow(timeOfFlight, 2.0);
    double denominator = timeOfFlight * sin(shooterAltitude);
    return numerator / denominator;
}

double solveQuartic(double a, double b, double c, double d, double e);
double solveDepressedQuarticRealHigh(double a, double b, double c, double d){
    return solveQuartic(a, 0, b, c, d);
}
double solveQuadraticRealHigh(double a, double b, double c);

double calcTargetTime(double v_x,double v_y,glm::vec3 targetPos,double shooterAltitude) {
    double cos2a = pow(cos(shooterAltitude), 2);
    double sin2a = pow(sin(shooterAltitude), 2);
    double a = 0.25 * pow(g, 2) * cos2a;
    double b = g * targetPos.z * cos2a - v_x * v_x * sin2a - v_y * v_y * sin2a;
    double c = 2.0 * (targetPos.x * v_x + targetPos.y * v_y) * sin2a;
    double d =
        pow(targetPos.z, 2) * cos2a
            - pow(targetPos.x, 2) * sin2a
            - pow(targetPos.y, 2) * sin2a;
    if (c == 0) {
        return sqrt(solveQuadraticRealHigh(a, b, d));
    } else {
        return solveDepressedQuarticRealHigh(a, b, c, d);
    }
}

double solveQuadraticRealHigh(double a, double b, double c) {
    if (a > 0) return (-b + sqrt(b * b - 4 * a * c)) / (2 * a);
    else return (-b - sqrt(b * b - 4 * a * c)) / (2 * a);
}

double solveQuartic(double a, double b, double c, double d, double e) {
    // Coefficients for: p(x) = a0 + a1*x + a2*x^2 + a3*x^3 + a4*x^4
    // Example: x^4 - 6x^3 + 11x^2 - 6x + 0 (roots are 0, 1, 2, 3)
    // double a4 = 1.0, a3 = -6.0, a2 = 11.0, a1 = -6.0, a0 = 0.0;
    double a4 = a, a3 = b, a2 = c, a1 = d, a0 = e;

    // 1. Normalize coefficients to make the polynomial monic
    double c3 = a3 / a4;
    double c2 = a2 / a4;
    double c1 = a1 / a4;
    double c0 = a0 / a4;

    // 2. Create the companion matrix
    Eigen::Matrix4d companion;
    companion << 0, 0, 0, -c0,
                 1, 0, 0, -c1,
                 0, 1, 0, -c2,
                 0, 0, 1, -c3;

    // 3. Compute eigenvalues (these are the roots)
    Eigen::EigenSolver<Eigen::Matrix4d> solver(companion);
    
    // std::cout << "The roots of the quartic polynomial are:\n" 
    //           << solver.eigenvalues() << std::endl;
              auto roots = solver.eigenvalues();

    double maxRealRoot = 0;
    // bool foundReal = false;
    const double epsilon = 1e-9; // Threshold for imaginary part

    for (int i = 0; i < roots.size(); ++i) {
        // Check if the root is essentially real
        if (std::abs(roots[i].imag()) < epsilon) {
            double currentReal = roots[i].real();
            if (currentReal > maxRealRoot) {
                maxRealRoot = currentReal;
                // foundReal = true;
            }
        }
    }
    // if (foundReal) {
        // std::cout << "Highest real root: " << maxRealRoot << std::endl;
    // } else {
    //     std::cout << "No real roots found." << std::endl;
    // }
    return maxRealRoot;
}