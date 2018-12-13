#include <math.h>
#include <stdio.h>
#include <stdint.h>

typedef double (*xi)(double e_bar);

double plant(double u, double y)
{
    const double f = 0.01;             // damping (e.g. friction), but in time domian -> so increases cycle period! (i.e. "+")
    const double h = 0.1;

    //General equation of dynamics -> eq. 2 in the ABAG paper
    double ydot = f - h * u;

    //Euler integration...consider changing to Runge-Kutta method
    y += ydot;

    //Why? Properels can rotate in oposite direction when freewheeling mode is on and we have air resistance on
    // now it ReLU function applied on the value
    if (y < 0.0) y = 0.0;

    printf("ydot: %f\n", ydot);

    return y;
}

double xi_bias(double e_bar)
{
    const double e_b_bar = 0.75;

    if (e_bar < -e_b_bar) return -1.0;
    else if (e_bar > e_b_bar) return 1.0;
    else return 0.0;
}

double xi_gain(double e_bar)
{
    const double e_g_bar = 0.5;

    if (e_bar < -e_g_bar) return 1.0;
    else if (e_bar > e_g_bar) return 1.0;
    else return -1.0;
}

double sat(double value)
{
    if (value < 0.0) return 0.0;
    if (value > 1.0) return 1.0;
    return value;
}

double abag_ctrl(double y, double y_d, xi xi_bias, xi xi_gain)
{
    static double bias = 0.0;
    static double gain = 0.0;
    static double e_bar = 0.0;

    const double delta_gain = 0.001953;
    const double delta_bias = 0.000976;

    //Line 14 in ABAG paper implementation??    
    const double min_gain = 0.000976;

    double alpha = 0.75;

    if (y > y_d) e_bar = alpha * e_bar + (1.0 - alpha);
    else         e_bar = alpha * e_bar - (1.0 - alpha);

    bias = sat(bias + delta_bias * xi_bias(e_bar));
    gain = sat(gain + delta_gain * xi_gain(e_bar));

    double u;
    if (y > y_d) u = sat(bias + gain);
    else         u = sat(bias - gain);

    printf("e_bar [%]: %f\n", e_bar);
    printf("y: %f\n", y);
    printf("y_d: %f\n", y_d);
    printf("x: %f\n", 1.0 / y);
    printf("x_d: %f\n", 1.0 / y_d);
    printf("u [%]: %f\n", u);
    printf("gain [%]: %f\n", gain);
    printf("bias [%]: %f\n", bias);

    return u;
}


int main(int argc, char **argv)
{
    double x = 20.0;    // from 20 Hz
    double x_d = 50.0;  // to 50 Hz

    double y = 1.0 / x;
    double y_d = 1.0 / x_d;
    double u = 0.0;

    for (int i = 0; i < 1000; i++) {
        printf("Iteration: %i \n", i);
        u = abag_ctrl(y, y_d, &xi_bias, &xi_gain);
        y = plant(u, y);

        printf("\n");
    }

    return 0;
}