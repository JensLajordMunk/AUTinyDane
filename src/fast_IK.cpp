// verify_ik_fk.cpp
#include <iostream>
#include <fstream>
#include <cmath>
#include <algorithm>
#include <numbers>

struct Vector3 {
    double x;
    double y;
    double z;
};

struct ServoAngles {
    double theta_ab;
    double theta_thigh;
    double theta_shin;
};

class LegIK {
private:
    // Constant lengths
    static constexpr double O_AB = 0.044421905;
    static constexpr double L_thigh = 0.0800;
    static constexpr double L_shin = 0.08521850619;
    static constexpr double L_space = 0.02973213749;
    static constexpr double L_pizza = 0.02770000309;
    static constexpr double L_link = 0.0377;
    static constexpr double L_tie = 0.1015;
    static constexpr double L_servo = 0.0240;
    static constexpr double L_slink = 0.0300;
    static constexpr double r_foot = 0.01000000;
    static constexpr double servo_offset_x = 0.022;
    static constexpr double servo_offset_z = 0.020;

    // Constant angles
    static constexpr double c = 2.773216451;
    static constexpr double tau = 1.437802238;
    static constexpr double nu = 0.73781505;


    // Function that returns
    // (a^2 + b^2 - c^2) / (2*a*b)
    double cosLaw(double adj1, double adj2, double opp) const {
        const double numerator = adj1*adj1 + adj2*adj2 - opp*opp;
        const double denominator = 2 * adj1 * adj2;

        return std::acos(std::clamp(numerator/denominator, -1.0, 1.0));
    }

    // Function that returns
    // sqrt(a^2 + b^2 - 2*a*b*cos(angle))
    double calculateSplit(double adj1, double adj2, double angle) const {
        return std::sqrt(adj1*adj1 + adj2*adj2 - (2*adj1*adj2*std::cos(angle)));
    }


public:
    // Function that returns the ab/ad angle from the foot position in the ab/ad frame
    // This angle is only dependant on the y and z position of the foot
    ServoAngles inverseKinematics(Vector3 foot_pos, int leg_index) const {
        // Predefine variables
        ServoAngles angles;
        double current_O_AB = O_AB * ((leg_index % 2 == 0) ? 1.0 : -1.0);

        foot_pos.z = foot_pos.z + r_foot;

        // 1. ab/ad angle
        // Calculate intermediate lengths
        const double yz_D_AF = std::hypot(foot_pos.y, foot_pos.z);

        // Calculate intermediate angles
        const double alpha = std::atan2(foot_pos.z, foot_pos.y);
        const double beta = std::acos(current_O_AB/yz_D_AF);

        // Calculate abductor angle
        angles.theta_ab = alpha + beta;
        
        // 2. thigh angle
        // Calculate intermediate lengths
        const double yz_D_HF = std::sqrt(yz_D_AF*yz_D_AF - O_AB*O_AB);
        const double xzprime_D_HF = std::hypot(foot_pos.x, yz_D_HF);

        // Calculate intermediate angles
        const double gamma = std::atan2(foot_pos.x, yz_D_HF);
        const double psi = cosLaw(L_thigh, xzprime_D_HF, L_shin);

        // Calculate thigh angle
        angles.theta_thigh = psi - gamma;

        // 3. shin angle
        // Calculate phi from definition
        const double phi = cosLaw(L_thigh, L_shin, xzprime_D_HF);
        const double knee = std::numbers::pi-(angles.theta_thigh + phi);

        // Calculate split1 form definition
        //const double L_split1 = calculateSplit(L_link, L_thigh, (2*std::numbers::pi - phi - c));
        const double link_angle = c - knee - std::numbers::pi*0.5; // absolute angle of link endpoint
        const double x1 = L_link * std::cos(link_angle) + L_thigh * std::sin(angles.theta_thigh);
        const double z1 = -L_link * std::sin(link_angle) + L_thigh * std::cos(angles.theta_thigh);
        const double L_split1 = std::hypot(x1, z1);
        const double theta_trick = -0.5*std::numbers::pi+(cosLaw(L_split1,L_pizza,L_tie) + std::atan2(x1, z1));

        // Calculate mu from definition
        //const double mu = std::numbers::pi - cosLaw(L_split1, L_thigh, L_link) - cosLaw(L_split1, L_pizza, L_tie) - angles.theta_thigh;

        // Calculate omega from definition
        const double omega = std::numbers::pi - tau - theta_trick; //std::numbers::pi - tau - (std::numbers::pi*0.5-mu);

        // Calculate L split 2 from definition
        const double x2 = L_pizza * std::cos(omega) - servo_offset_x;
        const double z2 = L_pizza * std::sin(omega) + servo_offset_z;
        const double L_split2 = std::hypot(z2, x2);

        // Calculate gamma, zeta, and delta from definitions
        const double lambda = cosLaw(L_servo, L_split2, L_slink);
        const double zeta = std::atan2(z2, x2);
        std::cout << zeta << "\n";
        //const double delta = (std::numbers::pi / 2) - nu;

        // Calculate shin angle
        angles.theta_shin = lambda + zeta;

        // Return servo angles
        return angles;
    }
};

extern "C" {
    LegIK* LegIK_new() { return new LegIK(); }
    void LegIK_delete(LegIK* ik) { delete ik; }
    ServoAngles LegIK_ik(LegIK* ik, double x, double y, double z, int leg_index) {
        return ik->inverseKinematics({x, y, z}, leg_index);
    }
}