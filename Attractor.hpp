#pragma once
#include <fstream>
#include <functional>
#include <vector>

#include <boost/multiprecision/gmp.hpp>

#include "Vec4.hpp"

using namespace boost::multiprecision;

template<typename X, typename Y, typename Z = Y, typename W = Z>
class Attractor {

    // Functors/functions used to generate coordinates
    std::function<mpf_float(Vec4&)> next_x;
    std::function<mpf_float(Vec4&)> next_y;
    std::function<mpf_float(Vec4&)> next_z = [](Vec4& z) { return 0; };
    std::function<mpf_float(Vec4&)> next_w = [](Vec4& w) { return 0; };

    // Current state
    Vec4 state_{0, 0, 0, 0};
public:
    /** Number of dimensions this attractor represents */
    unsigned dim;

    std::vector<Vec4> points;

    /** 2-dimensional constructor */
    Attractor(X x_func, Y y_func) {
        next_x = x_func;
        next_y = y_func;
        dim = 2;
    }

    /** 3-dimensional constructor */
    Attractor(X x_func, Y y_func, Z z_func) {
        next_x = x_func;
        next_y = y_func;
        next_z = z_func;
        dim = 3;
    }

    /** 4-dimensional constructor */
    Attractor(X x_func, Y y_func, Z z_func, W w_func) {
        next_x = x_func;
        next_y = y_func;
        next_z = z_func;
        next_w = w_func;
        dim = 4;
    }

    void set_initial_point(double x_, double y_, double z_ = 0, double w_ = 0) {
        state_.x = x_;
        state_.y = y_;
        state_.z = z_;
        state_.w = w_;
    }

    /** Run one iteration */
    inline void update() {
        // Save state
        points.push_back(state_);

        // And then update it
        mpf_float x = next_x(state_);
        mpf_float y = next_y(state_);
        mpf_float z = next_z(state_);
        mpf_float w = next_w(state_);

        state_ = Vec4{x, y, z, w};
    }

    /** Run for iter iterations */
    void run(unsigned long long iter) {
        for(unsigned long long i = 0; i < iter; ++i) {
            if(i % 2048 == 0) {
                std::cout << "Iteration #" << i << '\n';

                std::cout
                    << "\tState = ("
                    << state_.x << ", " << state_.y
                    << state_.z << ", " << state_.w
                    << std::endl;
            }

            update();
        }
    }

    /** Dump currently generated points to a CSV */
    void dump_csv(std::filesystem::path path) {
        std::ofstream ofs(path);

        switch(dim) {
            case 4:
                ofs << "x,y,z,w\n";
                break;
            case 3:
                ofs << "x,y,z\n";
                break;
            case 2:
                ofs << "x,y\n";
                break;
            default:
                break;
        }

        ofs.precision(12);

        for(auto vec : points) {
            /*double x = vec.x;
            double y = vec.y;
            std::cout << x << ' ' << y << '\n';*/
            ofs << vec.x << ',' << vec.y;

            if(dim >= 3)
                ofs << ',' << vec.z;

            if(dim >= 4)
                ofs << ',' << vec.w;

            ofs << '\n';
        }
    }
};
