#pragma once
#include <filesystem>
#include <fstream>
#include <functional>
#include <iostream>
#include <vector>

#include <pcl/impl/point_types.hpp>
#include <pcl/point_cloud.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>

#include "multiprecision.hpp"
#include "Vec4.hpp"

template<typename X, typename Y, typename Z = Y, typename W = Z>
class Attractor {

    // Functors/functions used to generate coordinates
    std::function<bigfloat(Vec4&)> next_x;
    std::function<bigfloat(Vec4&)> next_y;
    std::function<bigfloat(Vec4&)> next_z = [](Vec4& z) { return 0; };
    std::function<bigfloat(Vec4&)> next_w = [](Vec4& w) { return 0; };

    // Current state
    Vec4 state_{0.1, 0.2, 0.3, 0.4};
public:
    /** Number of dimensions this attractor represents */
    unsigned dim;

    std::vector<Vec4> points;

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud{nullptr};

    /** 2-dimensional constructor */
    Attractor(X x_func, Y y_func) {
        next_x = x_func;
        next_y = y_func;
        dim = 2;

        cloud.reset(new pcl::PointCloud<pcl::PointXYZRGBA>());
    }

    /** 3-dimensional constructor */
    Attractor(X x_func, Y y_func, Z z_func) {
        next_x = x_func;
        next_y = y_func;
        next_z = z_func;
        dim = 3;

        cloud.reset(new pcl::PointCloud<pcl::PointXYZRGBA>());
    }

    /** 4-dimensional constructor */
    Attractor(X x_func, Y y_func, Z z_func, W w_func) {
        next_x = x_func;
        next_y = y_func;
        next_z = z_func;
        next_w = w_func;
        dim = 4;

        cloud.reset(new pcl::PointCloud<pcl::PointXYZRGBA>());
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

        const pcl::PointXYZRGBA point(state_.x, state_.y, state_.z, 255, 255, 255, 32);
        cloud->push_back(point);

        // And then update it
        bigfloat x = next_x(state_);
        bigfloat y = next_y(state_);
        bigfloat z = next_z(state_);
        bigfloat w = next_w(state_);

        state_ = Vec4{x, y, z, w};
    }

    /** Run for iter iterations */
    void run(unsigned long long iter) {
        for(unsigned long long i = 0; i < iter; ++i) {
            if(i % 8192 == 0) {
                std::cout << "Iteration #" << i << '\n';

                std::cout
                    << "\tState = ("
                    << state_.x << ", " << state_.y << ", "
                    << state_.z << ", " << state_.w << ")"
                    << std::endl;
            }

            update();
        }
    }

    /** Dump currently generated points to a CSV */
    void dump_csv(std::filesystem::path path) {
        return;

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
