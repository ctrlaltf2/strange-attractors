#pragma once
#include <algorithm>
#include <cmath>
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

    double max_dist = 0.0;

    /** Run one iteration */
    inline void update() {
        bigfloat x = next_x(state_);
        bigfloat y = next_y(state_);
        bigfloat z = next_z(state_);
        bigfloat w = next_w(state_);

        bigfloat dx = state_.x - x;
        bigfloat dy = state_.y - y;
        bigfloat dz = state_.z - z;
        bigfloat dw = state_.w - w;

        bigfloat dist = std::sqrt(dx*dx + dy*dy + dz*dz + dw*dw);
        max_dist = std::max(max_dist, dist);
        bigfloat dist_ratio = std::clamp(dist / 3.0, 0.0, 1.0);

        // Assign on gradient from #ac1616 to #f3d035 based on distance travelled
        const uint8_t r = dist_ratio * 236 + ((1 - dist_ratio) * 50);
        const uint8_t g = dist_ratio *   9 + ((1 - dist_ratio) * 236);
        const uint8_t b = dist_ratio *   9 + ((1 - dist_ratio) * 9);

        const pcl::PointXYZRGBA point(100*state_.x, 100*state_.y, 100*state_.z, r, g, b, 32);
        cloud->push_back(point);

        state_ = Vec4{x, y, z, w};
    }

    /** Run for iter iterations */
    void run(unsigned long long iter) {
        for(unsigned long long i = 0; i < iter; ++i) {
            /*if(i % 8192 == 0) {
                std::cout << "Iteration #" << i << '\n';

                std::cout
                    << "\tState = ("
                    << state_.x << ", " << state_.y << ", "
                    << state_.z << ", " << state_.w << ")"
                    << std::endl;
            }*/

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
