#include <chrono>
#include <iostream>
#include <random>

#include <pcl/filters/voxel_grid.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>

#include "Attractor.hpp"
#include "Vec4.hpp"


float pick(float a, float b) {
    static std::random_device r;
    static std::default_random_engine generator(r());
    static std::uniform_real_distribution distribution(a, b);

    return distribution(generator);
}

int main() {
    /*
    double a = pick(-1, 1);
    double b = pick(-1, 1);
    double c = pick(-1, 1);
    double d = pick(-1, 1);
    double e = pick(-1, 1);
    double f = pick(-1, 1);*/

    double a = 1.7;
    double b = 1.7;
    double c = 0.6;
    double d = 1.2;
    double e = 0.0;
    double f = 0.0;

    std::cout
        << a << '\n'
        << b << '\n'
        << c << '\n'
        << d << '\n'
        << e << '\n'
        << f << std::endl;

    // Rampe1
    /*
    auto next_x = [=](Vec4& pt) {
        return pt.z * sin(a * pt.x) + cos(b * pt.y);
    };

    auto next_y = [=](Vec4& pt) {
        return pt.x * sin(c * pt.y) + cos(d * pt.z);
    };

    auto next_z = [=](Vec4& pt) {
        return pt.y * sin(e * pt.z) + cos(f * pt.x);
    };*/

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_full(new pcl::PointCloud<pcl::PointXYZRGBA>());
    for(unsigned i = 0; i < 1; ++i) {
        const double a = 1.7 + 0.05*i;
        std::cout << "iteration " << i << ", a = " << a << '\n';

        // Clifford
        auto next_x = [=](Vec4& pt) {
            return sin(a * pt.y) + c * cos(a * pt.x);
        };

        auto next_y = [=](Vec4& pt) {
            return sin(b * pt.x) + d * cos(b * pt.y);
        };

        // Instantiate a 2D attractor in the clifford style
        Attractor Clifford(next_x, next_y);

        Clifford.run(100000000);

        *cloud_full += *Clifford.cloud;
    }

    pcl::visualization::CloudViewer viewer("Cloud Viewer");
    // Downsample for visualization
    /*
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGBA>());
pcl::VoxelGrid<pcl::PointXYZRGBA> sor;
    sor.setInputCloud(Clifford.cloud);
    sor.setLeafSize(0.01f, 0.01f, 0.01f);
    sor.filter(*cloud_filtered);*/

    //std::cout << "Before filter: " << Clifford.cloud->size() << " points\n";
    //std::cout << "After filter: " << cloud_filtered->size() << " points\n";

    //blocks until the cloud is actually rendered
    // viewer.showCloud(cloud_filtered);
    viewer.showCloud(cloud_full);

    while(!viewer.wasStopped()) { }

    return 0;
}
