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
    double a = pick(-1, 1);
    double b = pick(-1, 1);
    double c = pick(-1, 1);
    double d = pick(-1, 1);
    double e = pick(-1, 1);
    double f = pick(-1, 1);

    // Rampe1
    auto next_x = [=](Vec4& pt) {
        return pt.z * sin(a * pt.x) + cos(b * pt.y);
    };

    auto next_y = [=](Vec4& pt) {
        return pt.x * sin(c * pt.y) + cos(d * pt.z);
    };

    auto next_z = [=](Vec4& pt) {
        return pt.y * sin(e * pt.z) + cos(f * pt.x);
    };

    // Clifford
    /*
    auto next_x = [=](Vec4& pt) {
        return sin(pt.z * pt.y) + c * cos(pt.z * pt.x);
    };

    auto next_y = [=](Vec4& pt) {
        return sin(b * pt.x) + d * cos(b * pt.y);
    };

    auto next_z = [=](Vec4& pt) {
        return pick(-1, 1);
    };*/

    // Instantiate a 2D attractor in the clifford style
    Attractor Clifford(next_x, next_y, next_z);

    Clifford.run(10000000);

    std::cout
        << a << '\n'
        << b << '\n'
        << c << '\n'
        << d << '\n'
        << e << '\n'
        << f << std::endl;

    pcl::visualization::CloudViewer viewer("Cloud Viewer");

    // Downsample for visualization
    /*
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGBA>());
pcl::VoxelGrid<pcl::PointXYZRGBA> sor;
    sor.setInputCloud(Clifford.cloud);
    sor.setLeafSize(0.1f, 0.1f, 0.1f);
    sor.filter(*cloud_filtered);*/

    std::cout << "Before filter: " << Clifford.cloud->size() << " points\n";
    //std::cout << "After filter: " << cloud_filtered->size() << " points\n";

    //blocks until the cloud is actually rendered
    //viewer.showCloud(cloud_filtered);
    viewer.showCloud(Clifford.cloud);

    while(!viewer.wasStopped()) { }

    return 0;
}
