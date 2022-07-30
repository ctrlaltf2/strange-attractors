#include <chrono>
#include <iostream>
#include <random>

// TODO: Make a CMake option
//#define CUDA

#ifdef CUDA
#include <pcl/cuda/filters/voxel_grid.h>
#else
#include <pcl/filters/voxel_grid.h>
#endif

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

#define QUAD_3

int main() {
// https://sequelaencollection.home.blog/3d-chaotic-attractors/
#ifdef QUAD_3
    std::cout << "3D Quadratic System" << '\n';
    const std::array<const double, 30> a = {
        -0.875,
        -0.173,
         0.307,
        -0.436,
         0.598,
         0.003,
        -0.039,
         0.96,
        -0.84,
         0.885,
         0.774,
         0.281,
        -0.015,
         0.585,
         0.442,
        -0.18,
        -0.535,
        -0.151,
        -0.971,
        -0.48,
         0.777,
         0.418,
         0.185,
         0.006,
         0.45,
        -0.066,
         0.498,
         0.142,
        -0.246,
        -0.939
    };

    auto next_x = [=](Vec4& pt) {
        const auto& x = pt.x;
        const auto& y = pt.y;
        const auto& z = pt.z;

        return  a[0] +
                a[1]*x + a[2]*y + a[3]*z +
                a[4]*x*y + a[5]*x*z + a[6]*y*z +
                a[7]*x*x + a[8]*y*y + a[9]*z*z;
    };

    auto next_y = [=](Vec4& pt) {
        const auto& x = pt.x;
        const auto& y = pt.y;
        const auto& z = pt.z;

        return  a[10] +
                a[11]*x + a[12]*y + a[13]*z +
                a[14]*x*y + a[15]*x*z + a[16]*y*z +
                a[17]*x*x + a[18]*y*y + a[19]*z*z;
    };

    auto next_z = [=](Vec4& pt) {
        const auto& x = pt.x;
        const auto& y = pt.y;
        const auto& z = pt.z;

        return  a[20] +
                a[21]*x + a[22]*y + a[23]*z +
                a[24]*x*y + a[25]*x*z + a[26]*y*z +
                a[27]*x*x + a[28]*y*y + a[29]*z*z;
    };

    Attractor attractor(next_x, next_y, next_z);

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>());
    for(unsigned i = 0; i < 15; ++i) {
        std::cout << "Iteration #" << i << '\n';

        // Run some iterations, until memory is almost capped out
        attractor.run(100000000);

        // Downsample cloud into new cloud
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGBA>());

#ifdef CUDA
        pcl_cuda::VoxelGrid<pcl::PointXYZRGBA> sor;
#else
        pcl::VoxelGrid<pcl::PointXYZRGBA> sor;
#endif

        sor.setInputCloud(attractor.cloud);
        sor.setLeafSize(0.5f, 0.5f, 0.5f);
        sor.setMinimumPointsNumberPerVoxel(3); // Helps to highlight lines forming from dots, tends to filter stray points in between structures
        sor.filter(*cloud_filtered);

        // Append downsampled cloud to ongoing end cloud
        *cloud += *cloud_filtered;

        // Clean up old cloud, continue generating points
        attractor.cloud.reset(new pcl::PointCloud<pcl::PointXYZRGBA>());
    }

    // Downsample final cloud
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGBA>());
#ifdef CUDA
    pcl_cuda::VoxelGrid<pcl::PointXYZRGBA> sor;
#else
    pcl::VoxelGrid<pcl::PointXYZRGBA> sor;
#endif
    sor.setInputCloud(cloud);
    sor.setLeafSize(0.5f, 0.5f, 0.5f);
    sor.setMinimumPointsNumberPerVoxel(5); // Helps to highlight lines forming from dots, tends to filter stray points in between structures
    sor.filter(*cloud_filtered);
    attractor.cloud.reset();

    std::cout << "Saving to ./out.pcd.\n";
    pcl::io::savePCDFile("./out.pcd", *cloud_filtered);

    std::cout << "Plotting " << cloud_filtered->size() << " points\n";

    pcl::visualization::CloudViewer viewer("Cloud Viewer");
    viewer.showCloud(cloud_filtered);
#else
    std::cout << "Cliffod 2D attractor" << '\n';
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

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>());
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

        *cloud += *Clifford.cloud;
    }

    pcl::visualization::CloudViewer viewer("Cloud Viewer");
    // Downsample for visualization
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGBA>());
#ifdef CUDA
    pcl_cuda::VoxelGrid<pcl::PointXYZRGBA> sor;
#else
    pcl::VoxelGrid<pcl::PointXYZRGBA> sor;
#endif
    sor.setInputCloud(cloud);
    sor.setLeafSize(0.5f, 0.5f, 0.5f);
    sor.filter(*cloud_filtered);

    std::cout << "Before filter: " << cloud->size() << " points\n";
    std::cout << "After filter: " << cloud_filtered->size() << " points\n";

    viewer.showCloud(cloud_filtered);
#endif
    //blocks until the cloud is actually rendered
    // viewer.showCloud(cloud_filtered);

    while(!viewer.wasStopped()) { }

    return 0;
}
