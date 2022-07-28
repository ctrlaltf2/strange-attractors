#include <pcl/visualization/cloud_viewer.h>
#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>

#include "Attractor.hpp"
#include "Vec4.hpp"

int main() {
    double a =  1.6;
    double b =  1.2;
    double c = -1.9;
    double d =  1.6;

    auto next_x = [=](Vec4& pt) {
        return sin(a * pt.y) + c*cos(a * pt.x);
    };

    auto next_y = [=](Vec4& pt) {
        return sin(b * pt.x) + d*cos(b * pt.y);
    };

    // Instantiate a 2D attractor in the clifford style
    Attractor Clifford(next_x, next_y);

    Clifford.run(10000000);

    pcl::visualization::CloudViewer viewer("Cloud Viewer");

    //blocks until the cloud is actually rendered
    viewer.showCloud(Clifford.cloud);

    while(!viewer.wasStopped()) { }

    return 0;
}
