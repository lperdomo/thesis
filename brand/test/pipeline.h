#ifndef PIPELINE_H
#define PIPELINE_H

#include <vector>
#include <iostream>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include <opencv2/contrib/contrib.hpp>


#include <pcl/point_types.h>
#include <pcl/common/io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/range_image_visualizer.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "detector.h"
#include "match.h"
#include "input.h"
#include "extractor.h"

class Pipeline
{
public:
    Pipeline();
    void run(std::string rgbFile, std::string depthFile);
    Input<pcl::PointXYZ> depth;
    Input<pcl::PointXYZRGBA> rgb;
    const static int BRAND;
    const static int NARF;
    const static int NONE;
    const static int SIFT;
    const static int STAR;
    Detector detector;
    Extractor extractor;
};

#endif // PIPELINE_H
