/***********************************************************************************************************************
 * @file voxel_viewer.cpp
 * @brief Template for acquiring PCL point clouds from a Voxel device
 *
 * Template for acquiring PCL point clouds from a Voxel device. Incoming data streams from a Voxel device are acquired
 * and converted to PCL point clouds, which are then visualized in real time. This sample also demonstrates PCL and
 * OpenCV integration.
 *
 * @author Christopher D. McMurrough
 **********************************************************************************************************************/


#include <thread>
#include <chrono>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include "opencv2/opencv.hpp"
#include <Voxel.h>

// global variables
pcl::visualization::CloudViewer Viewer("Point Cloud");

// function prototypes
void cloudCallback(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloudIn);
void ImageCallback(const boost::shared_ptr<pcl::io::Image>& color, const boost::shared_ptr<pcl::io::DepthImage>& depth, float focalLengthInverse);

/***********************************************************************************************************************
 * @brief Callback function for received cloud data
 * @param[in] cloudIn the raw cloud data received by the grabber
 * @author Christopher D. McMurrough
 **********************************************************************************************************************/
void cloudCallback(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloudIn)
{
    // print frame count
    static int frameCount = 0;
    std::cout << "Point cloud frame " << frameCount << " acquired" << std::endl;
    frameCount++;

    // render point cloud
    Viewer.showCloud(cloudIn);
}

/*******************************************************************************************************************//**
* @brief handles the processing of a received color + depth image frame
* @param[in] color the incoming color frame acquired from the RGB-D device
* @param[in] depth the incoming depth frame from the RGB-D device
* @param[in] focalLengthInverse inverse focal length of the depth sensor
* @author Christopher D. McMurrough
***********************************************************************************************************************/
void ImageCallback(const boost::shared_ptr<pcl::io::Image>& color, const boost::shared_ptr<pcl::io::DepthImage>& depth, float focalLengthInverse)
{
    // print frame count
    static int frameCount = 0;
    std::cout << "RGB+D image frame " << frameCount << " acquired" << std::endl;
    frameCount++;

    // copy the color image frame
    cv::Mat colorImage = cv::Mat(color->getHeight(), color->getWidth(), CV_8UC3);
    color->fillRGB(colorImage.cols, colorImage.rows, colorImage.data, colorImage.step);
    cv::cvtColor(colorImage, colorImage, CV_RGB2BGR);

    // copy the depth image frame
    cv::Mat depthImage = cv::Mat(depth->getHeight(), depth->getWidth(), CV_32F);
    depth->fillDepthImage(depthImage.cols, depthImage.rows, (float *) depthImage.data, depthImage.step);

    // display image frames
    cv::imshow("Color Image", colorImage);
    cv::imshow("Depth Image", depthImage);
    cv::waitKey(10);
}

/***********************************************************************************************************************
 * @brief program entry point
 * @param[in] argc number of command line arguments
 * @param[in] argv string array of command line arguments
 * @returnS return code (0 for normal termination)
 * @author Christopher D. McMurrough
 **********************************************************************************************************************/
int main (int argc, char** argv)
{
    // create a new Voxel frame grabber
    pcl::Grabber* grabber = new Voxel::FrameGrabber();

    // register callback functions to handle incoming data streams
    boost::function<void (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&)> f1 = boost::bind(&cloudCallback, _1);
    boost::function<void (const boost::shared_ptr<pcl::io::Image>&, const boost::shared_ptr<pcl::io::DepthImage>&, float focalLengthInverse)> f2 = boost::bind(&ImageCallback, _1, _2, _3);
    grabber->registerCallback(f1);
    grabber->registerCallback(f2);

    // start receiving frames
    grabber->start();

    // wait until user quits program
    while (!Viewer.wasStopped())
    {
        std::this_thread::sleep_for (std::chrono::milliseconds(100));
    }

    // stop the grabber
    grabber->stop();
    grabber->stop();

    // exit program
    return 0;
}
