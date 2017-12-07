/***********************************************************************************************************************
 * @file voxel_save.cpp
 * @brief Template for acquiring PCL point clouds from a Voxel device
 *
 * Template for acquiring PCL point clouds from a Voxel device. Incoming data frames are handled asynchronously with a 
 * callback. Point clouds are saved upon pressing the 'p' key
 *
 * @author Christopher D. McMurrough
 **********************************************************************************************************************/

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Voxel.h>
#include <signal.h>
#include <thread>
#include <chrono>
#include <string> 

// declare the RGB-D grabber object
pcl::Grabber* Grabber = new Voxel::FrameGrabber();

// function prototypes
void cloudCallback(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloudIn);
void exitHandler(int terminationCode);

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
	
	// save the 100th frame
	if(frameCount == 100)
	{
		std::string fileName = "cloud.pcd";
	    if(pcl::io::savePCDFile<pcl::PointXYZRGBA>(fileName, *cloudIn, true) == -1)
		{
			std::printf("error while attempting to save pcd file: %s \n", fileName);
			return false;
		}
	}
}

/*******************************************************************************************************************//**
* @brief gracefully exits the program after deallocating shared memory variables
* @param[in] code exit status code
* @author Christopher D. McMurrough
***********************************************************************************************************************/
void exitHandler(int terminationCode)
{
    // shutdown the cameraGrabber
    Grabber->stop();

    // return termination code
    if(terminationCode == EXIT_FAILURE)
    {
        exit(EXIT_FAILURE);
    }
    else
    {
        exit(EXIT_SUCCESS);
    }
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
    // register callback functions to handle incoming data streams
    boost::function<void (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&)> f1 = boost::bind(&cloudCallback, _1);
    Grabber->registerCallback(f1);
	
    // create the program termination event handler (ctrl+c)
    struct sigaction sigIntHandler;
    sigIntHandler.sa_handler = exitHandler;
    sigemptyset(&sigIntHandler.sa_mask);
    sigIntHandler.sa_flags = 0;
    sigaction(SIGINT, &sigIntHandler, NULL);

    // start receiving frames
    Grabber->start();

    // wait until user quits program with ctrl+c
    while (true)
    {
        std::this_thread::sleep_for (std::chrono::milliseconds(100));
    }

    // stop the grabber
    Grabber->stop();

    // exit program
    return 0;
}
