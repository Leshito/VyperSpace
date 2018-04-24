/***********************************************************************************************************************
 * @file openni2_snapper.cpp
 * @brief Template for acquiring PCL point clouds from an OpenNI2 device
 *
 * Template for acquiring PCL point clouds from an OpenNI2 device. Incoming data streams from an OpenNI2 compliant
 * device are acquired and converted to PCL point clouds, which are then visualized in real time.
 *
 * @author Christopher D. McMurrough
 **********************************************************************************************************************/

#include <iostream>
#include <iomanip>
#include <thread>
#include <chrono>
#include <poll.h>

#include <pcl/io/openni2_grabber.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/common/time.h>
#include <pcl/io/pcd_io.h>
#include "CloudVisualizer.h"
#include "rewtMain.h"

using namespace std;

//the bounds of the passthrough filter
float zmin = 0.4, zmax = 1.2, ymin = -0.3, ymax = 0.05, xmin = -0.15, xmax = 0.155;

pcl::PointXYZRGBA cornerFTL;
pcl::PointXYZRGBA cornerFTR;
pcl::PointXYZRGBA cornerFBL;
pcl::PointXYZRGBA cornerFBR;
pcl::PointXYZRGBA cornerBTL;
pcl::PointXYZRGBA cornerBTR;
pcl::PointXYZRGBA cornerBBL;
pcl::PointXYZRGBA cornerBBR;


void drawWorkSpace(pcl::visualization::PCLVisualizer& viewer)
{
		viewer.addLine<pcl::PointXYZRGBA> (cornerFTL, cornerFTR, 255, 0, 0, "FTLtoFTR");
		viewer.addLine<pcl::PointXYZRGBA> (cornerFTL, cornerFBL, 255, 0, 0, "FTLtoFBL");
		viewer.addLine<pcl::PointXYZRGBA> (cornerFTL, cornerBTL, 255, 0, 0, "FTLtoBTL");
		viewer.addLine<pcl::PointXYZRGBA> (cornerFTR, cornerBTR, 255, 0, 0, "FTRtoBTR");
		viewer.addLine<pcl::PointXYZRGBA> (cornerFTR, cornerFBR, 255, 0, 0, "FTRtoFBR");
		viewer.addLine<pcl::PointXYZRGBA> (cornerFBR, cornerFBL, 255, 0, 0, "FBRtoFBL");
		viewer.addLine<pcl::PointXYZRGBA> (cornerFBR, cornerBBR, 255, 0, 0, "FBRtoBBR");
		viewer.addLine<pcl::PointXYZRGBA> (cornerBTL, cornerBBL, 255, 0, 0, "BTLtoBBL");
		viewer.addLine<pcl::PointXYZRGBA> (cornerBTL, cornerBTR, 255, 0, 0, "BTLtoBTR");
		viewer.addLine<pcl::PointXYZRGBA> (cornerBBL, cornerBBR, 255, 0, 0, "BBLtoBBR");
		viewer.addLine<pcl::PointXYZRGBA> (cornerBBL, cornerFBL, 255, 0, 0, "BBLtoFBL");
		viewer.addLine<pcl::PointXYZRGBA> (cornerBTR, cornerBBR, 255, 0, 0, "BTRtoBBR");
		

}

/***********************************************************************************************************************
 * @class OpenNI2Processor
 * @brief Class containing data acquisition mechanics for OpenNI2 devices
 * @author Christopher D. McMurrough
 **********************************************************************************************************************/
class OpenNI2Processor
{
private:

    struct pollfd stdin_poll = {.fd = STDIN_FILENO, .events = POLLIN | POLLRDBAND | POLLRDNORM | POLLPRI };

    // create a stop watch for measuring time
    pcl::StopWatch m_stopWatch;

    // create the cloud viewer object
    pcl::visualization::CloudViewer m_viewer;

    char input = '_';


public:

    /***********************************************************************************************************************
     * @brief Class constructor
     * @param[in] cloudRenderSetting sets the cloud visualization mode (render_off:0, render_on:1)
     * @param[in] cloudSaveSetting sets the disk save mode for cloud data (saves_off:0, saves_on:1)
     * @author Christopher D. McMurrough
     **********************************************************************************************************************/
    OpenNI2Processor() : m_viewer("Rendering Window") {}

    /***********************************************************************************************************************
     * @brief Starts data acquisition and handling
     * @author Christopher D. McMurrough
     **********************************************************************************************************************/
    void run()
    {	


        // create a new grabber for OpenNI2 devices
        pcl::Grabber* interface = new pcl::io::OpenNI2Grabber();
        
        // bind the callbacks to the appropriate member functions
        boost::function<void (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&)> f = boost::bind(&OpenNI2Processor::cloudCallback, this, _1);

        // connect callback function for desired signal. In this case its a point cloud with color values
        interface->registerCallback(f);

        // start receiving point clouds
        interface->start();

        // start the timer
        m_stopWatch.reset();

        m_viewer.runOnVisualizationThreadOnce(drawWorkSpace);

        // wait until user quits program
        while (!m_viewer.wasStopped())
        {
            std::this_thread::sleep_for (std::chrono::milliseconds(100));
        }

        // stop the grabber
        interface->stop();
    }

    /***********************************************************************************************************************
     * @brief Callback function for received cloud data
     * @param[in] cloudIn the raw cloud data received by the OpenNI2 device
     * @author Christopher D. McMurrough
     **********************************************************************************************************************/
    void cloudCallback(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloudIn)
    {	

        // get the elapsed time since the last callback
        double elapsedTime = m_stopWatch.getTimeSeconds();
        m_stopWatch.reset();

        if (poll(&stdin_poll, 1, 0) == 1)
        {
            scanf("%c", &input);
            if (input == 'v')
            {
		  		m_viewer.showCloud(cloudIn);
                rewtMain(cloudIn);
                while ((getchar()) != '\n');
            }
        }

        m_viewer.showCloud(cloudIn);

    }
};


/***********************************************************************************************************************
 * @brief program entry point
 * @param[in] argc number of command line arguments
 * @param[in] argv string array of command line arguments
 * @returnS return code (0 for normal termination)
 * @author Christopher D. McMurrough
 **********************************************************************************************************************/
int main (int argc, char** argv)
{

	cornerFTL.x = xmin;
	cornerFTL.y = ymax;
	cornerFTL.z = zmin;

	cornerFTR.x = xmax;
	cornerFTR.y = ymax;
	cornerFTR.z = zmin;

	cornerFBL.x = xmin;
	cornerFBL.y = ymin;
	cornerFBL.z = zmin;

	cornerFBR.x = xmax;
	cornerFBR.y = ymin;
	cornerFBR.z = zmin;

	cornerBTL.x = xmin;
	cornerBTL.y = ymax;
	cornerBTL.z = zmax;

	cornerBTR.x = xmax;
	cornerBTR.y = ymax;
	cornerBTR.z = zmax;

	cornerBBL.x = xmin;
	cornerBBL.y = ymin;
	cornerBBL.z = zmax;

	cornerBBR.x = xmax;
	cornerBBR.y = ymin;
	cornerBBR.z = zmax;

    // create the processing object
    OpenNI2Processor ONI2Processor;

    // start the processing object
    ONI2Processor.run();

    // exit program
    return 0;
}