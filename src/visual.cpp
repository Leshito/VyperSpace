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
#include <fstream>
#include <string>


#include <pcl/io/openni2_grabber.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/common/time.h>
//#include <boost/make_shared.hpp>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include "CloudVisualizer.h"
#include "rewtMain.h"
#include "armGrabber.h"

using namespace std;
typedef pcl::PointCloud<pcl::PointXYZRGBA> CloudT;
pcl::PointCloud<pcl::PointXYZRGBA>::Ptr armPtr (new pcl::PointCloud<pcl::PointXYZRGBA>);
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
  pcl::PCDWriter writer;
    /***********************************************************************************************************************
     * @brief Callback function for received cloud data
     * @param[in] cloudIn the raw cloud data received by the OpenNI2 device
     * @author Christopher D. McMurrough
     **********************************************************************************************************************/
    void cloudCallback(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloudIn)
    {
			//printf("print\n");
         //std::vector < pcl::PointCloud<pcl::PointXYZRGBA>::Ptr, Eigen::aligned_allocator <pcl::PointCloud <pcl::PointXYZRGBA>::Ptr > > armSnap;
        //CloudT::Ptr arm(new CloudT);
				//pcl::PointCloud<pcl::PointXYZRGBA> arm;

				//armPtr = arm.makeShared();
				        // get the elapsed time since the last callback
        double elapsedTime = m_stopWatch.getTimeSeconds();
        m_stopWatch.reset();

        if (poll(&stdin_poll, 1, 0) == 1)
        {
            scanf("%c", &input);
            if (input == 'v')
            {
		  		      m_viewer.showCloud(cloudIn);
                writer.write<pcl::PointXYZRGBA> ("arm_before_rewt.pcd", *armPtr, false);
                	rewtMain(cloudIn,armPtr);


                while ((getchar()) != '\n');
            }
						else if (input == 'a')
            {
		  		      m_viewer.showCloud(cloudIn);
                armPtr = armGrabber(cloudIn);
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
	string line;
	string tok;
char *tokens;
float boxParams[6] = {0, 0, 0, 0, 0, 0};
double x;





ifstream inconfig;
inconfig.open("configuration.txt");
if (inconfig.is_open())
{
	getline(inconfig, line);
	getline(inconfig, line);
	getline(inconfig, line);
	std::cout << line << std::endl;
	std::stringstream stream(line);

	//stream >> space;
	//std::cout << space << std::endl;
	//stream >> space;
	//std::cout << space << std::endl;
	int it=0;
//	stream >> tok;
	//std::cout << tok << std::endl;
	//std::cout << "Mu dfja;skdlfj;s\n";
	while (stream)
	{
		stream >> boxParams[it];
		//boxParams[it]= std::stof(tok);
		//stream >> space;
		//std::cout << boxParams[it] << std::endl;
		//boxParams[it]= x;
		//std::cout << boxParams[it] << std::endl;
		it++;
	}

	for(int x=0; x<6;x++)
		std::cout << boxParams[x] << std::endl;
	inconfig.close();
}
  //boxParams contents
	// [0]  [1] [2]  [3]  [4]  [5]
	//xmax xmin ymax ymin zmax zmin

	cornerFTL.x = boxParams[1];
	cornerFTL.y = boxParams[2];
	cornerFTL.z = boxParams[5];

	cornerFTR.x = boxParams[0];
	cornerFTR.y = boxParams[2];
	cornerFTR.z = boxParams[5];

	cornerFBL.x = boxParams[1];
	cornerFBL.y = boxParams[3];
	cornerFBL.z = boxParams[5];

	cornerFBR.x = boxParams[0];
	cornerFBR.y = boxParams[3];
	cornerFBR.z = boxParams[5];

	cornerBTL.x = boxParams[1];
	cornerBTL.y = boxParams[2];
	cornerBTL.z = boxParams[4];

	cornerBTR.x = boxParams[0];
	cornerBTR.y = boxParams[2];
	cornerBTR.z = boxParams[4];

	cornerBBL.x = boxParams[1];
	cornerBBL.y = boxParams[3];
	cornerBBL.z = boxParams[4];

	cornerBBR.x = boxParams[0];
	cornerBBR.y = boxParams[3];
	cornerBBR.z = boxParams[4];

    // create the processing object
    OpenNI2Processor ONI2Processor;

    // start the processing object
    ONI2Processor.run();

    // exit program
    return 0;
}
