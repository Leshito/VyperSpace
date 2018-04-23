/***********************************************************************************************************************
* @file display_pcd.cpp
* @brief load and display PCD data files
*
* Simple example of loading and displaying PCD files, can be used as a template for processing saved data
*
* @author Christopher D. McMurrough
**********************************************************************************************************************/

#include "CloudVisualizer.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/common/time.h>
#include "load_pcd.h"

#define NUM_COMMAND_ARGS 1

using namespace std;

// function prototypes
void pointPickingCallback(const pcl::visualization::PointPickingEvent& event, void* cookie);
void keyboardCallback(const pcl::visualization::KeyboardEvent &event, void* viewer_void);

/***********************************************************************************************************************
* @brief callback function for handling a point picking event
* @param[in] event handle generated by the visualization window
* @param[in] cookie user data passed by the event
* @author Christoper D. McMurrough
**********************************************************************************************************************/
void pointPickingCallback(const pcl::visualization::PointPickingEvent& event, void* cookie)
{
    static int pickCount = 0;
    static pcl::PointXYZRGBA lastPoint;

    pcl::PointXYZRGBA p;
    event.getPoint(p.x, p.y, p.z);

    cout << "POINT CLICKED: " << p.x << " " << p.y << " " << p.z << endl;

    // if we have picked a point previously, compute the distance
    if(pickCount % 2 == 1)
    {
        double d = std::sqrt((p.x - lastPoint.x) * (p.x - lastPoint.x) + (p.y - lastPoint.y) * (p.y - lastPoint.y) + (p.z - lastPoint.z) * (p.z - lastPoint.z));
        cout << "DISTANCE BETWEEN THE POINTS: " << d << endl;
    }

    // update the last point and pick count
    lastPoint.x = p.x;
    lastPoint.y = p.y;
    lastPoint.z = p.z;
    pickCount++;
}

/***********************************************************************************************************************
* @brief callback function for handling a keypress event
* @param[in] event handle generated by the visualization window
* @param[in] viewer_void user data passed by the event
* @author Christoper D. McMurrough
**********************************************************************************************************************/
void keyboardCallback(const pcl::visualization::KeyboardEvent &event, void* viewer_void)
{
    // handle key down events
    if(event.keyDown())
    {
        // handle any keys of interest
        switch(event.getKeyCode())
        {
            case 'a':
                cout << "KEYPRESS DETECTED: '" << event.getKeySym() << "'" << endl;
                break;
            case 'v':
                cout <<"Voxel Filter: " << endl;
            default:
                break;
        }
    }
}

/***********************************************************************************************************************
* @brief Opens a point cloud file
*
* Opens a point cloud file in either PCD or PLY format
*
* @param[out] cloudOut pointer to opened point cloud
* @param[in] filename path and name of input file
* @return false if an error occurred while opening file
* @author Christopher D. McMurrough
**********************************************************************************************************************/
bool openCloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloudOut, const char* fileName)
{
    // convert the file name to string
    std::string fileNameStr(fileName);

    // handle various file types
    std::string fileExtension = fileNameStr.substr(fileNameStr.find_last_of(".") + 1);
    if(fileExtension.compare("pcd") == 0)
    {
        // attempt to open the file
        if(pcl::io::loadPCDFile<pcl::PointXYZRGBA>(fileNameStr, *cloudOut) == -1)
        {
            PCL_ERROR("error while attempting to read pcd file: %s \n", fileNameStr.c_str());
            return false;
        }
        else
        {
            return true;
        }
    }
    else if(fileExtension.compare("ply") == 0)
    {
        // attempt to open the file
        if(pcl::io::loadPLYFile<pcl::PointXYZRGBA>(fileNameStr, *cloudOut) == -1)
        {
            PCL_ERROR("error while attempting to read pcl file: %s \n", fileNameStr.c_str());
            return false;
        }
        else
        {
            return true;
        }
    }
    else
    {
        PCL_ERROR("error while attempting to read unsupported file: %s \n", fileNameStr.c_str());
        return false;
    }
}

/***********************************************************************************************************************
* @brief program entry point
* @param[in] argc number of command line arguments
* @param[in] argv string array of command line arguments
* @returnS return code (0 for normal termination)
* @author Christoper D. McMurrough
**********************************************************************************************************************/
int loadPcd(int argc, char* file, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud)
{
    // validate and parse the command line arguments
    if(argc != NUM_COMMAND_ARGS + 1)
    {
        std::printf("USAGE: %s <file_name>\n", file);
        return 0;
    }

    // parse the command line arguments
    char* fileName = file;
    std::cout << fileName << std::endl;
    // create a stop watch for measuring time
    pcl::StopWatch watch;

    // initialize the cloud viewer
    CloudVisualizer CV("Rendering Window");

    // start timing the processing step
    watch.reset();

    // open the point cloud
    //pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
    //openCloud(cloud, fileName);

    // get the elapsed time
    double elapsedTime = watch.getTimeSeconds();
    cout << elapsedTime << " seconds passed " << std::endl;

    // render the scene
    CV.addCloud(cloud);
    CV.addCoordinateFrame(cloud->sensor_origin_, cloud->sensor_orientation_);

    // register mouse and keyboard event callbacks
    CV.registerPointPickingCallback(pointPickingCallback, cloud);
    CV.registerKeyboardCallback(keyboardCallback);

    // enter visualization loop
    while(CV.isRunning())
    {
        CV.spin(100);
    }

    // exit program
    return 0;
}
