//
// Created by sdhm on 7/18/19.
// 读取sensor_msgs/PointCloud2, 显示和保存点云
// 获得的点云是无组织的
//

#include <iostream>
#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <sensor_msgs/PointCloud2.h>

using namespace std;

class Receiver {
private:
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud;
    ros::Subscriber pcl_sub;
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner;

    bool updateCloud = false;
    bool running = false;
    bool save = false;
    size_t frame = 0;

    pcl::PCDWriter writer;
    std::ostringstream oss;

    void cloud_callback(const sensor_msgs::PointCloud2 &input) {
        pcl::fromROSMsg(input, *cloud);

        updateCloud = true;
    }

    void keyboardEvent(const pcl::visualization::KeyboardEvent &event, void *viewer_void)
    {
        auto *viewer = static_cast<pcl::visualization::PCLVisualizer *>(viewer_void);

        if(event.keyDown())
        {
            switch(event.getKeyCode())
            {
                case 27:
                case 'q':
                    running = false;
                    break;
                case ' ':
                case 's':
                    save = true;
                    break;
                case 'a':
                    if (viewer->contains("ref")) {
                        viewer->removeCoordinateSystem("ref");
                        printf("\033[0;36m%s\033[0m\n", "[Keyboard Event] Remove coordinate system.");
                    } else {
                        viewer->addCoordinateSystem(0.1, "ref");
                        printf("\033[0;36m%s\033[0m\n", "[Keyboard Event] Add coordinate system.");
                    }
                    break;
            }
        }
    }

    static string getCurrentTimeStr()
    {
        time_t t = time(nullptr);
        char ch[64] = {0};
//        strftime(ch, sizeof(ch)-1, "%Y-%m-%d %H-%M-%S", localtime(&t));
        strftime(ch, sizeof(ch)-1, "%m%d", localtime(&t));     //年-月-日 时-分-秒
        return ch;
    }

    void saveCloudAndImages()
    {
        string baseName, cloudName, colorName;

        while (true) {
            oss.str("");
            oss << std::setfill('0') << std::setw(2) << frame;
            baseName = oss.str();
            cloudName = "./" + baseName + "_cloud_" + getCurrentTimeStr() + ".pcd";
            colorName = "./" + baseName + "_color_" + getCurrentTimeStr() + ".jpg";

            if ((access(cloudName.c_str(), 0)) == 0) { // 0已存在,-1不存在
                frame++;
            }
            else {
                break;
            }
        }

        printf("Saving cloud: %s\n", cloudName.c_str());
        writer.writeBinary(cloudName, *cloud);
        if (cloud->isOrganized()) {
            printf("Cloud is Organized!\n");
        } else {
            printf("Cloud is not Organized!\n");
        }
        printf("Saving complete!\n");
        ++frame;
    }

public:
    Receiver() : spinner(0){
        cloud = pcl::PointCloud<pcl::PointXYZRGBA>::Ptr(new pcl::PointCloud<pcl::PointXYZRGBA>());
        pcl_sub = nh.subscribe("/camera/depth/color/points", 1, &Receiver::cloud_callback, this);
    };

    void run(){
        spinner.start();
        updateCloud = false;

        pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("Cloud Viewer"));
        const std::string cloudName = "cloud";
        pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBA> rgb(cloud); // 非有序点云, 须着色器才能显示颜色
        viewer->addPointCloud<pcl::PointXYZRGBA>(cloud, rgb, cloudName);
//        viewer->addCoordinateSystem(0.1, "ref");
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, cloudName);
        viewer->initCameraParameters();
        viewer->setBackgroundColor(0, 0, 0);
        viewer->setPosition(0, 0);
        viewer->setSize(640, 480);
        viewer->setShowFPS(true);
        viewer->setCameraPosition(0, 0, 0, 0, -1, 0);
        viewer->registerKeyboardCallback(&Receiver::keyboardEvent, *this, (void *)viewer.get());

        while (ros::ok() && ! viewer->wasStopped())
        {
            if(updateCloud) {
                viewer->updatePointCloud<pcl::PointXYZRGBA>(cloud, rgb, cloudName);
                updateCloud = false;
            }
            if (save) {
                save = false;
                saveCloudAndImages();
            }
            viewer->spinOnce(0);
        }
        viewer->close();
    };
};

int main (int argc, char** argv)
{
    ros::init(argc, argv, "pointcoud2");
    Receiver receiver;
    receiver.run();
    return 0;
}
