//
// Created by sdhm on 7/18/19.
// 读取sensor_msgs/Image以及sensor_msgs/CameraInfo, 通过深度图和相机参数创建点云, 显示和保存和彩色图像
// 获得的点云为有组织的！
//

#include <utility>
#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <cmath>
#include <mutex>
#include <thread>
#include <chrono>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>

#include <opencv2/opencv.hpp>

#include <ros/ros.h>
#include <ros/spinner.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>

#include <cv_bridge/cv_bridge.h>

#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>

#include "realsense2_viewer.h"

class Receiver
{
public:
    enum Mode
    {
        IMAGE = 0,
        CLOUD,
        BOTH
    };

private:
    std::mutex lock;

    const std::string topicColor, topicDepth;
    const bool useExact, useCompressed;

    bool updateImage, updateCloud;
    bool save;
    bool running;
    size_t frame;
    const size_t queueSize;

    cv::Mat color, depth;
    cv::Mat cameraMatrixColor, cameraMatrixDepth;
    cv::Mat lookupX, lookupY;

    typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::CameraInfo> ExactSyncPolicy;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::CameraInfo> ApproximateSyncPolicy;

    ros::NodeHandle nh;
    ros::AsyncSpinner spinner;
    image_transport::ImageTransport it;
    image_transport::SubscriberFilter *subImageColor, *subImageDepth;
    message_filters::Subscriber<sensor_msgs::CameraInfo> *subCameraInfoColor, *subCameraInfoDepth;

    message_filters::Synchronizer<ExactSyncPolicy> *syncExact;
    message_filters::Synchronizer<ApproximateSyncPolicy> *syncApproximate;

    std::thread imageViewerThread;
    Mode mode;

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud;
    pcl::PCDWriter writer;
    std::ostringstream oss;
    std::vector<int> params;

public:
    Receiver(std::string topicColor, const std::string &topicDepth, const bool useExact, const bool useCompressed)
            : topicColor(std::move(topicColor)), topicDepth(topicDepth), useExact(useExact), useCompressed(useCompressed),
              updateImage(false), updateCloud(false), save(false), running(false), frame(0), queueSize(5),
              nh("~"), spinner(0), it(nh), mode(CLOUD)
    {
        cameraMatrixColor = cv::Mat::zeros(3, 3, CV_64F);
        cameraMatrixDepth = cv::Mat::zeros(3, 3, CV_64F);
        params.push_back(cv::IMWRITE_JPEG_QUALITY);
        params.push_back(100);
        params.push_back(cv::IMWRITE_PNG_COMPRESSION);
        params.push_back(1);
        params.push_back(cv::IMWRITE_PNG_STRATEGY);
        params.push_back(cv::IMWRITE_PNG_STRATEGY_RLE);
        params.push_back(0);
    }

    void run(const Mode mode)
    {
        start(mode);
        stop();
    }

private:
    void start(const Mode mode)
    {
        this->mode = mode;
        running = true;

        std::string topicCameraInfoColor = topicColor.substr(0, topicColor.rfind('/')) + "/camera_info";
        std::string topicCameraInfoDepth = topicDepth.substr(0, topicDepth.rfind('/')) + "/camera_info";

        image_transport::TransportHints hints(useCompressed ? "compressed" : "raw");
        subImageColor = new image_transport::SubscriberFilter(it, topicColor, queueSize, hints);
        subImageDepth = new image_transport::SubscriberFilter(it, topicDepth, queueSize, hints);
        subCameraInfoColor = new message_filters::Subscriber<sensor_msgs::CameraInfo>(nh, topicCameraInfoColor, queueSize);
        subCameraInfoDepth = new message_filters::Subscriber<sensor_msgs::CameraInfo>(nh, topicCameraInfoDepth, queueSize);

        if(useExact)
        {
            syncExact = new message_filters::Synchronizer<ExactSyncPolicy>(ExactSyncPolicy(queueSize), *subImageColor, *subImageDepth, *subCameraInfoColor, *subCameraInfoDepth);
            syncExact->registerCallback(boost::bind(&Receiver::callback, this, _1, _2, _3, _4));
        }
        else
        {
            syncApproximate = new message_filters::Synchronizer<ApproximateSyncPolicy>(ApproximateSyncPolicy(queueSize), *subImageColor, *subImageDepth, *subCameraInfoColor, *subCameraInfoDepth);
            syncApproximate->registerCallback(boost::bind(&Receiver::callback, this, _1, _2, _3, _4));
        }

        spinner.start();

        std::chrono::milliseconds duration(1);
        while(!updateImage || !updateCloud)
        {
            if(!ros::ok())
            {
                return;
            }
            std::this_thread::sleep_for(duration);
        }
        cloud = pcl::PointCloud<pcl::PointXYZRGBA>::Ptr(new pcl::PointCloud<pcl::PointXYZRGBA>());
        cloud->height = color.rows;
        cloud->width = color.cols;
        cloud->is_dense = false;
        cloud->points.resize(cloud->height * cloud->width);
        createLookup(this->color.cols, this->color.rows);

        switch(mode)
        {
            case CLOUD:
                cloudViewer();
                break;
            case IMAGE:
                imageViewer();
                break;
            case BOTH:
                imageViewerThread = std::thread(&Receiver::imageViewer, this);
                cloudViewer();
                break;
        }
    }

    void stop()
    {
        spinner.stop();

        if(useExact)
        {
            delete syncExact;
        }
        else
        {
            delete syncApproximate;
        }

        delete subImageColor;
        delete subImageDepth;
        delete subCameraInfoColor;
        delete subCameraInfoDepth;

        running = false;
        if(mode == BOTH)
        {
            imageViewerThread.join();
        }
    }

    void callback(const sensor_msgs::Image::ConstPtr& imageColor, const sensor_msgs::Image::ConstPtr imageDepth,
                  const sensor_msgs::CameraInfo::ConstPtr& cameraInfoColor, const sensor_msgs::CameraInfo::ConstPtr cameraInfoDepth)
    {
        cv::Mat color, depth;

        readCameraInfo(cameraInfoColor, cameraMatrixColor);
        readCameraInfo(cameraInfoDepth, cameraMatrixDepth);
        readRgbImage(imageColor, color);
        readDepthImage(imageDepth, depth);

        lock.lock();
        this->color = color;
        this->depth = depth;
        updateImage = true;
        updateCloud = true;
        lock.unlock();
    }

    void imageViewer()
    {
        cv::Mat color, color_show;
        std::chrono::time_point<std::chrono::high_resolution_clock> start, now;
        double fps = 0;
        size_t frameCount = 0;
        std::ostringstream oss;
        const cv::Point pos(5, 15);
        const cv::Scalar colorText = CV_RGB(255, 255, 255);
        const double sizeText = 0.5;
        const int lineText = 1;
        const int font = cv::FONT_HERSHEY_SIMPLEX;

        cv::namedWindow("Image Viewer");
        oss << "starting...";

        start = std::chrono::high_resolution_clock::now();
        for(; running && ros::ok();)
        {
            if(updateImage)
            {
                lock.lock();
                color = this->color;
                this->color.copyTo(color_show);
                updateImage = false;
                lock.unlock();

                ++frameCount;
                now = std::chrono::high_resolution_clock::now();
                double elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - start).count() / 1000.0;
                if(elapsed >= 1.0)
                {
                    fps = frameCount / elapsed;
                    oss.str("");
                    oss << "fps: " << fps << " ( " << elapsed / frameCount * 1000.0 << " ms)";
                    start = now;
                    frameCount = 0;
                }

                cv::putText(color_show, oss.str(), pos, font, sizeText, colorText, lineText, CV_AA);
                cv::imshow("Image Viewer", color_show);
            }

            int key = cv::waitKey(1);
            switch(key & 0xFF)
            {
                case 27:
                case 'q':
                    running = false;
                    break;
                case ' ':
                case 's':
                    if(mode == IMAGE)
                    {
                        createCloud(depth, color, cloud);
                        saveCloudAndImages(cloud, color, depth);
                    }
                    else
                    {
                        save = true;
                    }
                    break;
            }
        }
        cv::destroyAllWindows();
        cv::waitKey(100);
    }

    void cloudViewer()
    {
        cv::Mat color, depth;
        pcl::visualization::PCLVisualizer::Ptr visualizer(new pcl::visualization::PCLVisualizer("Cloud Viewer"));
        const std::string cloudName = "rendered";

        lock.lock();
        color = this->color;
        depth = this->depth;
        updateCloud = false;
        lock.unlock();

        createCloud(depth, color, cloud);

        visualizer->addPointCloud(cloud, cloudName);
//        visualizer->addCoordinateSystem(0.1);
        visualizer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, cloudName);
        visualizer->initCameraParameters();
        visualizer->setBackgroundColor(0, 0, 0);
        visualizer->setPosition(mode == BOTH ? color.cols : 0, 0);
        visualizer->setSize(color.cols, color.rows);
        visualizer->setShowFPS(true);
        visualizer->setCameraPosition(0, 0, 0, 0, -1, 0);
        visualizer->registerKeyboardCallback(&Receiver::keyboardEvent, *this, (void *)visualizer.get());

        for(; running && ros::ok() && !visualizer->wasStopped();)
        {
            if(updateCloud)
            {
                lock.lock();
                color = this->color;
                depth = this->depth;
                updateCloud = false;
                lock.unlock();

                createCloud(depth, color, cloud);

                visualizer->updatePointCloud(cloud, cloudName);
            }
            if(save)
            {
                save = false;
                cv::Mat depthDisp;
                saveCloudAndImages(cloud, color, depth);
            }
            visualizer->spinOnce(10);
        }
        visualizer->close();
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

    void readRgbImage(const sensor_msgs::Image::ConstPtr& msgImage, cv::Mat &image) const
    {
        cv_bridge::CvImageConstPtr pCvImage;
        pCvImage = cv_bridge::toCvShare(msgImage, msgImage->encoding);
        cv::cvtColor(pCvImage->image, image, cv::COLOR_BGR2RGB);
    }

    void readDepthImage(const sensor_msgs::Image::ConstPtr& msgImage, cv::Mat &image) const
    {
        cv_bridge::CvImageConstPtr pCvImage;
        pCvImage = cv_bridge::toCvShare(msgImage, msgImage->encoding);
        pCvImage->image.copyTo(image);
    }

    void readCameraInfo(const sensor_msgs::CameraInfo::ConstPtr cameraInfo, cv::Mat &cameraMatrix) const
    {
        double *itC = cameraMatrix.ptr<double>(0, 0);
        for(size_t i = 0; i < 9; ++i, ++itC)
        {
            *itC = cameraInfo->K[i];
        }
    }

    void createCloud(const cv::Mat &depth, const cv::Mat &color, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud) const {
        const float badPoint = std::numeric_limits<float>::quiet_NaN();

#pragma omp parallel for
        for (int r = 0; r < depth.rows; ++r) {
            pcl::PointXYZRGBA *itP = &cloud->points[r * depth.cols];
            const uint16_t *itD = depth.ptr<uint16_t>(r);
            const cv::Vec3b *itC = color.ptr<cv::Vec3b>(r);
            const float y = lookupY.at<float>(0, r);
            const float *itX = lookupX.ptr<float>();

            for (size_t c = 0; c < (size_t) depth.cols; ++c, ++itP, ++itD, ++itC, ++itX) {
                register const float depthValue = *itD / 1000.0f;
                // Check for invalid measurements
                if (*itD == 0) {
                    // not valid
                    itP->x = itP->y = itP->z = badPoint;
                    itP->rgba = 0;
                    continue;
                }
                itP->z = depthValue;
                itP->x = *itX * depthValue;
                itP->y = y * depthValue;
                itP->b = itC->val[0];
                itP->g = itC->val[1];
                itP->r = itC->val[2];
                itP->a = 255;
            }
        }
    }

    static std::string getCurrentTimeStr()
    {
        time_t t = time(nullptr);
        char ch[64] = {0};
//        strftime(ch, sizeof(ch)-1, "%Y-%m-%d %H-%M-%S", localtime(&t));
        strftime(ch, sizeof(ch)-1, "%y%m%d", localtime(&t));     //年-月-日 时-分-秒
        return ch;
    }

    void saveCloudAndImages(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr cloud, const cv::Mat &color, const cv::Mat &depth)
    {
        std::string baseName, cloudName, colorName, depthName;

        while (true) {
            oss.str("");
            oss << std::setfill('0') << std::setw(2) << frame;
            baseName = oss.str();
            cloudName = "./" + getCurrentTimeStr() + "_" + baseName + "_cloud" +  + ".pcd";
            colorName = "./" + getCurrentTimeStr() + "_" + baseName + "_color" +  + ".jpg";
            depthName = "./" + getCurrentTimeStr() + "_" + baseName + "_depth" +  + ".png";

            if ((access(cloudName.c_str(), 0)) == 0) { // 0已存在,-1不存在
                frame++;
            }
            else {
                break;
            }
        }

        printf("%s\n", ("[INFO] Saving cloud: " + cloudName).c_str());
        writer.writeBinary(cloudName, *cloud);
        printf("%s\n", ("[INFO] Saving color: " + colorName).c_str());
        cv::imwrite(colorName, color, params);
        printf("%s\n", ("[INFO] Saving depth: " + depthName).c_str());
        cv::imwrite(depthName, depth, params);

        if (cloud->isOrganized()) {
            printf("[INFO] Cloud is Organized!\n");
        } else {
            printf("[INFO] Cloud is not Organized!\n");
        }
        printf("[INFO] Saving complete!\n");

        ++frame;
    }

    void createLookup(size_t width, size_t height)
    {
        const float fx = 1.0f / cameraMatrixColor.at<double>(0, 0);
        const float fy = 1.0f / cameraMatrixColor.at<double>(1, 1);
        const float cx = cameraMatrixColor.at<double>(0, 2);
        const float cy = cameraMatrixColor.at<double>(1, 2);
        float *it;

        lookupY = cv::Mat(1, height, CV_32F);
        it = lookupY.ptr<float>();
        for(size_t r = 0; r < height; ++r, ++it)
        {
            *it = (r - cy) * fy;
        }

        lookupX = cv::Mat(1, width, CV_32F);
        it = lookupX.ptr<float>();
        for(size_t c = 0; c < width; ++c, ++it)
        {
            *it = (c - cx) * fx;
        }
    }
};

void help(const std::string &path)
{
    std::cout << path << FG_BLUE " [options]" << std::endl
    << FG_GREEN "  name" NO_COLOR ": " FG_YELLOW "'any string'" NO_COLOR
        " equals to the realsense2 topic base name" << std::endl
    << FG_YELLOW "    'compressed'" NO_COLOR " use compressed instead of raw topics" << std::endl
    << FG_YELLOW "    'approx'" NO_COLOR " use approximate time synchronization" << std::endl;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "relasens2_viewer", ros::init_options::AnonymousName);

    if(!ros::ok())
    {
        return 0;
    }

    std::string ns = R2_DEFAULT_NS;
    std::string topicColor = R2_TOPIC_IMAGE_COLOR R2_TOPIC_IMAGE_RAW;
    std::string topicDepth = R2_TOPIC_ALIGNED_DEPTH R2_TOPIC_IMAGE_RAW;
    bool useExact = false; // D435i could'n useExact now!
    bool useCompressed = false;
    Receiver::Mode mode = Receiver::BOTH;

    for(size_t i = 1; i < (size_t)argc; ++i)
    {
        std::string param(argv[i]);

        if(param == "-h" || param == "--help" || param == "-?" || param == "--?")
        {
            help(argv[0]);
            ros::shutdown();
            return 0;
        }
        else if(param == "approx")
        {
            useExact = false;
        }
        else if(param == "compressed")
        {
            useCompressed = true;
        }
        else if(param == "image")
        {
            mode = Receiver::IMAGE;
        }
        else if(param == "cloud")
        {
            mode = Receiver::CLOUD;
        }
        else if(param == "both")
        {
            mode = Receiver::BOTH;
        }
        else
        {
            ns = param;
        }
    }

    topicColor = "/" + ns + topicColor;
    topicDepth = "/" + ns + topicDepth;
    OUT_INFO("topic color: " FG_CYAN << topicColor << NO_COLOR);
    OUT_INFO("topic depth: " FG_CYAN << topicDepth << NO_COLOR);

    Receiver receiver(topicColor, topicDepth, useExact, useCompressed);

    OUT_INFO("starting receiver...");
    receiver.run(mode);

    ros::shutdown();
    return 0;
}
