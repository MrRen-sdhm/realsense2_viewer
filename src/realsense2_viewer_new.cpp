//
// Created by sdhm on 7/18/19.
// 读取sensor_msgs/PointCloud2以及sensor_msgs/Image, 显示和保存点云和彩色图像
// 获得的点云是无组织的
//

#include "realsense2_viewer_new.h"

using namespace std;

class Receiver {
public:
    enum Mode {
        IMAGE = 0,
        CLOUD,
        BOTH
    };

private:
    std::mutex lock;

    bool updateImage, updateCloud;
    bool save;
    bool running;
    size_t frame;
    const size_t queueSize;

    ros::NodeHandle nh;
    ros::AsyncSpinner spinner;
    ros::Subscriber pcl_sub;

    const std::string topicColor;
    const bool useCompressed;

    cv::Mat color, depth;

    std::thread imageViewerThread;
    Mode mode;

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud;
    pcl::PCDWriter writer;
    std::ostringstream oss;
    std::vector<int> params;

    typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::CameraInfo> ExactSyncPolicy;
    message_filters::Synchronizer<ExactSyncPolicy> *syncExact;

    image_transport::ImageTransport it;
    image_transport::SubscriberFilter *subImageColor;
    message_filters::Subscriber<sensor_msgs::CameraInfo> *subCameraInfoColor;

public:
    Receiver(string topicColor, const bool useCompressed)
            : topicColor(std::move(topicColor)), useCompressed(useCompressed),
              updateImage(false), updateCloud(false), save(false), running(false), frame(0), queueSize(5),
              nh("~"), spinner(0), it(nh), mode(CLOUD) {

        pcl_sub = nh.subscribe("/camera/depth/color/points", 1, &Receiver::cloud_callback, this);
        cloud = pcl::PointCloud<pcl::PointXYZRGBA>::Ptr(new pcl::PointCloud<pcl::PointXYZRGBA>());
    }

    void run(const Mode mode) {
        start(mode);
        stop();
    }

private:
    void start(const Mode mode) {
        this->mode = mode;
        running = true;

        std::string topicCameraInfoColor = topicColor.substr(0, topicColor.rfind('/')) + "/camera_info";

        image_transport::TransportHints hints(useCompressed ? "compressed" : "raw");
        subImageColor = new image_transport::SubscriberFilter(it, topicColor, queueSize, hints);
        subCameraInfoColor = new message_filters::Subscriber<sensor_msgs::CameraInfo>(nh, topicCameraInfoColor,
                                                                                      queueSize);

        syncExact = new message_filters::Synchronizer<ExactSyncPolicy>(ExactSyncPolicy(queueSize), *subImageColor,
                                                                       *subCameraInfoColor);
        syncExact->registerCallback(boost::bind(&Receiver::image_callback, this, _1, _2));


        spinner.start();

        std::chrono::milliseconds duration(1);
        while (!updateImage || !updateCloud) {
            if (!ros::ok()) {
                return;
            }
            std::this_thread::sleep_for(duration);
        }

        switch (mode) {
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

        delete syncExact;
        delete subImageColor;
        delete subCameraInfoColor;

        running = false;
        if(mode == BOTH)
        {
            imageViewerThread.join();
        }
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
                        saveCloudAndImages();
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
        lock.lock();
        updateCloud = false;
        lock.unlock();

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

        for(; running && ros::ok() && !viewer->wasStopped();)
        {
            if(updateCloud)
            {
                lock.lock();
                updateCloud = false;
                lock.unlock();

                viewer->updatePointCloud<pcl::PointXYZRGBA>(cloud, rgb, cloudName);
            }
            if(save)
            {
                save = false;
                saveCloudAndImages();
            }
            viewer->spinOnce(0);
        }
        viewer->close();
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
        printf("Saving image: %s\n", colorName.c_str());
        cv::imwrite(colorName, color, params);
        if (cloud->isOrganized()) {
            printf("Cloud is Organized!\n");
        } else {
            printf("Cloud is not Organized!\n");
        }
        printf("Saving complete!\n");
        ++frame;
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

    void readImage(const sensor_msgs::Image::ConstPtr& msgImage, cv::Mat &image) const
    {
        cv_bridge::CvImageConstPtr pCvImage;
        pCvImage = cv_bridge::toCvShare(msgImage, msgImage->encoding);
        cv::cvtColor(pCvImage->image, image, cv::COLOR_BGR2RGB);
    }

    void image_callback(const sensor_msgs::Image::ConstPtr& imageColor,
                                const sensor_msgs::CameraInfo::ConstPtr& cameraInfoColor)
    {
        cv::Mat color, depth;
        readImage(imageColor, color);

        lock.lock();
        this->color = color;
        this->depth = depth;
        updateImage = true;
        updateCloud = true;
        lock.unlock();
    }

    void cloud_callback(const sensor_msgs::PointCloud2& input)
    {
        pcl::fromROSMsg(input, *cloud);
        lock.lock();
        updateImage = true;
        updateCloud = true;
        lock.unlock();
    }

};


int main (int argc, char** argv)
{
    ros::init(argc, argv, "realsense2_viewer_new", ros::init_options::AnonymousName);

    bool useCompressed = false;
    Receiver::Mode mode = Receiver::BOTH;

    for(size_t i = 1; i < (size_t)argc; ++i)
    {
        std::string param(argv[i]);

        if(param == "-h" || param == "--help" || param == "-?" || param == "--?")
        {
//            help(argv[0]);
            ros::shutdown();
            return 0;
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
    }

    string topicColor = "/camera/color/image_raw";

    Receiver receiver(topicColor, useCompressed);

    ROS_INFO("starting receiver...");
    receiver.run(mode);

    return 0;
}


