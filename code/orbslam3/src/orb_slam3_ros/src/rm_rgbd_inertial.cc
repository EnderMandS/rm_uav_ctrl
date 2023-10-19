/**
* 
* Adapted from ORB-SLAM3: Examples/ROS/src/ros_mono_inertial.cc and ros_rgbd.cc
*
*/

#include "common.h"

using namespace std;

class ImuGrabber
{
public:
    ImuGrabber(){};
    void GrabImu(const sensor_msgs::ImuConstPtr &imu_msg);

    queue<sensor_msgs::ImuConstPtr> imuBuf;
    std::mutex mBufMutex;
};

class ImageGrabber
{
public:
    ImageGrabber(ImuGrabber *pImuGb): mpImuGb(pImuGb){}

    void GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB, const sensor_msgs::ImageConstPtr& msgD);
    cv::Mat GetImage(const sensor_msgs::ImageConstPtr &img_msg);
    void SyncWithImu();

    queue<sensor_msgs::ImageConstPtr> imgRGBBuf, imgDBuf;
    std::mutex mBufMutex;
    ImuGrabber *mpImuGb;
};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "RGBD_Inertial");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
    if (argc > 1)
    {
        ROS_WARN ("Arguments supplied via command line are ignored.");
    }
    ROS_INFO("ORB RGBD Inertial start");
    std::string node_name = ros::this_node::getName();

    ros::NodeHandle node_handler;
    image_transport::ImageTransport image_transport(node_handler);

    std::string voc_file, settings_file;
    bool enable_pangolin = false;
    node_handler.param<std::string>(node_name + "/voc_file", voc_file, "file_not_set");
    node_handler.param<std::string>(node_name + "/settings_file", settings_file, "file_not_set");
    node_handler.param<bool>(node_name + "/enable_pangolin", enable_pangolin, false);
    node_handler.param<std::string>(node_name + "/world_frame_id", world_frame_id, "map");
    node_handler.param<std::string>(node_name + "/cam_frame_id", cam_frame_id, "camera");
    node_handler.param<std::string>(node_name + "/imu_frame_id", imu_frame_id, "imu");

    if (voc_file == "file_not_set" || settings_file == "file_not_set")
    {
        ROS_ERROR("Please provide voc_file and settings_file in the launch file");       
        ros::shutdown();
        return -1;
    }
    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    sensor_type = ORB_SLAM3::System::IMU_RGBD;
    pSLAM = new ORB_SLAM3::System(voc_file, settings_file, sensor_type, enable_pangolin);
    ROS_INFO("Create ORB SLAM3 System success.");

    ImuGrabber imugb;
    ImageGrabber igb(&imugb);

    ros::Subscriber sub_imu = node_handler.subscribe("/imu", 1000, &ImuGrabber::GrabImu, &imugb);

    message_filters::Subscriber<sensor_msgs::Image> sub_rgb_img(node_handler, "/camera/rgb/image_raw", 100);
    message_filters::Subscriber<sensor_msgs::Image> sub_depth_img(node_handler, "/camera/depth_registered/image_raw", 100);

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), sub_rgb_img, sub_depth_img);
    sync.registerCallback(boost::bind(&ImageGrabber::GrabRGBD, &igb, _1, _2));

    setup_publishers(node_handler, image_transport, node_name);
    setup_services(node_handler, node_name);

    ROS_INFO("Create ros component success.");

    std::thread sync_thread(&ImageGrabber::SyncWithImu, &igb);

    ros::spin();

    // Stop all threads
    pSLAM->Shutdown();
    ros::shutdown();

    return 0;
}

//////////////////////////////////////////////////
// Functions
//////////////////////////////////////////////////

void ImageGrabber::GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD)
{
    if (this->mBufMutex.try_lock()) {
        if (!imgRGBBuf.empty())
            imgRGBBuf.pop();
        imgRGBBuf.push(msgRGB);

        if (!imgDBuf.empty())
            imgDBuf.pop();
        imgDBuf.push(msgD);

        this->mBufMutex.unlock();
    }
}

cv::Mat ImageGrabber::GetImage(const sensor_msgs::ImageConstPtr &img_msg)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(img_msg);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }

    return cv_ptr->image.clone();
}

void ImageGrabber::SyncWithImu() {
    cv::Mat im, depth;
    double tIm;
    ros::Time msg_time;
    vector<ORB_SLAM3::IMU::Point> vImuMeas;
    Eigen::Vector3f Wbb;

    while(1) {
        if (this->mBufMutex.try_lock()) {
            if (mpImuGb->mBufMutex.try_lock()) {
                if (!imgRGBBuf.empty() && mpImuGb->imuBuf.size()>1) {
                    tIm = imgRGBBuf.front()->header.stamp.toSec();
                    if (tIm < mpImuGb->imuBuf.back()->header.stamp.toSec() && 
                        tIm >= mpImuGb->imuBuf.front()->header.stamp.toSec()) {
                        msg_time = imgRGBBuf.front()->header.stamp;
                        im = GetImage(imgRGBBuf.front());
                        depth = GetImage(imgDBuf.front());
                        imgRGBBuf.pop();
                        imgDBuf.pop();
                        this->mBufMutex.unlock();

                        // Load imu measurements from buffer
                        vImuMeas.clear();
                        #ifdef MY_DEBUG
                            int imuCnt=0;
                        #endif
                        while(mpImuGb->imuBuf.front()->header.stamp.toSec() <= tIm) {
                            cv::Point3f acc(
                                mpImuGb->imuBuf.front()->linear_acceleration.x, 
                                mpImuGb->imuBuf.front()->linear_acceleration.y, 
                                mpImuGb->imuBuf.front()->linear_acceleration.z);
                            cv::Point3f gyr(
                                mpImuGb->imuBuf.front()->angular_velocity.x, 
                                mpImuGb->imuBuf.front()->angular_velocity.y, 
                                mpImuGb->imuBuf.front()->angular_velocity.z);
                            double t = mpImuGb->imuBuf.front()->header.stamp.toSec();
                            vImuMeas.push_back(ORB_SLAM3::IMU::Point(acc, gyr, t));
                            Wbb <<  mpImuGb->imuBuf.front()->angular_velocity.x,
                                    mpImuGb->imuBuf.front()->angular_velocity.y,
                                    mpImuGb->imuBuf.front()->angular_velocity.z;
                            mpImuGb->imuBuf.pop();
                            #ifdef MY_DEBUG
                                ++imuCnt;
                            #endif
                        }
                        mpImuGb->mBufMutex.unlock();

                        // ORB-SLAM3 runs in TrackRGBD()
                        #ifdef MY_DEBUG
                            ROS_INFO("Track RGBD start whit imuCnt: %d", imuCnt);
                        #endif
                        pSLAM->TrackRGBD(im, depth, tIm, vImuMeas);
                        #ifdef MY_DEBUG
                            ROS_INFO("Track RGBD end");
                        #endif

                        publish_topics(msg_time, Wbb);
                        continue;   // jump unlock. time gone when tacking, no need for thread sleep
                    }   // if(tIm < mpImuGb->imuBuf.back()->header.stamp.toSec())
                }   // if(!imgRGBBuf.empty() && !mpImuGb->imuBuf.empty())
                mpImuGb->mBufMutex.unlock();
            }   // mpImuGb->mBufMutex.try_lock()
            this->mBufMutex.unlock();
        }   // this->mBufMutex.try_lock()
        std::chrono::milliseconds tSleep(1);
        std::this_thread::sleep_for(tSleep);
    } // while(1)
} // void ImageGrabber::SyncWithImu

void ImuGrabber::GrabImu(const sensor_msgs::ImuConstPtr &imu_msg)
{
    if (mBufMutex.try_lock()) {
        imuBuf.push(imu_msg);
        mBufMutex.unlock();
    }
}