#include <iostream>
#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/Header.h>
#include <utils/Lane.h>

using namespace std;
using namespace cv;

class LaneDetector
{
private:
    ros::NodeHandle nh;
    ros::Publisher pub;
    ros::Subscriber image_sub;
    cv::Mat maskh;
    cv::Mat masks;
    cv::Mat image;
    bool stopline;
    cv::Ptr<cv_bridge::CvImage> cv_ptr;
    utils::Lane p;

public:
    LaneDetector() : nh("~")
    {
        // Initialize the masks for histogram filter
        cout << "Lane detection using histogram filter" << endl;
        maskh = cv::Mat::zeros(480, 640, CV_8UC1);
        int h = static_cast<int>(0.8 * 480);
        cv::Point polyh[] = {cv::Point(0, h), cv::Point(640, h), cv::Point(640, 480), cv::Point(0, 480)};
        cv::fillPoly(maskh, &polyh, const_cast<const cv::Point**>(&polyh + 1), 1, cv::Scalar(255, 255, 255), cv::LINE_AA);

        masks = cv::Mat::zeros(480, 640, CV_8UC1);
        cv::Point polys[] = {cv::Point(0, 300), cv::Point(640, 300), cv::Point(640, 340), cv::Point(0, 340)};
        cv::fillPoly(masks, &polys, const_cast<const cv::Point**>(&polys + 1), 1, cv::Scalar(255, 255, 255), cv::LINE_AA);

        image = cv::Mat::zeros(480, 640, CV_8UC1);
        stopline = false;

        // Initialize the lane follower node
        pub = nh.advertise<utils::Lane>("lane", 3);
        cv_ptr = boost::make_shared<cv_bridge::CvImage>();
        image_sub = nh.subscribe("automobile/image_raw", 1, &LaneDetector::imageCallback, this);
    }

    void imageCallback(const sensor_msgs::ImageConstPtr& msg)
    {
        // Update the header information
        std_msgs::Header header = msg->header;
        p.header = header;

        // Convert the image to the OpenCV format
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
        image = cv_ptr->image;

        // Extract the lanes from the image
        double lanes = histogram(image);

        p.center = lanes;

        // Determine whether we arrive at intersection
        p.stopline = stopline;

        // Publish the lane message
        pub.publish(p);
    }

    double LaneDetector::histogram(cv::Mat image) {
        // Initialize stopline flag
        this->stopline = false;

        // Convert image to grayscale
        cv::Mat img_gray;
        cv::cvtColor(image, img_gray, cv::COLOR_BGR2GRAY);

        // Set image height and width
        int h = 480;
        int w = 640;

        // Find stopline
        cv::Mat img_rois;
        cv::bitwise_and(img_gray, this->masks, img_rois);
        int t = cv::max(img_rois) - 65;
        if (t < 30) {
            t = 30;
        }
        cv::clip(t, 30, 200);
        cv::Mat threshs;
        cv::threshold(img_rois, threshs, t, 255, cv::THRESH_BINARY);
        cv::Mat hists(1, w, CV_32F, cv::Scalar(0));
        for (int i = 0; i < w; i++) {
            hists.at<float>(0, i) = cv::sum(threshs.col(i))[0];
        }
        std::vector<int> lanes;
        int p = 0;
        for (int i = 0; i < w; i++) {
            if (hists.at<float>(0, i) >= 1500 && p == 0) {
                lanes.push_back(i);
                p = 255;
            }
            else if (hists.at<float>(0, i) == 0 && p == 255) {
                lanes.push_back(i);
                p = 0;
            }
        }
        if (lanes.size() % 2 == 1) {
            lanes.push_back(w - 1);
        }
        for (int i = 0; i < lanes.size() / 2; i++) {
            if (abs(lanes[2 * i] - lanes[2 * i + 1]) > 370 && t > 30) {
                this->stopline = true;
            }
        }
        cv::Mat img_gray;
        cv::cvtColor(image, img_gray, cv::COLOR_BGR2GRAY);
        int h = 480;
        int w = 640;
        cv::Mat img_roi = img_gray & maskh;
        int t = cv::max(img_roi) - 55;
        if (t < 30)
            t = 30;
        cv::clip(t, 30, 200);
        cv::Mat thresh;
        cv::threshold(img_roi, thresh, t, 255, cv::THRESH_BINARY);
        cv::Mat hist = cv::Mat::zeros(1, w, CV_32F);
        for (int i = 0; i < w; i++) {
            hist.at<float>(0, i) = cv::sum(thresh.col(i))[0];
        }

        // get lane marking delimiters
        std::vector<int> lanes;
        int p = 0;
        for (int i = 0; i < w; i++) {
            if (hist.at<float>(0, i) >= 1500 && p == 0) {
                lanes.push_back(i);
                p = 255;
            }
            else if (hist.at<float>(0, i) == 0 && p == 255) {
                lanes.push_back(i);
                p = 0;
            }
        }
        if (lanes.size() % 2 == 1) {
            lanes.push_back(w - 1);
        }

        // get lane markings
        std::vector<int> centers;
        for (int i = 0; i < lanes.size() / 2; i++) {
            if (abs(lanes[2 * i] - lanes[2 * i + 1]) > 350 && t > 50) {
                stopline = true;
            }
            else if (abs(lanes[2 * i] - lanes[2 * i + 1]) > 3) { // and abs(lanes[2*i]-lanes[2*i+1])<100: //exclude large lanes
                centers.push_back((lanes[2 * i] + lanes[2 * i + 1]) / 2);
            }
        }

        // get lane centers based on 4 cases
        if (centers.size() == 0) { // no lane detected
            return w / 2;
        }
        else if (centers.size() == 1) { // one lane detected
            if (centers[0] > w / 2) {
                return (centers[0] - 0) / 2;
            }
            else {
                return (centers[0] * 2 + 640) / 2;
            }
        }
        else if (abs(centers[0] - centers[centers.size() - 1]) < 200) { // the left most lane and the right most lane are close together (fuse them)
            if ((centers[0] + centers[centers.size() - 1]) > w) {
                return ((centers[0] + centers[centers.size() - 1]) / 2 + 0) / 2;
            }
            else {
                return ((centers[0] + centers[centers.size() - 1]) + 640) / 2;
            }
        }
        else { // the left most lane and the right most lane are far (avg them)
            return (centers[0] + centers[centers.size()-1])/2;
        }
    }

}