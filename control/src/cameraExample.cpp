#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <chrono>
using namespace std::chrono;

class LaneDetector {
public:
    LaneDetector() : it(nh) {
        image_sub = it.subscribe("/automobile/image_raw", 1, &LaneDetector::imageCallback, this);
        image_pub = it.advertise("/automobile/image_modified", 1);

        maskh = cv::Mat::zeros(480, 640, CV_8UC1);
        int h = static_cast<int>(0.8 * 480);
        cv::Point polyh[1][4];
        polyh[0][0] = cv::Point(0, h);
        polyh[0][1] = cv::Point(640, h);
        polyh[0][2] = cv::Point(640, 480);
        polyh[0][3] = cv::Point(0, 480);
        const cv::Point* ppt[1] = { polyh[0] };
        int npt[] = { 4 };
        cv::fillPoly(maskh, ppt, npt, 1, 255);
        
        masks = cv::Mat::zeros(480, 640, CV_8UC1);
        cv::Point polys[1][4]; //= { {0, 300}, {640, 300}, {640, 340}, {0, 340} };
        polys[0][0] = cv::Point(0, 300);
        polys[0][1] = cv::Point(640, 300);
        polys[0][2] = cv::Point(640, 340);
        polys[0][3] = cv::Point(0, 340);
        const cv::Point* ppt_s[1] = { polys[0] };
        cv::fillPoly(masks, ppt_s, npt, 1, 255);
        
        image = cv::Mat::zeros(480, 640, CV_8UC1);
        stopline = false;
        dotted = false;
        pl = 320;

        maskd = cv::Mat::zeros(480, 640, CV_8UC1);
        cv::Point polyd[1][4];// = { {0, 240}, {0, 480}, {256, 480}, {256, 240} };
        polyd[0][0] = cv::Point(0, 240);
        polyd[0][1] = cv::Point(0, 480);
        polyd[0][2] = cv::Point(256, 480);
        polyd[0][3] = cv::Point(256, 240);
        const cv::Point* ppt_d[1] = { polyd[0] };
        cv::fillPoly(maskd, ppt_d, npt, 1, 255);
        cv::namedWindow("Frame preview", cv::WINDOW_AUTOSIZE);
        
        ros::Rate rate(100); // Set the rate to 100 Hz
        while (ros::ok()) {
            ros::spinOnce();
            rate.sleep();
        }
    }

    void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
        try {
            cv::Mat cv_image = cv_bridge::toCvShare(msg, "bgr8")->image;
            addSquare(cv_image); // Add a square to the image
            auto start = high_resolution_clock::now();
            optimized_histogram(cv_image, true);
            auto stop = high_resolution_clock::now();
            auto duration = duration_cast<microseconds>(stop - start);
            std::cout << duration.count() << std::endl;
            // cv::imshow("Frame preview", cv_image);
            // cv::waitKey(1);

            // Publish the modified image
            sensor_msgs::ImagePtr modified_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", cv_image).toImageMsg();
            image_pub.publish(modified_msg);

        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
        }
    }

    std::vector<int> extract_lanes(cv::Mat hist_data) {
        std::vector<int> lane_indices;
        int previous_value = 0;
        for (int idx = 0; idx < hist_data.cols; ++idx) {
            int value = hist_data.at<int>(0, idx);
            if (value >= 1500 && previous_value == 0) {
                lane_indices.push_back(idx);
                previous_value = 255;
            } else if (value == 0 && previous_value == 255) {
                lane_indices.push_back(idx);
                previous_value = 0;
            }
        }
        if (lane_indices.size() % 2 == 1) {
            lane_indices.push_back(640 - 1);
        }
        return lane_indices;
    }

    double optimized_histogram(cv::Mat image, bool show = false) {
        stopline = false;
        cv::Mat img_gray;
        cv::cvtColor(image, img_gray, cv::COLOR_BGR2GRAY);
        int h = 480, w = 640;
        cv::Mat img_roi = img_gray & maskh;
        double minVal, maxVal;
        cv::Point minLoc, maxLoc;
        cv::minMaxLoc(img_roi, &minVal, &maxVal, &minLoc, &maxLoc);
        double threshold_value = std::min(std::max(maxVal - 55.0, 30.0), 200.0);
        cv::Mat thresh;
        cv::threshold(img_roi, thresh, threshold_value, 255, cv::THRESH_BINARY);
        cv::Mat hist = cv::Mat::zeros(1, w, CV_32SC1);
        cv::reduce(thresh, hist, 0, cv::REDUCE_SUM, CV_32S);

        cv::Mat img_rois = img_gray & masks;
        cv::minMaxLoc(img_roi, &minVal, &maxVal, &minLoc, &maxLoc); // Use img_roi or img_rois depending on your requirements
        double threshold_value_stop = std::min(std::max(maxVal - 65.0, 30.0), 200.0);
        cv::Mat threshs;
        cv::threshold(img_rois, threshs, threshold_value_stop, 255, cv::THRESH_BINARY);
        cv::Mat hists = cv::Mat::zeros(1, w, CV_32SC1);
        cv::reduce(threshs, hists, 0, cv::REDUCE_SUM, CV_32S);

        std::vector<int> stop_lanes = extract_lanes(hists);
        for (size_t i = 0; i < stop_lanes.size() / 2; ++i) {
            if (abs(stop_lanes[2 * i] - stop_lanes[2 * i + 1]) > 370 && threshold_value > 30) {
                stopline = true;
            }
        }

        std::vector<int> lanes = extract_lanes(hist);
        std::vector<double> centers;
        for (size_t i = 0; i < lanes.size() / 2; ++i) {
            if (3 < abs(lanes[2 * i] - lanes[2 * i + 1]) && abs(lanes[2 * i] - lanes[2 * i + 1]) < 100) {
                centers.push_back((lanes[2 * i] + lanes[2 * i + 1]) / 2.0);
            }
        }
        double center;
        if (centers.empty()) {
            center = w / 2.0;
        } else if (centers.size() == 1) {
            center = (centers[0] > w / 2.0) ? (centers[0] - 0) / 2 : (centers[0] * 2 + w) / 2;
        } else if (abs(centers[0] - centers.back()) < 200) {
            center = ((centers[0] + centers.back()) / 2 + 0) / 2.0;
            if ((centers[0] + centers.back()) > w) {
                center = (centers[0] + centers.back()) + w / 2;
            }
        } else {
            center = (centers[0] + centers.back()) / 2;
        }

        if (show) {
            if (stopline) {
                cv::putText(thresh, "Stopline detected!", cv::Point(static_cast<int>(w * 0.1), static_cast<int>(h * 0.1)), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
            }
            if (dotted) {
                cv::putText(image, "DottedLine!", cv::Point(static_cast<int>(w*0.1), static_cast<int>(h * 0.3)), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 255), 1, cv::LINE_AA);
        }
            cv::line(image, cv::Point(static_cast<int>(center), image.rows), cv::Point(static_cast<int>(center), static_cast<int>(0.8 * image.rows)), cv::Scalar(0, 0, 255), 5);
            cv::Mat add;
            cv::cvtColor(thresh, add, cv::COLOR_GRAY2BGR);
            cv::imshow("Lane", image + add);
            cv::waitKey(1);
        }
        return center;
    }

private:
    ros::NodeHandle nh;
    image_transport::ImageTransport it;
    image_transport::Subscriber image_sub;
    image_transport::Publisher image_pub;
    cv::Mat maskh, masks, image, maskd;
    bool stopline, dotted;
    int pl;
    void addSquare(cv::Mat& image) {
        cv::Point top_left(100, 100);
        cv::Point bottom_right(200, 200);
        cv::Scalar color(0, 255, 0); // Green
        int thickness = 2;
        cv::rectangle(image, top_left, bottom_right, color, thickness);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "CAMnod");
    LaneDetector laneDetector;
    return 0;
}
