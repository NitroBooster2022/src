#include "ros/ros.h"
#include "include/yolo-fastestv2.h"
#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.h"
#include "sensor_msgs/image_encodings.h"
#include "std_msgs/Header.h"
#include "utils/Sign.h"

void imageCallback(const sensor_msgs::ImageConstPtr &msg, yoloFastestv2 *api, ros::Publisher *pub) {
    // Convert ROS image to OpenCV image
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception &e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    // Perform object detection
    std::vector<TargetBox> boxes;
    api->detection(cv_ptr->image, boxes);

    // Fill Sign message with all detected objects
    utils::Sign sign_msg;
    sign_msg.header.stamp = ros::Time::now();
    sign_msg.header.frame_id = "camera_frame"; // Set to appropriate frame_id if needed

    sign_msg.num = boxes.size();

    for (const auto &box : boxes) {
        sign_msg.objects.push_back(box.cate);
        sign_msg.box1.push_back(box.x1);
        sign_msg.box1.push_back(box.y1);
        sign_msg.box2.push_back(box.x2);
        sign_msg.box2.push_back(box.y2);
        // sign_msg.confidence.push_back(box.score);
    }

    // Publish Sign message
    pub->publish(sign_msg);
}


int main(int argc, char **argv) {
    // ... (same as before, including class_names and yoloFastestv2 instance)
    double num_iterations = 1;
    double total;
    static const char* class_names[] = {
        "oneway", "highwayentrance", "stopsign", "roundabout", "park", "crosswalk", "noentry", "highwayexit", "priority",
                "lights","block","pedestrian","car"
    };
    yoloFastestv2 api;

    // api.loadModel("./model/yolo-fastestv2-opt.param",
    //              "./model/yolo-fastestv2-opt.bin");
    // api.loadModel("./model/alice7s-opt.param",
    //               "./model/alice7s-opt.bin");
    api.loadModel("/home/antoinedeng/Documents/Simulator/src/control/src/model/coco7-opt.param",
                  "/home/antoinedeng/Documents/Simulator/src/control/src/model/coco7-opt.bin");

    // Initialize ROS node and publisher
    ros::init(argc, argv, "object_detector");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    ros::Publisher pub = nh.advertise<utils::Sign>("detected_objects", 1000);
    image_transport::Subscriber sub = it.subscribe("automobile/image_raw", 1, boost::bind(&imageCallback, _1, &api, &pub));

    // Spin ROS node
    ros::spin();

    return 0;
}
