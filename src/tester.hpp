#include "tracking_tester/optional_bbox_msg.h"

#include <string>
#include <vector>
#include <opencv2/core/core.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <ros/ros.h>

class Tester
{
public:
    Tester() = delete;
    Tester(std::string in_path, std::string out_path);
    void run();
    void saveRecords(std::string path);
private:
    bool readDirectory(std::string path);
    static bool pathComparator(std::string p1, std::string p2);
    void sortFramePaths();
    void loadFrames();
    void loadAnnotations();
    void subscribe_advertise();
    void receiveBbox(const policy_manager::optional_bbox_msg& msg);
    void publishFrame(cv::Mat frame);
    double calculateIou(cv::Rect r1, cv::Rect r2);
    
    std::vector<std::string> frame_paths;
    std::string annotation_path;
    std::vector<cv::Mat> frames;
    std::vector<cv::Rect> annotations;
    cv::Rect current_bbox;
    ros::Subscriber final_bbox_sub;
    ros::Publisher frame_pub;
    int bboxes_counter{};
    std::vector<double> iou_record, frame_time_record;
};
