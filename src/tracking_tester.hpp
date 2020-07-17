#ifndef TRACKING_TESTER_
#define TRACKING_TESTER_

#include "tracking_tester/optional_bbox_msg.h"

#include <string>
#include <vector>
#include <opencv2/core/core.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <ros/ros.h>

/**
 * Class testing quality of output produced by policy_manager
 * Produces own frames on topic frame_producer/frame
 * Reads bboxes from policy manager
 * Outputs results to csv file
 * One instance should be used for one dataset
 */
class TrackingTester
{
public:
    TrackingTester() = delete; // initialize only with all arguments
    /**
     * The first argument is either `0` or `1` indicating whether visualizer of tester should be
     * launched. This creates another window. Second argument is path to directory
     * containing one `.ann` file with annotations in ALOV
     * dataset format and the rest of the files are images containing frame info with names in format
     * `number.extension`, where `number` is any arbitrary integer (possibly with leading zeros)
     * of module not greater than $10^{18}$. Frames are processed in increasing `number` order
     * (sorted numerically, not lexicographically). For now it is assumed, that target is visible
     * in every frame and there is an annotation for every frame.
     * The third (last) argument is path where output file will be written in csv format. If no
     * path is provided output isn't written anywhere.
     * 
     * @param vssualsze open visualizer window?
     * @param in_path path to directory with frames
     * @param out_path path to output, no output written if empty
     */
    TrackingTester(bool visualize, std::string in_path, std::string out_path);
private:
    void run(bool visualize);
    void saveRecords(std::string path);
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
    std::size_t bboxes_counter{};
    std::vector<double> iou_record, frame_time_record;
};

#endif
