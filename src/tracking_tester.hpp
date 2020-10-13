#ifndef TRACKING_TESTER_
#define TRACKING_TESTER_

#include <tracking_tester/optional_bbox_msg.h>
#include <stopwatch/saveRecordsService.h>

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
    TrackingTester() = delete; ///<initialize only with all arguments
    /**
     * The first argument is either `0` or `1` indicating whether visualizer of tester should be
     * launched. This creates another window. Second argument is FPS of playback.
     * It is an integerdetermining how many frames should be sent to system per second.
     * Note that very high valuesmight be unsupported. If it is set to 0, tester will wait
     * for bbox before it sends anotherframe. The third argument is path to directory
     * containing one `.ann` file with annotations in ALOV
     * dataset format and the rest of the files are images containing frame
     * info with names in format
     * `number.extension`, where `number` is any arbitrary integer (possibly with leading zeros)
     * of module not greater than $10^{18}$. Frames are processed in increasing `number` order
     * (sorted numerically, not lexicographically). For now it is assumed, that target is visible
     * in every frame and there is an annotation for every frame.
     * The paths have to be absolute paths.
     * The fourth (last) argument is path where output file will be written in csv format. If no
     * path is provided output isn't written anywhere.
     *
     * 
     * @param visualize open visualizer window?
     * @param playback_fps how fast should frames be sent to policy_manager
     * @param in_path path to directory with frames
     * @param out_path path to output, no output written if empty
     */
    TrackingTester(bool visualize, int playback_fps, std::string in_path,
            std::string out_path);
    ~TrackingTester(); ///< shutdown ros after tests
private:
    /**
     * Contains main loop
     * @param visualize open visualizer window?
     */
    void run(bool visualize, int playback_fps);

    /**
     * Save gathered data
     * @param path name of file to which data will be written
     */
    void saveRecords(std::string path);

    /**
     * Looks for required contents in specified directory
     * @path path to directory
     * @return true if successful, false otherwise
     */
    bool readDirectory(std::string path);

    /**
     * Comparator for std::sort to establish numerical order between paths
     * @param p1 absolute path
     * @param p2 absolute path
     * @return '<' equivalent
     */
    static bool pathComparator(std::string p1, std::string p2);

    /**
     * Sorts frame_paths field acording to pathComparator static method
    */
    void sortFramePaths();

    /**
     * Loads annotation file from annotation_path
     */
    void loadAnnotations();

    /**
     * Subscribes and advertises topics used by the node
     */
    void subscribe_advertise();
    /**
     * Callback for receiving bboxes from policy_manager
     * @param msg message from the topic
     */
    void receiveBbox(const policy_manager::optional_bbox_msg& msg);

    /**
     * Publishes frame to frame_producer/frame topic so that policy_manager can read it
     * @param frame the frame to be published
     */
    void publishFrame(cv::Mat frame);

    /**
     * Calculates intersection over union of two cv::Rects
     * @param r1 first rectangle
     * @param r2 second rectangle
     * @return the iou
     */
    double calculateIou(cv::Rect r1, cv::Rect r2);

    std::vector<std::string> frame_paths; ///<paths to frame files
    std::string annotation_path; ///<path to annotation file (.ann)
    std::vector<cv::Rect> annotations; ///<annotation file decyphered into cv::Rects
    cv::Rect current_bbox; ///<latest bbox received from policy_manager
    ros::Subscriber final_bbox_sub; ///<subscriber that receives bboxes
    ros::Publisher frame_pub; ///<publisher that publishes frames
    ros::ServiceClient stopwatch_save_client; ///<client for saveRecords

    /**
     * Helper struct for storing results
     */
    struct Record
    {
        double iou, frame_time;
        cv::Rect predicted_bbox;
        ros::Time time;
    };
    std::vector<Record> records; ///<records that are saved to csv file
};

#endif
