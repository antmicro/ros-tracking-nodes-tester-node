#include "tracking_tester.hpp"
#include "FileSystemUtils.hpp"

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <dirent.h>
#include <algorithm>
#include <sstream>
#include <fstream>
#include <cassert>

TrackingTester::TrackingTester(std::string in_path, std::string out_path)
{
    if (!readDirectory(in_path))
    {
        ROS_ERROR("Loading files error. Terminating");
        ros::shutdown();
    }
    sortFramePaths();
    loadFrames();
    loadAnnotations();
    subscribe_advertise();
    run();
    if (!out_path.empty()) saveRecords(out_path);
}

void TrackingTester::run()
{
    if (annotations.size() != frames.size())
    {
        ROS_ERROR("number of annotations: %lu and number of frames: %lu differ",
                annotations.size(), frames.size());
        ros::shutdown();
    }
    const bool visualize = 1; // for simplicity it's here for now
    if (visualize)
    {
        cv::namedWindow("TrackingTester visualizer", cv::WINDOW_NORMAL);
        cv::resizeWindow("TrackingTester visualizer", 800, 600);
    }

    double frame_start_time = ros::Time::now().toSec();
    for (int processed_frames = 0; processed_frames < frames.size(); processed_frames++)
    {
        auto frame = frames[processed_frames];
        auto annotation = annotations[processed_frames];
        if (!ros::ok()) break;
        publishFrame(frame);
        while (ros::ok() && bboxes_counter < processed_frames + 1)
            ros::spinOnce();
            
        if (visualize)
        {   
            cv::rectangle(frame, annotation, cv::Scalar(0, 255, 0));
            cv::rectangle(frame, current_bbox, cv::Scalar(0, 0, 255));
            cv::imshow("TrackingTester visualizer", frame);
            if (cv::waitKey(1) == 'q')
            {
                system("rosnode kill -a");
                system("killall rosnode");
            }
        }
        double iou = calculateIou(annotation, current_bbox);
        auto curr_time = ros::Time::now().toSec();
        double frame_time = (curr_time - frame_start_time);
        frame_start_time = curr_time;
        ROS_INFO("IoU: %f%%, Frame time: %f", iou * 100.0, frame_time);
        iou_record.push_back(iou);
        frame_time_record.push_back(frame_time);
    }
    cv::destroyAllWindows();
}

void TrackingTester::saveRecords(std::string path)
{
    std::string filename = path;
    std::ofstream out(filename);
    if (!out)
    {
        ROS_ERROR("%s", ("Could not open file " + filename + " for writing.").c_str());
        ros::shutdown();
    }
    ROS_INFO("Saving records to file");
    out << "Frame number, IoU, frame time\n";
    out << std::setprecision(5) << std::fixed;
    for (int i = 0; i < iou_record.size(); i++)
        out << i + 1 << ",\t" << iou_record[i] << ",\t" << frame_time_record[i] << '\n';

    ROS_INFO("Records saved");
}

bool TrackingTester::readDirectory(std::string path)
{
    std::vector<std::string> ann_files;
    FileSystemUtils::listFiles(path, ann_files, ".ann");
    if (ann_files.size() == 0)
    {
        ROS_ERROR("Found no annotation files.");
        return 0;
    }
    else if (ann_files.size() > 1)
    {
        ROS_ERROR("Multiple annotation files aren't supported."
               "Merge them into one file.");
        return 0;
    }
    else annotation_path = ann_files.back();

    std::vector<std::string> frames;
    FileSystemUtils::listFiles(path, frames);
    frames.erase(std::remove_if(frames.begin(), frames.end(), [](std::string s)
            { return (s.substr(s.size() - 3) == "ann"); }), frames.end());
    frame_paths = frames;
    ROS_INFO("Found %lu frame files", frame_paths.size());
    return 1;
}

bool TrackingTester::pathComparator(std::string p1, std::string p2)
{
    p1 = p1.substr(0, p1.find_last_of("."));
    p2 = p2.substr(0, p2.find_last_of("."));
    p1 = p1.substr(p1.find_last_of("/") + 1);
    p2 = p2.substr(p2.find_last_of("/") + 1);
    long long a1{}, a2{};
    try
    { a1 = std::stoll(p1), a2 = std::stoll(p2); }
    catch (const std::exception& e)
    {
        ROS_ERROR("Invalid frame filename"); 
        ros::shutdown();
    }
    return a1 < a2;
}

void TrackingTester::sortFramePaths()
{
    std::sort(frame_paths.begin(), frame_paths.end(), pathComparator);
}

void TrackingTester::loadFrames()
{
    for (auto path : frame_paths)
    {
        cv::Mat image = cv::imread(path);
        if (!image.data)
        {
            ROS_ERROR("Unable to open image %s", path.c_str());
            ros::shutdown();
        }
        frames.push_back(image);
    }
    ROS_INFO("Loaded frames");
}

void TrackingTester::loadAnnotations()
{
    std::ifstream str(annotation_path);
    if (!str)
    {
        ROS_INFO("Error opening annotation file");
        ros::shutdown();
    }
    std::string line;
    while (getline(str, line))
    {
        std::stringstream ss(line);
        double x1, y1, x2, y2, buff;
        str >> buff >> x1 >> y1 >> buff >> buff >> buff >> buff >> x2 >> y2;
        int x = std::round(x1), y = std::round(y1), width = std::round(x2 - x1), 
            height = std::round(y2 - y1);
        cv::Rect rect(x, y, width, height);
        annotations.push_back(rect);
    }
    ROS_INFO("Annotations loaded");
}

void TrackingTester::subscribe_advertise()
{
    ros::NodeHandle tester_handle;
    final_bbox_sub = tester_handle.subscribe("policy_manager/final_bbox", 1,
            &TrackingTester::receiveBbox, this);
    frame_pub = tester_handle.advertise<sensor_msgs::Image>("frame_producer/frame", 1);
}

void TrackingTester::receiveBbox(const policy_manager::optional_bbox_msg& msg)
{
    current_bbox.x = msg.bbox.x;
    current_bbox.y = msg.bbox.y;
    current_bbox.width = msg.bbox.width;
    current_bbox.height = msg.bbox.height;
    if (!msg.valid)
        current_bbox.x = 1e4, current_bbox.height = 0;

    bboxes_counter++;
}

void TrackingTester::publishFrame(cv::Mat frame)
{
    const auto msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
    frame_pub.publish(msg);
}

double TrackingTester::calculateIou(cv::Rect r1, cv::Rect r2)
{
    using ll = long long;
    auto oneDimension = [](ll x1, ll l1, ll x2, ll l2)
        {
            if (x1 > x2) std::swap(x1, x2), std::swap(l1, l2);
            return std::max(0ll, std::min(x1 + l1 - x2, l2));
        };
    ll areaOfIntersection = oneDimension(r1.x, r1.width, r2.x, r2.width)
        * oneDimension(r1.y, r1.height, r2.y, r2.height);
    return 1.0 * areaOfIntersection / (1ll * r1.width * r1.height + 1ll * r2.width * r2.height
            - areaOfIntersection);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "tester");
    ROS_INFO("Initialized!");
    ros::NodeHandle tester_handle;

    if (argc < 2 || argc > 3)
    {
        ROS_ERROR("Provide one or two arguments: path to directory containing frames with"
                " annotations and path where output should be saved (optional)");
        ros::shutdown();
    }
    std::string in_path = argv[1], out_path = (argc == 3 ? argv[2] : "");
    TrackingTester tester(in_path, out_path);
}
