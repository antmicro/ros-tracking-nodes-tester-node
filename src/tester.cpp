#include "tester.hpp"

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <dirent.h>
#include <algorithm>
#include <sstream>
#include <fstream>
#include <cassert>

inline bool endsWith(std::string const & value, std::string const & ending)
{
    if (ending.size() > value.size()) return false;
    return std::equal(ending.rbegin(), ending.rend(), value.rbegin());
}

void listFiles(std::string dir_path, std::vector<std::string> &files, std::string extension="")
{
    DIR *dir;
    struct dirent *entry;

    if (!(dir = opendir(dir_path.c_str())))
        return;

    while ((entry = readdir(dir)) != NULL) {
        if (entry->d_type == DT_DIR) {
            char path[1024];
            if (strcmp(entry->d_name, ".") == 0 || strcmp(entry->d_name, "..") == 0)
                continue;
            snprintf(path, sizeof(path), "%s/%s", dir_path.c_str(), entry->d_name);
            listFiles(path, files, extension);
        } else {
            std::string file(entry->d_name);
            if (endsWith(file, extension)) {
                files.push_back(dir_path+"/"+entry->d_name);
            }
        }
    }
    closedir(dir);
}

Tester::Tester(std::string path)
{
    if (!readDirectory(path))
    {
        ROS_ERROR("Loading files error. Terminating");
        std::exit(1);
    }
    sortFramePaths();
    loadFrames();
    loadAnnotations();
    subscribe_advertise();
    run();
    saveRecords(path);
}

void Tester::run()
{
    if (annotations.size() != frames.size())
    {
        ROS_ERROR("number of annotations: %lu and number of frames: %lu differ",
                annotations.size(), frames.size());
        std::exit(1);
    }
    const bool visualize = 1; // for simplicity it's here for now
    if (visualize)
    {
        cv::namedWindow("Tester visualizer", cv::WINDOW_NORMAL);
        cv::resizeWindow("Tester visualizer", 800, 600);
    }

    double start_time = ros::Time::now().toSec();
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
            cv::imshow("Tester visualizer", frame);
            if (cv::waitKey(1) == 'q')
            {
                system("rosnode kill -a");
                system("killall rosnode");
            }
        }
        double iou = calculateIou(annotation, current_bbox);
        double fps = (processed_frames + 1) / (ros::Time::now().toSec() - start_time);
        ROS_INFO("IoU: %f%%, FPS: %f", iou * 100.0, fps);
        iouRecord.push_back(iou);
        fpsRecord.push_back(fps);
    }
    cv::destroyAllWindows();
}

void Tester::saveRecords(std::string path)
{
    std::string filename = path + "/out.csv";
    std::ofstream out(filename);
    if (!out)
    {
        ROS_ERROR("%s", ("Could not open file " + filename + " for writing.").c_str());
        exit(1);
    }
    ROS_INFO("Saving records to file");
    out << "Frame number, IoU, cumulative FPS\n";
    out << std::setprecision(5) << std::fixed;
    for (int i = 0; i < iouRecord.size(); i++)
        out << i + 1 << ", " << iouRecord[i] << ", " << fpsRecord[i] << '\n';

    ROS_INFO("Records saved");
}

bool Tester::readDirectory(std::string path)
{
    std::vector<std::string> ann_files;
    listFiles(path, ann_files, ".ann");
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
    listFiles(path, frames);
    frames.erase(std::remove_if(frames.begin(), frames.end(), [](std::string s)
            { return (s.substr(s.size() - 3) == "ann"); }), frames.end());
    frame_paths = frames;
    ROS_INFO("Found %lu frame files", frame_paths.size());
    return 1;
}

bool Tester::pathComparator(std::string p1, std::string p2)
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
        std::exit(1);
    }
    return a1 < a2;
}

void Tester::sortFramePaths()
{
    std::sort(frame_paths.begin(), frame_paths.end(), pathComparator);
}

void Tester::loadFrames()
{
    for (auto path : frame_paths)
    {
        cv::Mat image = cv::imread(path);
        if (!image.data)
        {
            ROS_ERROR("Unable to open image %s", path.c_str());
            std::exit(1);
        }
        frames.push_back(image);
    }
    ROS_INFO("Loaded frames");
}

void Tester::loadAnnotations()
{
    std::ifstream str(annotation_path);
    if (!str)
    {
        ROS_INFO("Error opening annotation file");
        std::exit(1);
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

void Tester::subscribe_advertise()
{
    ros::NodeHandle tester_handle;
    final_bbox_sub = tester_handle.subscribe("policy_manager/final_bbox", 1,
            &Tester::receiveBbox, this);
    frame_pub = tester_handle.advertise<sensor_msgs::Image>("frame_producer/frame", 1);
}

void Tester::receiveBbox(const policy_manager::optional_bbox_msg& msg)
{
    current_bbox.x = msg.bbox.x;
    current_bbox.y = msg.bbox.y;
    current_bbox.width = msg.bbox.width;
    current_bbox.height = msg.bbox.height;
    if (!msg.valid)
        current_bbox.x = 1e4, current_bbox.height = 0;

    bboxes_counter++;
}

void Tester::publishFrame(cv::Mat frame)
{
    const auto msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
    frame_pub.publish(msg);
}

double Tester::calculateIou(cv::Rect r1, cv::Rect r2)
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

    if (argc != 2)
    {
        ROS_ERROR("Provide path to directory containing frames with annotations as the only"
            "argument.");
        std::exit(1);
    }
    std::string path = argv[1];
    Tester tester(path);
}
