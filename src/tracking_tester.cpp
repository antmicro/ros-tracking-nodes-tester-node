#include "tracking_tester.hpp"
#include "FileSystemUtils.hpp"

#include <cxxopts.hpp>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <dirent.h>
#include <algorithm>
#include <sstream>
#include <fstream>
#include <cassert>

const double check_for_bbox_rate = 240.0;

TrackingTester::TrackingTester(bool visualize, int playback_fps, std::string in_path,
        std::string out_path)
{
    if (!readDirectory(in_path))
    {
        ROS_ERROR("Loading files error. Terminating");
        ros::shutdown();
    }
    sortFramePaths();
    loadAnnotations();
    subscribe_advertise();
    run(visualize, playback_fps);
    if (!out_path.empty()) saveRecords(out_path);
}

TrackingTester::~TrackingTester()
{
}

void TrackingTester::run(bool visualize, int playback_fps)
{
    if (annotations.size() != frame_paths.size())
    {
        ROS_ERROR("number of annotations: %lu and number of frames: %lu differ",
                annotations.size(), frame_paths.size());
        ros::shutdown();
    }
    if (visualize)
    {
        cv::namedWindow("TrackingTester visualizer", cv::WINDOW_NORMAL);
        cv::resizeWindow("TrackingTester visualizer", 800, 600);
    }

    double frame_start_time = ros::Time::now().toSec();
    double frame_end_time = ros::Time::now().toSec();
    for (size_t processed_frames = 0; processed_frames < frame_paths.size();
            processed_frames++)
    {
        std::string path = frame_paths[processed_frames];
        cv::Mat image = cv::imread(path);
        if (!image.data)
        {
            ROS_ERROR("Unable to open image %s", path.c_str());
            ros::shutdown();
        }

        auto frame = image;
        auto annotation = annotations[processed_frames];
        frame_start_time = ros::Time::now().toSec();
        publishFrame(frame);
        if (!ros::ok()) break;

        if (playback_fps) // passively wait until we can process another frame
        {
            ros::Time::sleepUntil(ros::Time(frame_end_time + 1.0 / playback_fps));
            ros::spinOnce();
        }
        else
        {
            ros::Rate loop_rate(check_for_bbox_rate); 
            // wait until we get another frame, but check for it rarely to save CPU time
            while (ros::ok() && records.size() < processed_frames + 1)
            {
                ros::spinOnce();
                loop_rate.sleep();
            }
        }

        double iou = calculateIou(annotation, current_bbox);
        double frame_time = (ros::Time::now().toSec() - frame_start_time);
        double total_frame_time = (ros::Time::now().toSec() - frame_end_time);
        frame_end_time = ros::Time::now().toSec();

        if (visualize)
        {
            if (playback_fps)
                ROS_INFO("IoU: IoU: %f%%, FPS: %f", iou * 100.0, 1.0 / total_frame_time);
            else 
                ROS_INFO("IoU: %f%%, Policy manager time: %f,"
                        "Total frame time: %f, Total FPS: %f",
                        iou * 100.0,
                        frame_time, total_frame_time, 1.0 / total_frame_time);
        }
        else
        {
             if ((processed_frames + 1) % (frame_paths.size() / 10) == 0)
                 ROS_INFO("Testing...\t%d%%", static_cast<int>(
                             (processed_frames + 1) / (frame_paths.size() / 10) * 10));
        }

        Record record(iou, frame_time, current_bbox, ros::Time::now());
        records.push_back(record);
        
        if (visualize)
        {   
            cv::rectangle(frame, annotation, cv::Scalar(0, 255, 0));
            cv::rectangle(frame, current_bbox, cv::Scalar(0, 0, 255));
            cv::imshow("TrackingTester visualizer", frame);
            if (cv::waitKey(1) == 'q')
            {
                system("rosnode kill -a");
                system("killall roscore");
            }
        }
    }
    double iou_avg = 0.0;
    for (auto rec : records)
    {
        auto iou = rec.iou;
        iou_avg += iou;
    }
    iou_avg /= records.size();
    iou_avg *= 100.0;
    ROS_INFO("----- Avarage IoU: %5.2f%% -----", iou_avg);
    if (visualize)
    {
        while (cv::waitKey(100) != 'q');
        cv::destroyAllWindows();
    }
}

void TrackingTester::saveRecords(std::string path)
{
    stopwatch::saveRecordsService stopwatch_srv;
    std::string stopwatch_path = path.substr(0, path.find_last_of("."))
        + "_stopwatch.csv";
    stopwatch_srv.request.out_path = stopwatch_path;
    if (!stopwatch_save_client.call(stopwatch_srv))
    {
        ROS_ERROR("Stopwatch failed");
    }
    std::string filename = path;
    std::ofstream out(filename);
    if (!out)
    {
        ROS_ERROR("%s", ("Could not open file " + filename + " for writing.").c_str());
        ros::shutdown();
    }
    ROS_INFO("Saving records to file");
    out << "frame_number,time,iou,frame_time,left,top,width,height,realLeft,realTop,realWidth,"
        "realHeight\n";
    out << std::setprecision(5) << std::fixed;
    for (size_t i = 0; i < records.size(); i++)
    {
        auto rec = records[i];
        auto bbox = rec.predicted_bbox;
        auto bbox_real = annotations[i];
        out << i + 1 << ',' << rec.time << ',' << rec.iou << ',' << rec.frame_time << ','
           << bbox.x << ',' << bbox.y << ',' << bbox.width << ',' << bbox.height << ','
           << bbox_real.x << ',' << bbox_real.y << ',' << bbox_real.width << ','
               << bbox_real.height << '\n';
    }
    ROS_INFO("Records saved");
}

bool TrackingTester::readDirectory(std::string path)
{
    std::vector<std::string> ann_files;
    FileSystemUtils::listFiles(path, ann_files, ".ann");
    if (ann_files.size() == 0)
    {
        ROS_ERROR("Found no annotation files.");
        return false;
    }
    else if (ann_files.size() > 1)
    {
        ROS_ERROR("Multiple annotation files aren't supported."
               "Merge them into one file.");
        return false;
    }
    else annotation_path = ann_files.back();

    std::vector<std::string> frames;
    FileSystemUtils::listFiles(path, frames);
    frames.erase(std::remove_if(frames.begin(), frames.end(), [](std::string s)
            { return s.size() >= 3 && (s.substr(s.size() - 3) == "ann"); }), frames.end());
    frame_paths = frames;
    ROS_INFO("Found %lu frame files", frame_paths.size());
    return true;
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
    stopwatch_save_client = tester_handle.serviceClient<stopwatch::saveRecordsService>(
            "stopwatch/saveRecords");
}

void TrackingTester::receiveBbox(const policy_manager::optional_bbox_msg& msg)
{
    current_bbox.x = msg.bbox.x;
    current_bbox.y = msg.bbox.y;
    current_bbox.width = msg.bbox.width;
    current_bbox.height = msg.bbox.height;
    if (!msg.valid)
        current_bbox = cv::Rect{};
}

void TrackingTester::publishFrame(cv::Mat frame)
{
    const auto msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
    frame_pub.publish(msg);
}

double TrackingTester::calculateIou(cv::Rect r1, cv::Rect r2)
{
    cv::Rect intersection = r1 & r2;
    double iou = static_cast<double>(intersection.area())
        / (r1.area() + r2.area() - intersection.area());
    return iou;
}

int main(int argc, char** argv)
{
   	cxxopts::Options options("tracking_tester", "Used to test tracking performance");
	options.add_options()
		("v,visualize", "Print frames on screen")
		("f,fps", "fps of playback; default is wait mode",
		 cxxopts::value<int>()->default_value(0))
		("i,input", "path to directory containing frames and annotations",
		 cxxopts::value<std::string>())
		("o,output", "path where output should be saved",
		 cxxopts::value<std::string>()->default_value(""));
	auto args = options.parse(argc, argv);

	ros::init(argc, argv, "tester");
    ROS_INFO("Initialized!");
    ros::NodeHandle tester_handle;

    std::string in_path = args["i"], out_path = args["o"];
    TrackingTester tester(args["v"], args["f"], in_path, out_path);
    system("rosnode kill -a");
    system("killall roscore");
}
