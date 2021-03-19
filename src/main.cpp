#include "tracking_tester.hpp"
#include "FileSystemUtils.hpp"

#include <cxxopts.hpp>

int main(int argc, char** argv)
{
    cxxopts::Options options("tracking_tester", "Used to test tracking performance");
    options.add_options()
        ("v,visualize", "Print frames on screen")
        ("f,fps", "fps of playback; default is wait mode", cxxopts::value<int>()->default_value("0"))
        ("i,input", "path to directory containing frames and annotations", cxxopts::value<std::string>())
        ("o,output", "path where output should be saved", cxxopts::value<std::string>()->default_value(""));
    auto args = options.parse(argc, argv);

    ros::init(argc, argv, "tester");
    ROS_INFO("Initialized!");
    ros::NodeHandle tester_handle;

    std::string in_path = args["i"].as<std::string>(), out_path = args["o"].as<std::string>();
    TrackingTester tester(args["v"].as<bool>(), args["f"].as<int>(), in_path, out_path);
}
