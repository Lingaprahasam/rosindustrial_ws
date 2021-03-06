#include "ros/ros.h"
#include "myworkcell/LocalizePart.h"

class ScanNPlan
{
public:
  ScanNPlan(ros::NodeHandle& nh)
    {
        vision_client_ = nh.serviceClient<myworkcell::LocalizePart>("localize_part");
    }

    void start(const std::string& base_frame)
    {
        ROS_INFO("Attempting to localize part");        
        // Localize the part
        myworkcell::LocalizePart srv;

        // Using Parameter to request
        srv.request.base_frame = base_frame;
        ROS_INFO_STREAM("Requesting pose in base frame: " << base_frame);

        if (!vision_client_.call(srv))
        {
            ROS_ERROR("Could not localize part");
            return;
        }
        ROS_INFO_STREAM("part localized: " << srv.response);
    }


private:
    // Planning components
    ros::ServiceClient vision_client_;
};

int main(int argc, char **argv)
{
    std::string base_frame;

    ros::init(argc, argv, "myworkcell_node");
    ros::NodeHandle nh;

    // Private Node Handle
    ros::NodeHandle private_node_handle ("~");

    // parameter name, string object reference, default value
    private_node_handle.param<std::string>("base_frame", base_frame, "world"); 

    ROS_INFO("ScanNPlan node has been initialized");

    ScanNPlan app(nh);

    ros::Duration(.5).sleep();  // wait for the class to initialize
    app.start(base_frame);

    ros::spin();
}