#include "ros/ros.h"
#include "my_workcell/LocalizePart.h"

class ScanNPlan
{
public:
  ScanNPlan(ros::NodeHandle& nh)
    {
        vision_client_ = nh.serviceClient<my_workcell::LocalizePart>("localize_part");
    }

    void start()
    {
        ROS_INFO("Attempting to localize part");
        // Localize the part
        my_workcell::LocalizePart srv;
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
    ros::init(argc, argv, "myworkcell_node");
    ros::NodeHandle nh;

    ROS_INFO("ScanNPlan node has been initialized");

    ScanNPlan app(nh);

    ros::Duration(.5).sleep();  // wait for the class to initialize
    app.start();

    ros::spin();
}