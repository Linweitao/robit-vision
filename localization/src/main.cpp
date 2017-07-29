#include "localizer.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "localizer");
    int i, j;
    float main_x = 0, main_y = 0, main_theta = 0;
    localizer sample1;
    sample1.msg_subsribe();
    sample1.set_robot_x(800.0);
    sample1.set_robot_y(900.0);
    sample1.set_robot_theta(180.0*PI/180.0);
    //sample1.calculate();

    ros::spin();
    return 0;  
}

