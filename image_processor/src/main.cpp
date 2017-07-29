/*
 该节点没用，暂且不管
  20170727
*/
#include "msg_transport.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "image_processor");
    RGB_Transport Image;
    for(;;)
    {
        Image.Msg_Subsribe();
        Image.Msg_Publisher();
        Mat dst=Image.get_src();
        if(dst.empty())
        {
            cout << "dst empty" << endl;
        }
        else
	{
  	    imshow("output_src",dst);
            waitKey(1);
	}
        ros::spin();
    }
}
