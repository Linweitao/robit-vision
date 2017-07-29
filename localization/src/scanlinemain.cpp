#include "sign.h"

int main(int argc, char** argv)
{
	ros::init(argc,argv,"scanline");
        sign_mark begin;
        begin.Msg_Subsribe();
	begin.Loc_Publisher();

        ros::spin();
	return 0;
}
