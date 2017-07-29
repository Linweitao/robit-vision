#include "localizer.h"

Near_table::Near_table(){
	dis=0;
	x=0;
	y=0;
}
void Near_table::set(float a,float b,float c)
{
	dis=a;
 	x=b;
	y=c;
}
float Near_table::getx()
{
	return x;
}
float Near_table::gety()
{
	return y;
}




void localizer::Near_lookup_table()
{   
	int i,j, k = 0;
	float a,b,c,d,e;        
	FILE *fp;
	fp = fopen("list.txt", "r"); 
	for (i = 0; i <= 1800; i++)
	    for (j = 0; j <= 1200; j++)
	    {
		fscanf(fp, "%f %f %f %f %f", &d,&e,&a,&b,&c);
		data[int(d)][int(e)].set(a,b,c);
		//cout<<data[i][j].getx()<<endl;

	    }
	fclose(fp);
}

localizer::localizer()
{
    Near_lookup_table();
    int i;
    for( i = 0 ; i < sign_num ; i++ )
    {
	nearestx[i] = 5;
        nearesty[i] = 5;
	sub_local_x[i] = 3;
    	sub_local_y[i] = 3;
    }
    robot_x = 1.0;
    robot_y = 2.0;
    robot_theta =3.0;
    dx = 0.0;
    dy = 0.0;
    dtheta = 0.0;
    src=imread("field.jpg");
}

localizer::~localizer()
{
}

void localizer::set_robot_x(float x)
{
    robot_x = x ;
}

void localizer::set_robot_y(float y)
{
    robot_y = y ;
}

void localizer::set_robot_theta(float theta)
{
    robot_theta = theta ;
}

void localizer::msg_subsribe()
{
    sub = it.subscribe("signlocation", 1, &localizer::convert_callback, this);
}

void localizer::convert_callback(const localization::Samples::ConstPtr& msg)
{
    cout<<robot_x<<" "<<robot_y<<" "<<robot_theta<<endl;
    int i=0, j=0;
    for(i=0; i<sign_num; i++)
    {
	sub_local_x[i] = msg->local_x[i];
	sub_local_y[i] = msg->local_y[i];
	sub_local_theta[i] = msg->local_theta[i];
    //cout<<msg->local_theta[i]<<endl;
    }
    //match_degree(robot_x,robot_y,robot_theta);//test
    calculate();
    cout<<robot_x<<" "<<robot_y<<" "<<robot_theta<<endl;
    show=src.clone();
    match_degree(robot_x,robot_y,robot_theta);
    shape_robot(show,int(robot_x),int(robot_y));
    for(j=0;j<sign_num;j++) shape_sign(show,int(sub_world_x[j]),int(sub_world_y[j]));
    imshow("field",show);
    waitKey(1);
}

void localizer::gredient_desent()
{
    int i;
    float max = 10000;
    dx=0;
    dy=0;
    dtheta=0;
    for( i = 0 ; i < sign_num ; i++ )
    {
	dx += 2 * ( nearestx[i] - sub_world_x[i] ) / max;
    }
    for( i = 0 ; i < sign_num ; i++ )
    {
	dy += 2 * ( nearesty[i] - sub_world_y[i] ) / max;
    }
    for( i = 0 ; i < sign_num ; i++ )
    {
	dtheta += 2 * ( ( nearestx[i] - sub_world_x[i] ) * ( - sin(robot_theta) * sub_local_x[i] - cos(robot_theta) * sub_local_y[i] ) + 
                        ( nearesty[i] - sub_world_y[i] ) * ( cos(robot_theta) * sub_local_x[i] - sin(robot_theta) * sub_local_y[i] ) ) / max/5000;
    }
    //cout << "d: "<< dx << " " << dy << " " << dtheta << endl;
}

float localizer::match_degree(float x,float y,float theta)
{
    int i,j;
    float max = 10000;
    int signx=0,signy=0;
    float Sum=0;
    for ( i = 0 ; i < sign_num ; i++ )
    {	sub_world_x[i]=cos(theta) * sub_local_x[i] - sin(theta) * sub_local_y[i] +x;
	    sub_world_y[i]=sin(theta) * sub_local_x[i] + cos(theta) * sub_local_y[i] +y;

        if(sub_world_x[i]<=0)     signx=0;
	else if(sub_world_x[i]>=1800)	signx=1800;
	else signx=sub_world_x[i];

	if(sub_world_y[i]<=0)     signy=0;
	else if(sub_world_y[i]>=1200)    signy=1200;
	else signy=sub_world_y[i];

	nearestx[i]=data[int(signx)][int(signy)].getx();
	nearesty[i]=data[int(signx)][int(signy)].gety();
	
        Sum += (1.0 - ( ( sub_world_x[i] - nearestx[i] ) * ( sub_world_x[i] - nearestx[i] ) + ( sub_world_y[i] - nearesty[i] ) * ( sub_world_y[i] - nearesty[i] ) ) / max);
	//cout<<"sub and near: "<<sub_world_x[i]<<" "<<sub_world_y[i]<<" "<<nearestx[i]<<" "<<nearesty[i]<<endl;
	//cout<<"sum "<<Sum<<endl;
    } 
    return Sum;
}

void localizer::calculate()
{   //sign_num=25  ,step_width=200,   step_min=20,  max=10000,  dtheta=~/1000
    //sign_num=40  ,step_width=50,   step_min=2,  max=10000,  dtheta=~/5000
    int k=0;
    float step_width = 40;
    float step_min=5;
    float max_iterater = 5;
    float x=robot_x, y=robot_y, t=robot_theta;
    float sum=0;
    while(k!=max_iterater&&step_width>=step_min){

    	sum=match_degree(robot_x,robot_y,robot_theta);
    	gredient_desent();	
    	x= robot_x + dx * step_width;
    	y= robot_y + dy * step_width;
      t= robot_theta + dtheta * step_width;
      //t= robot_theta;
      if(((x-robot_x)*(x-robot_x)+(y-robot_y)*(y-robot_y))>1000){
        step_width = step_width / 2.0;
        continue;
      }
    	while(t<0) t+=2*PI;
    	while(t>2*PI) t-=2*PI;
      cout << x << " " <<y<< " " <<t<< " " <<robot_x<< " " <<robot_y<< " " << robot_theta<< endl;
      cout << "match: "<< match_degree(x,y,t) << " " << sum << endl;
    	if(match_degree(x,y,t)>sum)
    	{       //cout << "match: "<< match_degree(x,y,t) << " " << sum<< endl;
    		robot_x = x;
    		robot_y = y;
    		robot_theta = t;
    		//cout<<"k: "<<k<<endl;
    	}
    	step_width = step_width / 2.0;
    	//cout<<"test: "<<robot_x<<" "<<robot_y<<" "<<robot_theta<<endl;
    	k++;
       	//cout<< k++<<endl;
	
    }
}



void localizer::shape_sign(Mat &src,int x ,int y){
  Vec3b color;
  color[0]=255;
  color[1]=0;
  color[2]=0;
    y=1200-y;
  for(int i=y-5;i<y+5;i++)
  for(int j=x-5;j<x+5;j++)
  if(i<1200&&j<1800&&i>=0&&j>=0)src.at<Vec3b>(i, j)=color;
}

void localizer::shape_robot(Mat &src,int x ,int y){
    Vec3b color;
    color[0]=0;
    color[1]=0;
    color[2]=255;
    y=1200-y;
    for(int i=y-20;i<y+20;i++)
    for(int j=x-20;j<x+20;j++)
    if(i<1200&&j<1800&&i>=0&&j>=0)src.at<Vec3b>(i, j)=color;
}

