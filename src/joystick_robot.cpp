#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>

//操作モードの変更を行う関数
void cv_mode_change(int& ,int , int&);
void speed_change(int& ,int , int&);

class TeleopTurtle{
	public:
	TeleopTurtle();
	private:
	void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
	ros::NodeHandle nh;
	int vel_linear,vel_angular,vel_viceangular,mode=0,low_speed=0;
	int Buttons_data[13];
	double l_scale_,a_scale_;
	ros::Publisher vel_pub_;
	ros::Subscriber joy_sub_;
};

TeleopTurtle::TeleopTurtle():vel_linear(1),vel_angular(0),l_scale_(1), a_scale_(-1), vel_viceangular(2)
{
	joy_sub_=nh.subscribe<sensor_msgs::Joy>("joy",10,&TeleopTurtle::joyCallback,this);
	//vel_pub_=nh.advertise<geometry_msgs::Twist>("turtle1/cmd_vel",1);
	vel_pub_=nh.advertise<geometry_msgs::Twist>("/cmd_vel",1);
}

void TeleopTurtle::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
	geometry_msgs::Twist twist;
	twist.linear.x=l_scale_*joy->axes[vel_linear];
	if ((joy->buttons[2] || joy->buttons[4])&&!joy->buttons[6]&&!joy->buttons[8]&&!joy->buttons[10]&&!joy->buttons[7]&&!joy->buttons[9]){
		cv_mode_change(mode,joy->buttons[4],Buttons_data[4]);
		speed_change(low_speed,joy->buttons[2],Buttons_data[2]);
	}
	if(low_speed){
		a_scale_=-1;
		l_scale_=0.5;
	}
	else {
		a_scale_=-2;
		l_scale_=1;
	}
	switch (mode)
	{
		case 0:
			twist.angular.z=a_scale_*joy->axes[vel_angular];
			break;
		
		case 1:
			twist.angular.z=a_scale_*joy->axes[vel_viceangular];
			break;
		
		case 2:
			if(twist.linear.x==0)
				twist.angular.z=a_scale_*joy->axes[2];
			else twist.angular.z=a_scale_*joy->axes[0];
			break;
				
		default:
			twist.angular.z=a_scale_*joy->axes[vel_angular];
			break;
	}
	
	ROS_INFO_STREAM("("<<joy->axes[vel_linear]<<""<<joy->axes[vel_angular]<<""<<joy->axes[vel_viceangular]<<")");

	vel_pub_.publish(twist);
}
int main(int argc,char** argv)
{
	ros::init(argc,argv,"teleop_robot");
	TeleopTurtle teleop_robot;
	ros::spin();
}

//操作モードの変更を行う関数
void cv_mode_change(int& mode,int Buttons , int& Buttons_data)
{
	if(Buttons==1&&Buttons_data==0&&(mode==0||mode==1)){
		++mode;																							//X.Yモード→X.Rzモード→X.Y.Rzモード→X.Yモード
		++Buttons_data;
	}
	else if(Buttons==1&&Buttons_data==0){
		mode=0;
		++Buttons_data;
	}
	else if(Buttons==0)
		Buttons_data=0;
}

void speed_change(int& low_speed ,int Buttons , int& Buttons_data){
	if(Buttons==1&&Buttons_data==0&&low_speed==0){
		++low_speed;
		++Buttons_data;
	}
	else if(Buttons==1&&Buttons_data==0){
		low_speed=0;
		++Buttons_data;
	}
	else if(Buttons==0)
		Buttons_data=0;
}
