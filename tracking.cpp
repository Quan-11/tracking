#include <stdlib.h>
#include <cv.h>
#include <math.h>
#include <highgui.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float32.h"
#include <geometry_msgs/Twist.h>
#include "sensor_msgs/Image.h"

#define LINEAR_X 0

using namespace cv;
using namespace std;
double threshold = 0.8;

Mat out_open;
double maxArea;

///////////颜色分割+开操作滤除小区域//////////

//目标识别及分割
void cutandshow(Mat img, int hmin, int hmax, int smin, int smax, int vmin, int vmax)
{
	int i,j,h,s,v;
	Mat target_colour; //存放目标图像
	target_colour.create(img.rows, img.cols, CV_8UC1);
	vector<Mat> hsvChannels;
	split(img, hsvChannels); //将img三通道分离
	//寻找目标
	for(i = 0; i < target_colour.rows; i++)
	{
		for(j = 0; j < target_colour.cols; j++)//遍历
		{
			h = hsvChannels[0].at<uchar>(i,j);
			s = hsvChannels[1].at<uchar>(i,j);
			v = hsvChannels[2].at<uchar>(i,j);
			//二值化
			if(h >= hmin && h <= hmax && s >= smin && s <= smax && v >= vmin && v <= vmax)  //阈值范围
				target_colour.at<uchar>(i,j) = 255;
			else
				target_colour.at<uchar>(i,j) = 0;
		}
	}
	//开操作
	out_open = target_colour.clone();
	Mat kernel = getStructuringElement(MORPH_RECT, Size(5,5)); //定义kernel操作矩阵
	morphologyEx(target_colour, out_open, MORPH_OPEN, kernel); //开操作
	
	//计算感兴趣区域的像素总数，用于衡量该区域面积的大小
	int num = 0;
	for(i = 0; i < out_open.rows; i++)
	{
		for(j = 0; j < out_open.cols; j++)
		{
			if(out_open.at<uchar>(i,j) == 255)  num++;
		}
	}

	//imshow("out_open",out_open);
	//waitKey();
	return num;
}


//找中点，求面积
Point FindCenter(Mat aftergs)
{
	vector<vector<Point> > contours; //储存轮廓点集
	vector<Vec4i> hierarchy;
	findContours(out_open, contours, hierarchy, CV_RETR_EXTERNAL, CHAIN_APPROX_NONE, Point()); ///找轮廓
    //找最大的轮廓作为目标，排除干扰
	maxArea = 0;
	int maxIndex = 0;
	Point center;
	if(contours.size() == 0) //若未找到轮廓，返回（0，0)
	{
		center.x = 0;
		center.y = 0;
		cout<<"cant find"<<endl;
		return center;
	}
	else
	{
		for(int i = 0; i<contours.size(); i++)
		{
			int area = contourArea(contours[i]);
			if(area > maxArea) //找最大的轮廓
			{
				maxArea = area;//全局变量储存最大轮廓面积
				maxIndex = i;
			}
		}
		drawContours(aftergs,contours,maxIndex,Scalar(0,0,255),1,8,hierarchy);//画出最大轮廓
		imshow("target_edge", aftergs);

		//找目标中点
		Point tmp;
		double x_sum = 0, y_sum = 0;
		for(int j = 0; j < contours[maxIndex].size(); j++)//累计轮廓所有点的x,y坐标值
		{
			tmp = contours[maxIndex][j];
			x_sum += tmp.x;
			y_sum += tmp.y;
		}
		if(maxArea < 0.03 * aftergs.rows * aftergs.cols) //若轮廓内区域过小，返回(0,0)
		{
			center.x = 0;
			center.y = 0; 
		}
		else
		{
			center.x = x_sum / contours[maxIndex].size();
			center.y = y_sum / contours[maxIndex].size(); //得到中心点坐标
		}
		return center;
	}
}


int main(int argc, char**argv)
{
	VideoCapture capture;
	capture.open(0);//1为打开zed相机，0为打开笔记本摄像头

	ROS_WARN("*****START");
	ros::init(argc,argv,"TargetTrack");//初始化ROS节点
		ros::NodeHandle n;

		// ros::Rate loop_rate(10);//定义速度发布频率
		ros::Publisher pub= n.advertise<geometry_msgs::Twist>("/smoother_cmd_vel",5);//定义速度发布器

	if (!capture.isOpened())
    {
        cout<<"cannot open the camera!"<<endl;
        return 0;
    }
	waitKey(1000);
	Mat frame;//当前帧图片
	int nFrames= 0;//图片帧数
	int frameWidth= capture.get(CV_CAP_PROP_FRAME_WIDTH);//图片宽
	int frameHeight = capture.get(CV_CAP_PROP_FRAME_HEIGHT);//图片高
	int target_num;

	//angular_PID
	int delta_x1 = 0, delta_x2 = 0, delta_x3 = 0; //x1 当前时刻偏差,x2 上一时刻偏差, x3上上时刻偏差
	double Kp_w = 3, Ki_w = 0.01 * 1, Kd_w = 3, K_w = 0.001/6.0;//K是像素位置与速度的转换比例
	double v0_angular = 0; //上一时刻的角速度(初值)
	double delta_v_angular, v1_angular;

	//处理中心点在中心线两侧跳变
	int center_last= 0, center_now = 0;
	int flag = 2;

	//linear_PID
	double Kp_v = 3, Ki_v = 0.01 * 1, Kd_v = 3, K_v = 0.000001/6.0;//K是像素面积与速度的转换比例
	int delta_area1 = 0, delta_area2 = 0, delta_area3 = 0;
	double v0_linear = 0.2;
	double v1_linear, delta_v_linear;

	double k;

	while(ros::ok())
	{
		capture.read(frame);
		if(frame.empty())
		{
			break;
		}
		//Mat frln = frame.clone();//使用笔记本摄像头
		Mat frln = frame(cv::Rect(0,0,frame.cols /2,frame.rows));//截取zed的左目图片
		Mat HSV_img;
		Mat aftergs;
		GaussianBlur(frln,aftergs,Size(3,3),1);//高斯滤波
		cvtColor(aftergs, HSV_img, CV_RGB2HSV);//色度空间转换

		target_num = cutandshow(HSV_img, 78, 155, 43, 255, 46,255);//目标识别及分割
		Point center;
		center = FindCenter(aftergs);//寻找中心点
		circle(aftergs, center, 10, Scalar(0,0,255), -1);//绘制中心点
		imshow("center",aftergs);

		//运动控制
		geometry_msgs::Twist vel;

		//角速度PID
		delta_x3 = delta_x2;
		delta_x2 = delta_x1;
		delta_x1 = abs(center.x - frln.cols/2);
		delta_v_angular = K_w *(Kp_w * (delta_x1 - delta_x2) + Ki_w * delta_x1 + Kd_w * (delta_x1 - 2 * delta_x2 + delta_x3));
		v1_angular = v0_angular + delta_v_angular;
		v0_angular = v1_angular;
		//处理中心点在中心线两侧跳变
		center_last = center_now;
		center_now = center.x;

		//线速度PID
		// delta_area3 = delta_area2;
		// delta_area2 = delta_area1;
		// delta_area1 = abs(maxArea - 0.2 * frln.cols * frln.rows);

		// delta_v_linear = K_v *(Kp_v * (delta_area1 - delta_area2) + Ki_v * delta_area1 + Kd_v * (delta_area1 - 2 * delta_area2 + delta_area3));
		// v1_linear = v0_linear + delta_v_linear;
		// v0_linear = v1_linear;
		// k = maxArea/(frln.cols * frln.rows);
		// cout<<v1_linear<<endl<<endl<<k<<endl;




		//角度  顺时针angular<0
		if(center.x == 0 && center.y == 0) //当前图像未检测到目标图像，控制机器人“找”
		{
			if(center_last > frln.cols/2 || flag == 0)
			{
				vel.linear.x = 0;
				vel.linear.y = 0;
				vel.linear.z = 0;
				vel.angular.x = 0;
				vel.angular.y = 0;
				vel.angular.z = -0.35;
				flag = 0;
			}
			else
			{
				vel.linear.x = 0;
				vel.linear.y = 0;
				vel.linear.z = 0;
				vel.angular.x = 0;
				vel.angular.y = 0;
				vel.angular.z = 0.35;
				flag = 1;
			}
			cout<<"找"<<endl;
		}
		else if(center.x > frln.cols/2) //目标区域在右侧，控制机器人向右旋转
		{
			flag = 2;
			cout<<"右"<<endl;
			if(maxArea < frln.cols * frln.rows * 0.2 && maxArea > frln.cols * frln.rows*0.03) //目标区域距离机器人较远，控制机器人向前运动
			{
				//线速度PID
				delta_area3 = delta_area2;
				delta_area2 = delta_area1;
				delta_area1 = abs(maxArea - 0.2 * frln.cols * frln.rows);
				delta_v_linear = K_v *(Kp_v * (delta_area1 - delta_area2) + Ki_v * delta_area1 + Kd_v * (delta_area1 - 2 * delta_area2 + delta_area3));
				v1_linear = v0_linear + delta_v_linear;
				v0_linear = v1_linear;
				k = maxArea/(frln.cols * frln.rows);
				cout<<v1_linear<<endl<<endl<<k<<endl;
				///////////
				if((center_last-frln.cols/2)*(center_now-frln.cols/2)<=0)
				{
					vel.linear.x = v1_linear;
					vel.linear.y = 0;
					vel.linear.z = 0;
					vel.angular.x = 0;
					vel.angular.y = 0;
					vel.angular.z = v1_angular;
				}
				else
				{
					vel.linear.x = v1_linear;
					vel.linear.y = 0;
					vel.linear.z = 0;
					vel.angular.x = 0;
					vel.angular.y = 0;
					vel.angular.z = (-1) * v1_angular;
					cout<<"前"<<endl;
				}
			}
			else if(maxArea > frln.cols * frln.rows * 0.4) //目标区域距离机器人过近，控制机器人向后运动
			{
				vel.linear.x = -0.1;
				vel.linear.y = 0;
				vel.linear.z = 0;
				vel.angular.x = 0;
				vel.angular.y = 0;
				vel.angular.z = 0;
				cout<<"后"<<endl;
			}
			else //目标区域距离机器人在合适位置，机器人保持不动
			{
				if(center.x > frln.cols/2 && center.x < frln.cols/2 + 100)
				{
					vel.linear.x = 0;
					vel.linear.y = 0;
					vel.linear.z = 0;
					vel.angular.x = 0;
					vel.angular.y = 0;
					vel.angular.z = 0;
					cout<<"停"<<endl;
				}
				else
				{
					vel.linear.x = 0;
					vel.linear.y = 0;
					vel.linear.z = 0;
					vel.angular.x = 0;
					vel.angular.y = 0;
					vel.angular.z = v1_angular * (-1);
				}
			}
		}
		else if(center.x < frln.cols/2 && center.y != 0) //目标区域在左侧，控制机器人向左旋转
		{
			flag = 2;
			cout<<"左"<<endl;
			if(maxArea < frln.cols * frln.rows * 0.2 && maxArea > frln.cols * frln.rows * 0.05) //目标区域距离机器人较远，控制机器人向前运动
			{
				//线速度PID
				delta_area3 = delta_area2;
				delta_area2 = delta_area1;
				delta_area1 = abs(maxArea - 0.2 * frln.cols * frln.rows);
				delta_v_linear = K_v *(Kp_v * (delta_area1 - delta_area2) + Ki_v * delta_area1 + Kd_v * (delta_area1 - 2 * delta_area2 + delta_area3));
				v1_linear = v0_linear + delta_v_linear;
				v0_linear = v1_linear;
				k = maxArea/(frln.cols * frln.rows);
				///////////////
				if((center_last-frln.cols/2) * (center_now-frln.cols/2) <= 0)
				{
					vel.linear.x = v1_linear;
					vel.linear.y = 0;
					vel.linear.z = 0;
					vel.angular.x = 0;
					vel.angular.y = 0;
					vel.angular.z = (-1) * v1_angular;
				}
				else
				{
					vel.linear.x = v1_linear;
					vel.linear.y = 0;
					vel.linear.z = 0;
					vel.angular.x = 0;
					vel.angular.y = 0;
					vel.angular.z = v1_angular;
					cout<<"前"<<endl;
				}
			}
			else if(maxArea > frln.cols * frln.rows * 0.4) //目标区域距离机器人过近，控制机器人向后运动
			{
				vel.linear.x = -0.1;
				vel.linear.y = 0;
				vel.linear.z = 0;
				vel.angular.x = 0;
				vel.angular.y = 0;
				vel.angular.z = 0;
				cout<<"后"<<endl;
			}
			else //目标区域距离机器人在合适位置，机器人保持不动
			{
				if(center.x < frln.cols/2 && center.x > frln.cols/2 - 100)
				{
					vel.linear.x = 0;
					vel.linear.y = 0;
					vel.linear.z = 0;
					vel.angular.x = 0;
					vel.angular.y = 0;
					vel.angular.z = 0;
					cout<<"停"<<endl;
				}
				else
				{
					vel.linear.x = 0;
					vel.linear.y = 0;
					vel.linear.z = 0;
					vel.angular.x = 0;
					vel.angular.y = 0;
					vel.angular.z = v1_angular;
				}
			}
		}
		pub.publish(vel); //发布速度
		ros::spinOnce();
		waitKey(5);
	}
	waitKey(0);
	return 0;
}
