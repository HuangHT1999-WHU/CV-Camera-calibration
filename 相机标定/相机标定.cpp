// 相机标定.cpp : 定义控制台应用程序的入口点。
//

#include "stdafx.h"
#include <iostream> 
#include <fstream>
#include <opencv2/core/core.hpp>   
#include <opencv2/highgui/highgui.hpp>  
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#define BOARD_WIDTH 22   //实际测量得到的标定板上每个棋盘格width的大小
#define BOARD_HEIGHT 23  //实际测量得到的标定板上每个棋盘格height的大小

using namespace cv;
using namespace std;

 void image_format()
{
	//图片格式转化
	cout<<"开始执行图片格式转化"<<endl;
	ifstream fin_for_jpg("image_contents_for_jpg.txt"); //标定所用图像文件的jpg文件的路径，需要在该文件下根据需要修改image_contents_for_jpg.txt
	string filename_jpg;
	int image_count_jpg=0;
	while (getline(fin_for_jpg,filename_jpg))
	{
		image_count_jpg++;	
		Mat M=imread(filename_jpg);   // 读入图片 
		if(M.empty())     // 判断文件是否正常打开  
		 {
			 fprintf(stderr, "Can not load image %s\n", filename_jpg);
			 waitKey(6000);  // 等待6000 ms后窗口自动关闭   
			return;
		  }
		//需要的话打开可以查看jpg图片信息
		//cout<<M.channels()<<endl;
		//cout<<M.rows<<endl;
		//cout<<M.cols<<endl;
		//cout<<M.type();

	    //imshow("image",M);  // 显示图片 
		char buffer[999];
		char tail[]=".bmp";
		sprintf(buffer, "%s%d%s", "hongtian_huang_", image_count_jpg,tail);
		imwrite(buffer,M); // 存为bmp格式图片
	}
	cout<<"图片格式转化完成"<<endl;
	return;
}

int main() 
{
	//初次使用请使用此函数转化图片格式：jpg to bmp，已转化过图片格式请注释词句提高效率
	//image_format();

	//正式开始
	ifstream fin("image_contents.txt"); //标定所用图像文件的bmp格式的路径，需要在该文件下根据需要修改image_contents.txt
	ofstream fout("caliberation_result.txt");  //保存标定结果的文件
	//读取每一幅图像，从中提取出角点，然后对角点进行亚像素精确化	
	//cout<<"开始提取角点………………"<<endl;
	int image_count=0;  //图像数量计数器
	Size image_size;  //图像的尺寸
	Size board_size = Size(7,9);    // 标定板上每行、列的角点数，width=7,height=9
	vector<Point2f> image_points_temp;  //暂存每幅图像上检测到的角点的容器
	vector<vector<Point2f>> image_points; //保存检测到的所有角点的容器
	string filename;
	int count= -1 ;//用于存储角点个数。
	while (getline(fin,filename))
	{
		image_count++;		
		// 用于观察检验输出
		cout<<"第"<<image_count<<"张图片开始处理"<<endl;		
		Mat image=imread(filename);
		if(image.empty()){     
			// 判断文件是否正常打开  
			cout<<"读取此文件出错，请检查图片是否存在"<<endl;
			system("pause");
			return -1;
		}
		image_size.width = image.cols;
		image_size.height =image.rows;			
		cout<<"image_width_"<<image_count<<"= "<<image_size.width<<endl;
		cout<<"image_height_"<<image_count<<"= "<<image_size.height<<endl;
		//提取角点
		if (0 == findChessboardCorners(image,board_size,image_points_temp))
		{			
			cout<<"无法找到角点，请重试此图片!"<<endl; //找不到角点
			system("pause");
			exit(1);
		} 
		else 
		{
			Mat image_gray;
			cvtColor(image,image_gray,CV_RGB2GRAY);//将彩色图像转化为灰度图像
			//亚像素精确化
			find4QuadCornerSubpix(image_gray,image_points_temp,Size(5,5)); //对粗提取的角点进行精确化
			//Size(5,5)为角点搜索窗口的尺寸
			image_points.push_back(image_points_temp);  //保存亚像素角点
		}
	}
	fin.close();
	fin.clear();
	cout<<"角点提取完成，即将进行相机标定"<<endl;
 
	//相机标定
	//棋盘三维信息
	Size square_size = Size(BOARD_WIDTH,BOARD_HEIGHT);  //定义实际测量得到的标定板上每个棋盘格的大小(width,height)
	vector<vector<Point3f>> object_points; //用以保存标定板上角点的三维坐标
	/*内外参数*/
	Mat cameraMatrix=Mat(3,3,CV_32FC1,Scalar::all(0)); //相机内参数矩阵
	Mat distCoeffs=Mat(1,5,CV_32FC1,Scalar::all(0)); //相机的5个畸变系数：k1,k2,p1,p2,k3
	vector<Mat> tvecsMat;  //每幅图像的旋转向量
	vector<Mat> rvecsMat; //每幅图像的平移向量

	vector<int> point_counts;  // 定义一个容器储存每幅图像中角点的数量
	//初始化每幅图像中的完整的标定板的角点数量
	for (int i=0;i<image_count;i++){
		point_counts.push_back(board_size.width*board_size.height);//将每幅图像中的角点数量依次存入容器
	}

	//初始化标定板上角点的三维坐标
	for (int t=0;t<image_count;t++) {
		//循环图片
		vector<Point3f> PointSet_temp;//用以暂时保存每一张标定板上角点的三维坐标
		for (int i=0;i<board_size.height;i++) {
			//循环行
			for (int j=0;j<board_size.width;j++) {
				//循环列
				Point3f realPoint;//真实点，含有x,y,z三个坐标
				realPoint.x = i*square_size.width;//行坐标
				realPoint.y = j*square_size.height;//列坐标
				realPoint.z = 0;//假设标定板放在世界坐标系中z=0的平面上
				PointSet_temp.push_back(realPoint);//逐个存入一张图中所有的角点
			}
		}
		object_points.push_back(PointSet_temp);//将一张图中所有的角点的容器中存入object_points
	}

	//开始进行相机标定
	calibrateCamera(object_points,image_points,image_size,cameraMatrix,distCoeffs,rvecsMat,tvecsMat,CV_CALIB_RATIONAL_MODEL);
	cout<<"标定完成"<<endl;

	//对标定结果进行评价
	cout<<"开始评价标定结果"<<endl;
	double error_total = 0.0; //所有图像的平均误差的总和
	double error_avarage_per = 0.0; //每幅图像的平均误差
	vector<Point2f> image_points_new; //保存重新投影计算得到的新的投影点
	cout<<"每幅图像的标定误差："<<endl;
	fout<<"每幅图像的标定误差：\n";
	for (int i=0;i<image_count;i++)
	{
		vector<Point3f> PointSet_temp_s=object_points[i];//将第i幅图像中保存的标定板上角点的三维坐标转存到暂存容器PointSet_temp_s中
		//通过得到的相机内外参数，对空间的三维点进行重新投影计算，得到新的投影点
		projectPoints(PointSet_temp_s,rvecsMat[i],tvecsMat[i],cameraMatrix,distCoeffs,image_points_new);

		//以下开始计算新的投影点和旧的投影点之间的误差
		vector<Point2f> Image_points_temp_s = image_points[i];//将检测到的角点ImagePoint_temp存入暂存容器Image_points_temp_s中

		//分别为新旧投影点定义一个Mat类
		Mat Image_points_temp_s_Mat = Mat(1,Image_points_temp_s.size(),CV_32FC2);
		Mat image_points_new_Mat = Mat(1,image_points_new.size(), CV_32FC2);

		for (int j = 0 ; j<Image_points_temp_s.size(); j++)
		{
			image_points_new_Mat.at<Vec2f>(0,j) = Vec2f(image_points_new[j].x, image_points_new[j].y);
			Image_points_temp_s_Mat.at<Vec2f>(0,j) = Vec2f(Image_points_temp_s[j].x, Image_points_temp_s[j].y);
		}

		error_avarage_per = norm(image_points_new_Mat, Image_points_temp_s_Mat, CV_L2);//求每幅图像的范数
		/* 此处的第三个参数可以取：
		CV_C   1  先求解数组1和数组2对应元素的差值，然后对所有差值去绝对值，最后返回所有绝对值中最大的那个数作为函数的返回值
		CV_L1  2  先求解数组1和数组2对应元素的差值，然后对所有差值去绝对值，对这些绝对值进行累加求和
		CV_L2  4  先求解数组1和数组2对应元素的差值，然后求解各个差值平方的累加和
		*/
		error_avarage_per /= point_counts[i];
		error_total += error_avarage_per;
		std::cout<<"第"<<i+1<<"幅图像的平均误差："<<error_avarage_per<<"像素"<<endl;   
		fout<<"第"<<i+1<<"幅图像的平均误差："<<error_avarage_per<<"像素"<<endl;//将单幅图像的平均误差输出到结果文件中   
	}   
	cout<<"总体平均误差："<<error_total/image_count<<"像素"<<endl;   
	fout<<"总体平均误差："<<error_total/image_count<<"像素"<<endl<<endl;   
	cout<<"评价完成，开始保存定标结果"<<endl;  

	//保存定标结果   
	Mat rotation_matrix = Mat(3,3,CV_32FC1, Scalar::all(0)); //保存每幅图像的旋转矩阵
	fout<<"相机内参数矩阵："<<endl;   
	fout<<cameraMatrix<<endl<<endl;   
	fout<<"畸变系数："<<endl;   
	fout<<distCoeffs<<endl<<endl;
	
	fout<<"--------------------------------------------------------------------------------------------------"<<endl;
	fout<<"开始输出每幅图像的旋转向量、旋转矩阵和平移向量:"<<endl<<endl;
	for (int i=0; i<image_count; i++) 
	{ 
		fout<<"第"<<i+1<<"幅图像的旋转向量："<<endl;   
		fout<<tvecsMat[i]<<endl; 
		/* 将旋转向量转换为相对应的旋转矩阵 */   
		Rodrigues(tvecsMat[i],rotation_matrix);
		fout<<"第"<<i+1<<"幅图像的旋转矩阵："<<endl;
		fout<<rotation_matrix<<endl;
		fout<<"第"<<i+1<<"幅图像的平移向量："<<endl;
		fout<<rvecsMat[i]<<endl<<endl;
	}   
	cout<<"标定结果保存完毕，开始输出图像"<<endl; 
	ifstream fin_out("image_contents.txt"); //标定所用图像文件的路径
	string filename_out;
	for(int i=0;i<image_count;i++){
		getline(fin_out,filename_out);
		Mat image_RGBforOut_real=imread(filename_out);
		Mat image_RGBforOut=image_RGBforOut_real.clone();
		undistort(image_RGBforOut_real,image_RGBforOut,cameraMatrix,distCoeffs);//畸变矫正
		//cvtColor(image_gray,image_RGBforOut,CV_GRAY2RGB);//将彩色图像转化为灰度图像
		
		vector<Point2f> image_points_temp_forOut=image_points[i];  //暂存每幅图像上检测到的角点的容器
		
		vector<Point3f> image_points3f_temp_forOut_up;//z=0平面上方22.5处的对应的角点位置容器Point3f类型
		vector<Point2f> image_points_temp_forOut_up;//z=0平面上方22.5处的对应的角点位置容器Point2f类型

		vector<Point2f> image_points_temp_forOut_undis;  //暂存每幅图像上检测到的畸变处理后的角点的容器
		undistortPoints( image_points_temp_forOut, image_points_temp_forOut_undis,cameraMatrix,distCoeffs,noArray(),cameraMatrix);//角点做畸变处理
		drawChessboardCorners(image_RGBforOut,board_size,image_points_temp_forOut_undis,true); //用于在图片中标记角点
		//patternWasFound=true时，依次连接各个内角点
		//patternWasFound=false时，以（红色）圆圈标记处角点位置
		
		for (int i=0;i<board_size.height;i++) {
			//循环行
			for (int j=0;j<board_size.width;j++) {
				//循环列
				Point3f realPoint;//真实点，含有x,y,z三个坐标
				realPoint.x = i*square_size.width;//行坐标
				realPoint.y = j*square_size.height;//列坐标
				realPoint.z = 22.5;//假设标定板放在世界坐标系中z=0的平面上
				image_points3f_temp_forOut_up.push_back(realPoint);//逐个存入一张图中所有的角点
			}
		}
		//for (int t=0;t<board_size.height*board_size.width;t++) {
		//	//循环行
		//	Point3f realPoint;//真实点，含有x,y,z三个坐标
		//	realPoint.x = image_points_temp_forOut_undis[t].x;//行坐标
		//	realPoint.y = image_points_temp_forOut_undis[t].y;//列坐标
		//	realPoint.z = 22.5;//世界坐标系中z=22.5的平面上
		//	image_points3f_temp_forOut_up.push_back(realPoint);//逐个存入一张图中所有的角点
		//}

		//插入一个用来绘制Z轴的点(Z轴极大值点）
		Point3f Z_max=image_points3f_temp_forOut_up[0];
		Z_max.z = 35;//假设标定板放在世界坐标系中z=35的平面上
		image_points3f_temp_forOut_up.push_back(Z_max);//存入对应图像的角点容器

		for(int j=0;j<board_size.width*board_size.height+1;j++){
			projectPoints(image_points3f_temp_forOut_up, rvecsMat[i], tvecsMat[i],cameraMatrix, distCoeffs, image_points_temp_forOut_up);
		}

		//开始作图
		//作出横轴
		Point2f zero_point = image_points_temp_forOut_undis[0];
		Point2f width = image_points_temp_forOut_undis[3]; 
		line(image_RGBforOut, zero_point,width, cv::Scalar(255, 0, 0),6);

		//作出纵轴
		Point2f height = image_points_temp_forOut_undis[0+3*board_size.width];
		line(image_RGBforOut, zero_point, height, cv::Scalar(0, 255, 0),6);

		//作出Z轴
		Point2f Z = image_points_temp_forOut_up[board_size.width*board_size.height];
		line(image_RGBforOut, zero_point, Z, cv::Scalar(0, 0, 255),6);
		
		//开始绘制3Dbox
		//定义8个框点(从零点开始逆时针分别为1234，再到零点的上方开始5678（零点已定义）
		Point2f point2=image_points_temp_forOut_undis[2];
		Point2f point3=image_points_temp_forOut_undis[2+2*board_size.width];
		Point2f point4=image_points_temp_forOut_undis[0+2*board_size.width];
		Point2f point5=image_points_temp_forOut_up[0];
		Point2f point6=image_points_temp_forOut_up[2];
		Point2f point7=image_points_temp_forOut_up[2+2*board_size.width];
		Point2f point8=image_points_temp_forOut_up[0+2*board_size.width];

		//连接点与点
		line(image_RGBforOut, zero_point, point2, cv::Scalar(255, 0, 255),7);
		line(image_RGBforOut, point2, point3, cv::Scalar(255, 0, 255),7);
		line(image_RGBforOut, point3, point4, cv::Scalar(255, 0, 255),7);
		line(image_RGBforOut, zero_point, point4, cv::Scalar(255, 0, 255),7);
		line(image_RGBforOut, zero_point, point5, cv::Scalar(255, 0, 255),7);
		line(image_RGBforOut, point2, point6, cv::Scalar(255, 0, 255),7);
		line(image_RGBforOut, point3, point7, cv::Scalar(255, 0, 255),7);
		line(image_RGBforOut, point4, point8, cv::Scalar(255, 0, 255),7);
		line(image_RGBforOut, point5, point6, cv::Scalar(255, 0, 255),7);
		line(image_RGBforOut, point6, point7, cv::Scalar(255, 0, 255),7);
		line(image_RGBforOut, point7, point8, cv::Scalar(255, 0, 255),7);
		line(image_RGBforOut, point8, point5, cv::Scalar(255, 0, 255),7);

		//标记出原点位置
		circle(image_RGBforOut, zero_point, 9, cv::Scalar(255, 255, 255));//在图像中画出特征点，3是圆的半径 
		
		cout<<"绘图完成，开始输出图片"<<i+1<<endl;

		//输出为bmp图片，格式为：image+图片号+.bmp
		char buffer[999];
		char tail[]=".bmp";
		sprintf(buffer, "%s%d%s", "image_result", i+1,tail);
		imwrite(buffer,image_RGBforOut); // 存为bmp格式图片
	}
	cout<<"输出图片结束"<<endl;
	cout<<"运行结束，请查看结果"<<endl;
	system("pause");	
	return 0;
}
