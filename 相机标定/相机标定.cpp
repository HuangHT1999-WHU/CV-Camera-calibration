// ����궨.cpp : �������̨Ӧ�ó������ڵ㡣
//

#include "stdafx.h"
#include <iostream> 
#include <fstream>
#include <opencv2/core/core.hpp>   
#include <opencv2/highgui/highgui.hpp>  
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#define BOARD_WIDTH 22   //ʵ�ʲ����õ��ı궨����ÿ�����̸�width�Ĵ�С
#define BOARD_HEIGHT 23  //ʵ�ʲ����õ��ı궨����ÿ�����̸�height�Ĵ�С

using namespace cv;
using namespace std;

 void image_format()
{
	//ͼƬ��ʽת��
	cout<<"��ʼִ��ͼƬ��ʽת��"<<endl;
	ifstream fin_for_jpg("image_contents_for_jpg.txt"); //�궨����ͼ���ļ���jpg�ļ���·������Ҫ�ڸ��ļ��¸�����Ҫ�޸�image_contents_for_jpg.txt
	string filename_jpg;
	int image_count_jpg=0;
	while (getline(fin_for_jpg,filename_jpg))
	{
		image_count_jpg++;	
		Mat M=imread(filename_jpg);   // ����ͼƬ 
		if(M.empty())     // �ж��ļ��Ƿ�������  
		 {
			 fprintf(stderr, "Can not load image %s\n", filename_jpg);
			 waitKey(6000);  // �ȴ�6000 ms�󴰿��Զ��ر�   
			return;
		  }
		//��Ҫ�Ļ��򿪿��Բ鿴jpgͼƬ��Ϣ
		//cout<<M.channels()<<endl;
		//cout<<M.rows<<endl;
		//cout<<M.cols<<endl;
		//cout<<M.type();

	    //imshow("image",M);  // ��ʾͼƬ 
		char buffer[999];
		char tail[]=".bmp";
		sprintf(buffer, "%s%d%s", "hongtian_huang_", image_count_jpg,tail);
		imwrite(buffer,M); // ��Ϊbmp��ʽͼƬ
	}
	cout<<"ͼƬ��ʽת�����"<<endl;
	return;
}

int main() 
{
	//����ʹ����ʹ�ô˺���ת��ͼƬ��ʽ��jpg to bmp����ת����ͼƬ��ʽ��ע�ʹʾ����Ч��
	//image_format();

	//��ʽ��ʼ
	ifstream fin("image_contents.txt"); //�궨����ͼ���ļ���bmp��ʽ��·������Ҫ�ڸ��ļ��¸�����Ҫ�޸�image_contents.txt
	ofstream fout("caliberation_result.txt");  //����궨������ļ�
	//��ȡÿһ��ͼ�񣬴�����ȡ���ǵ㣬Ȼ��Խǵ���������ؾ�ȷ��	
	//cout<<"��ʼ��ȡ�ǵ㡭����������"<<endl;
	int image_count=0;  //ͼ������������
	Size image_size;  //ͼ��ĳߴ�
	Size board_size = Size(7,9);    // �궨����ÿ�С��еĽǵ�����width=7,height=9
	vector<Point2f> image_points_temp;  //�ݴ�ÿ��ͼ���ϼ�⵽�Ľǵ������
	vector<vector<Point2f>> image_points; //�����⵽�����нǵ������
	string filename;
	int count= -1 ;//���ڴ洢�ǵ������
	while (getline(fin,filename))
	{
		image_count++;		
		// ���ڹ۲�������
		cout<<"��"<<image_count<<"��ͼƬ��ʼ����"<<endl;		
		Mat image=imread(filename);
		if(image.empty()){     
			// �ж��ļ��Ƿ�������  
			cout<<"��ȡ���ļ���������ͼƬ�Ƿ����"<<endl;
			system("pause");
			return -1;
		}
		image_size.width = image.cols;
		image_size.height =image.rows;			
		cout<<"image_width_"<<image_count<<"= "<<image_size.width<<endl;
		cout<<"image_height_"<<image_count<<"= "<<image_size.height<<endl;
		//��ȡ�ǵ�
		if (0 == findChessboardCorners(image,board_size,image_points_temp))
		{			
			cout<<"�޷��ҵ��ǵ㣬�����Դ�ͼƬ!"<<endl; //�Ҳ����ǵ�
			system("pause");
			exit(1);
		} 
		else 
		{
			Mat image_gray;
			cvtColor(image,image_gray,CV_RGB2GRAY);//����ɫͼ��ת��Ϊ�Ҷ�ͼ��
			//�����ؾ�ȷ��
			find4QuadCornerSubpix(image_gray,image_points_temp,Size(5,5)); //�Դ���ȡ�Ľǵ���о�ȷ��
			//Size(5,5)Ϊ�ǵ��������ڵĳߴ�
			image_points.push_back(image_points_temp);  //���������ؽǵ�
		}
	}
	fin.close();
	fin.clear();
	cout<<"�ǵ���ȡ��ɣ�������������궨"<<endl;
 
	//����궨
	//������ά��Ϣ
	Size square_size = Size(BOARD_WIDTH,BOARD_HEIGHT);  //����ʵ�ʲ����õ��ı궨����ÿ�����̸�Ĵ�С(width,height)
	vector<vector<Point3f>> object_points; //���Ա���궨���Ͻǵ����ά����
	/*�������*/
	Mat cameraMatrix=Mat(3,3,CV_32FC1,Scalar::all(0)); //����ڲ�������
	Mat distCoeffs=Mat(1,5,CV_32FC1,Scalar::all(0)); //�����5������ϵ����k1,k2,p1,p2,k3
	vector<Mat> tvecsMat;  //ÿ��ͼ�����ת����
	vector<Mat> rvecsMat; //ÿ��ͼ���ƽ������

	vector<int> point_counts;  // ����һ����������ÿ��ͼ���нǵ������
	//��ʼ��ÿ��ͼ���е������ı궨��Ľǵ�����
	for (int i=0;i<image_count;i++){
		point_counts.push_back(board_size.width*board_size.height);//��ÿ��ͼ���еĽǵ��������δ�������
	}

	//��ʼ���궨���Ͻǵ����ά����
	for (int t=0;t<image_count;t++) {
		//ѭ��ͼƬ
		vector<Point3f> PointSet_temp;//������ʱ����ÿһ�ű궨���Ͻǵ����ά����
		for (int i=0;i<board_size.height;i++) {
			//ѭ����
			for (int j=0;j<board_size.width;j++) {
				//ѭ����
				Point3f realPoint;//��ʵ�㣬����x,y,z��������
				realPoint.x = i*square_size.width;//������
				realPoint.y = j*square_size.height;//������
				realPoint.z = 0;//����궨�������������ϵ��z=0��ƽ����
				PointSet_temp.push_back(realPoint);//�������һ��ͼ�����еĽǵ�
			}
		}
		object_points.push_back(PointSet_temp);//��һ��ͼ�����еĽǵ�������д���object_points
	}

	//��ʼ��������궨
	calibrateCamera(object_points,image_points,image_size,cameraMatrix,distCoeffs,rvecsMat,tvecsMat,CV_CALIB_RATIONAL_MODEL);
	cout<<"�궨���"<<endl;

	//�Ա궨�����������
	cout<<"��ʼ���۱궨���"<<endl;
	double error_total = 0.0; //����ͼ���ƽ�������ܺ�
	double error_avarage_per = 0.0; //ÿ��ͼ���ƽ�����
	vector<Point2f> image_points_new; //��������ͶӰ����õ����µ�ͶӰ��
	cout<<"ÿ��ͼ��ı궨��"<<endl;
	fout<<"ÿ��ͼ��ı궨��\n";
	for (int i=0;i<image_count;i++)
	{
		vector<Point3f> PointSet_temp_s=object_points[i];//����i��ͼ���б���ı궨���Ͻǵ����ά����ת�浽�ݴ�����PointSet_temp_s��
		//ͨ���õ����������������Կռ����ά���������ͶӰ���㣬�õ��µ�ͶӰ��
		projectPoints(PointSet_temp_s,rvecsMat[i],tvecsMat[i],cameraMatrix,distCoeffs,image_points_new);

		//���¿�ʼ�����µ�ͶӰ��;ɵ�ͶӰ��֮������
		vector<Point2f> Image_points_temp_s = image_points[i];//����⵽�Ľǵ�ImagePoint_temp�����ݴ�����Image_points_temp_s��

		//�ֱ�Ϊ�¾�ͶӰ�㶨��һ��Mat��
		Mat Image_points_temp_s_Mat = Mat(1,Image_points_temp_s.size(),CV_32FC2);
		Mat image_points_new_Mat = Mat(1,image_points_new.size(), CV_32FC2);

		for (int j = 0 ; j<Image_points_temp_s.size(); j++)
		{
			image_points_new_Mat.at<Vec2f>(0,j) = Vec2f(image_points_new[j].x, image_points_new[j].y);
			Image_points_temp_s_Mat.at<Vec2f>(0,j) = Vec2f(Image_points_temp_s[j].x, Image_points_temp_s[j].y);
		}

		error_avarage_per = norm(image_points_new_Mat, Image_points_temp_s_Mat, CV_L2);//��ÿ��ͼ��ķ���
		/* �˴��ĵ�������������ȡ��
		CV_C   1  ���������1������2��ӦԪ�صĲ�ֵ��Ȼ������в�ֵȥ����ֵ����󷵻����о���ֵ�������Ǹ�����Ϊ�����ķ���ֵ
		CV_L1  2  ���������1������2��ӦԪ�صĲ�ֵ��Ȼ������в�ֵȥ����ֵ������Щ����ֵ�����ۼ����
		CV_L2  4  ���������1������2��ӦԪ�صĲ�ֵ��Ȼ����������ֵƽ�����ۼӺ�
		*/
		error_avarage_per /= point_counts[i];
		error_total += error_avarage_per;
		std::cout<<"��"<<i+1<<"��ͼ���ƽ����"<<error_avarage_per<<"����"<<endl;   
		fout<<"��"<<i+1<<"��ͼ���ƽ����"<<error_avarage_per<<"����"<<endl;//������ͼ���ƽ��������������ļ���   
	}   
	cout<<"����ƽ����"<<error_total/image_count<<"����"<<endl;   
	fout<<"����ƽ����"<<error_total/image_count<<"����"<<endl<<endl;   
	cout<<"������ɣ���ʼ���涨����"<<endl;  

	//���涨����   
	Mat rotation_matrix = Mat(3,3,CV_32FC1, Scalar::all(0)); //����ÿ��ͼ�����ת����
	fout<<"����ڲ�������"<<endl;   
	fout<<cameraMatrix<<endl<<endl;   
	fout<<"����ϵ����"<<endl;   
	fout<<distCoeffs<<endl<<endl;
	
	fout<<"--------------------------------------------------------------------------------------------------"<<endl;
	fout<<"��ʼ���ÿ��ͼ�����ת��������ת�����ƽ������:"<<endl<<endl;
	for (int i=0; i<image_count; i++) 
	{ 
		fout<<"��"<<i+1<<"��ͼ�����ת������"<<endl;   
		fout<<tvecsMat[i]<<endl; 
		/* ����ת����ת��Ϊ���Ӧ����ת���� */   
		Rodrigues(tvecsMat[i],rotation_matrix);
		fout<<"��"<<i+1<<"��ͼ�����ת����"<<endl;
		fout<<rotation_matrix<<endl;
		fout<<"��"<<i+1<<"��ͼ���ƽ��������"<<endl;
		fout<<rvecsMat[i]<<endl<<endl;
	}   
	cout<<"�궨���������ϣ���ʼ���ͼ��"<<endl; 
	ifstream fin_out("image_contents.txt"); //�궨����ͼ���ļ���·��
	string filename_out;
	for(int i=0;i<image_count;i++){
		getline(fin_out,filename_out);
		Mat image_RGBforOut_real=imread(filename_out);
		Mat image_RGBforOut=image_RGBforOut_real.clone();
		undistort(image_RGBforOut_real,image_RGBforOut,cameraMatrix,distCoeffs);//�������
		//cvtColor(image_gray,image_RGBforOut,CV_GRAY2RGB);//����ɫͼ��ת��Ϊ�Ҷ�ͼ��
		
		vector<Point2f> image_points_temp_forOut=image_points[i];  //�ݴ�ÿ��ͼ���ϼ�⵽�Ľǵ������
		
		vector<Point3f> image_points3f_temp_forOut_up;//z=0ƽ���Ϸ�22.5���Ķ�Ӧ�Ľǵ�λ������Point3f����
		vector<Point2f> image_points_temp_forOut_up;//z=0ƽ���Ϸ�22.5���Ķ�Ӧ�Ľǵ�λ������Point2f����

		vector<Point2f> image_points_temp_forOut_undis;  //�ݴ�ÿ��ͼ���ϼ�⵽�Ļ��䴦���Ľǵ������
		undistortPoints( image_points_temp_forOut, image_points_temp_forOut_undis,cameraMatrix,distCoeffs,noArray(),cameraMatrix);//�ǵ������䴦��
		drawChessboardCorners(image_RGBforOut,board_size,image_points_temp_forOut_undis,true); //������ͼƬ�б�ǽǵ�
		//patternWasFound=trueʱ���������Ӹ����ڽǵ�
		//patternWasFound=falseʱ���ԣ���ɫ��ԲȦ��Ǵ��ǵ�λ��
		
		for (int i=0;i<board_size.height;i++) {
			//ѭ����
			for (int j=0;j<board_size.width;j++) {
				//ѭ����
				Point3f realPoint;//��ʵ�㣬����x,y,z��������
				realPoint.x = i*square_size.width;//������
				realPoint.y = j*square_size.height;//������
				realPoint.z = 22.5;//����궨�������������ϵ��z=0��ƽ����
				image_points3f_temp_forOut_up.push_back(realPoint);//�������һ��ͼ�����еĽǵ�
			}
		}
		//for (int t=0;t<board_size.height*board_size.width;t++) {
		//	//ѭ����
		//	Point3f realPoint;//��ʵ�㣬����x,y,z��������
		//	realPoint.x = image_points_temp_forOut_undis[t].x;//������
		//	realPoint.y = image_points_temp_forOut_undis[t].y;//������
		//	realPoint.z = 22.5;//��������ϵ��z=22.5��ƽ����
		//	image_points3f_temp_forOut_up.push_back(realPoint);//�������һ��ͼ�����еĽǵ�
		//}

		//����һ����������Z��ĵ�(Z�Ἣ��ֵ�㣩
		Point3f Z_max=image_points3f_temp_forOut_up[0];
		Z_max.z = 35;//����궨�������������ϵ��z=35��ƽ����
		image_points3f_temp_forOut_up.push_back(Z_max);//�����Ӧͼ��Ľǵ�����

		for(int j=0;j<board_size.width*board_size.height+1;j++){
			projectPoints(image_points3f_temp_forOut_up, rvecsMat[i], tvecsMat[i],cameraMatrix, distCoeffs, image_points_temp_forOut_up);
		}

		//��ʼ��ͼ
		//��������
		Point2f zero_point = image_points_temp_forOut_undis[0];
		Point2f width = image_points_temp_forOut_undis[3]; 
		line(image_RGBforOut, zero_point,width, cv::Scalar(255, 0, 0),6);

		//��������
		Point2f height = image_points_temp_forOut_undis[0+3*board_size.width];
		line(image_RGBforOut, zero_point, height, cv::Scalar(0, 255, 0),6);

		//����Z��
		Point2f Z = image_points_temp_forOut_up[board_size.width*board_size.height];
		line(image_RGBforOut, zero_point, Z, cv::Scalar(0, 0, 255),6);
		
		//��ʼ����3Dbox
		//����8�����(����㿪ʼ��ʱ��ֱ�Ϊ1234���ٵ������Ϸ���ʼ5678������Ѷ��壩
		Point2f point2=image_points_temp_forOut_undis[2];
		Point2f point3=image_points_temp_forOut_undis[2+2*board_size.width];
		Point2f point4=image_points_temp_forOut_undis[0+2*board_size.width];
		Point2f point5=image_points_temp_forOut_up[0];
		Point2f point6=image_points_temp_forOut_up[2];
		Point2f point7=image_points_temp_forOut_up[2+2*board_size.width];
		Point2f point8=image_points_temp_forOut_up[0+2*board_size.width];

		//���ӵ����
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

		//��ǳ�ԭ��λ��
		circle(image_RGBforOut, zero_point, 9, cv::Scalar(255, 255, 255));//��ͼ���л��������㣬3��Բ�İ뾶 
		
		cout<<"��ͼ��ɣ���ʼ���ͼƬ"<<i+1<<endl;

		//���ΪbmpͼƬ����ʽΪ��image+ͼƬ��+.bmp
		char buffer[999];
		char tail[]=".bmp";
		sprintf(buffer, "%s%d%s", "image_result", i+1,tail);
		imwrite(buffer,image_RGBforOut); // ��Ϊbmp��ʽͼƬ
	}
	cout<<"���ͼƬ����"<<endl;
	cout<<"���н�������鿴���"<<endl;
	system("pause");	
	return 0;
}
