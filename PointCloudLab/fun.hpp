#pragma once
#include <iostream>
#include <stdlib.h>
#include <vector>
#include <string>
#include <time.h>

#include <boost/thread/thread.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/io/vtk_lib_io.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/filters/bilateral.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/plane_clipper3D.h>
#include <pcl/filters/box_clipper3D.h>
#include <pcl/filters/crop_box.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/search/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <pcl/segmentation/progressive_morphological_filter.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/gicp.h>
#include <pcl/registration/ia_ransac.h>

#include <pcl/surface/mls.h>
#include <pcl/surface/gp3.h>

#include <pcl/surface/poisson.h>
#include <pcl/features/normal_3d.h>

#include <pcl/features/fpfh.h>
#include <pcl/features/boundary.h>
#include <pcl/features/eigen.h>
#include <pcl/features/feature.h>


#include <vtkPLYReader.h>
#include <vtkSTLReader.h>
#include <vtkTriangleFilter.h>
#include <vtkSmartPointer.h>
#include <vtkMassProperties.h>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>

#include <opencv2/highgui/highgui.hpp>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/ransac.h> 
#include <pcl/sample_consensus/sac_model_registration.h>

#include <pcl/common/transforms.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>

#include <pcl/range_image/range_image.h>

//#include <BGL.h>

//读取pcd点云文件
pcl::PointCloud<pcl::PointXYZ>::Ptr  pcdload()
{
	std::string xyz;
	std::cout << "输入pcd文件地址及名称" << std::endl;
	std::cin >> xyz;
	pcl::PointCloud<pcl::PointXYZ>::Ptr pcdcloud(new pcl::PointCloud<pcl::PointXYZ>);

	if (pcl::io::loadPCDFile<pcl::PointXYZ>(xyz, *pcdcloud) == -1)//*打开点云文件  xyz为pcd文件名
	{
		PCL_ERROR("Couldn't read file\n");
		exit(0);
	}
	return(pcdcloud);
}

//读取ply点云文件
pcl::PointCloud<pcl::PointXYZ>::Ptr  plyload()
{
	std::string xyz;
	std::cout << "输入ply文件地址及名称" << std::endl;
	std::cin >> xyz;
	pcl::PointCloud<pcl::PointXYZ>::Ptr plycloud(new pcl::PointCloud<pcl::PointXYZ>);

	if (pcl::io::loadPLYFile<pcl::PointXYZ>(xyz, *plycloud) == -1)//*打开点云文件  xyz为ply文件名
	{
		PCL_ERROR("Couldn't read file\n");
		exit(0);
	}
	return(plycloud);
}


//读取obj点云文件 并转成PointXYZ类型
pcl::PointCloud<pcl::PointXYZ>::Ptr  objload()
{
	std::string xyz;
	std::cout << "输入obj文件地址及名称" << std::endl;
	std::cin >> xyz;
	pcl::PolygonMesh mesh;
	;

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

	if (pcl::io::loadPolygonFileOBJ(xyz, mesh) == -1)//*打开点云文件  xyz为ply文件名
	{
		PCL_ERROR("Couldn't read file\n");
		exit(0);
	}
	pcl::fromPCLPointCloud2(mesh.cloud, *cloud);
	//pcl::io::savePCDFileASCII("xxxx.pcd", *cloud);     //将PointXYZ存为pcd ASCII码形式  “xxxx为地址和名称”
	return(cloud);
}

//保存obj点云文件 
void  objsave(pcl::PolygonMesh mesh,std::string path)
{
	
	pcl::io::saveOBJFile(path, mesh);
	
}



//将pcd文件转换为ply文件
void PCDtoPLYconvertor(std::string input_filename, std::string output_filename)
{
	pcl::PCLPointCloud2 cloud;
	if (pcl::io::loadPCDFile(input_filename, cloud) < 0)
	{
		cout << "Error: cannot load the PCD file!!!" << endl;
		exit(0);
	}
	pcl::PLYWriter writer;
	writer.write(output_filename, cloud, Eigen::Vector4f::Zero(), Eigen::Quaternionf::Identity(), false, true); //ply为ASCII 倒数第二个参数决定

}

//将ply文件转换为pcd文件
void PLYtoPCDconvertor(std::string input_filename, std::string output_filename)
{
	pcl::PCLPointCloud2 cloud;
	if (pcl::io::loadPLYFile(input_filename, cloud) < 0)
	{
		cout << "Error: cannot load the PLY file!!!" << endl;
		exit(0);
	}
	pcl::PCDWriter writer;
	writer.write(output_filename, cloud, Eigen::Vector4f::Zero(), Eigen::Quaternionf::Identity(), false); //pcd为ASCII 倒数第一个参数决定

}



//深度图转cloud 存为ply  相机参数已知,需要深度图颜色和深度信息
void depthtocloud(std::string rgbs, std::string depths)
{
	const double camera_factor = 1;
	const double camera_cx = 326.9487;
	const double camera_cy = 232.3900;
	const double camera_fx = 616.3767;
	const double camera_fy = 616.3767;

	cv::Mat rgb, depth;
	rgb = cv::imread(rgbs, 1);
	cout << "read rgb" << endl;
	depth = cv::imread(depths, -1);
	cout << "read depth" << endl;

	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
	//pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pc(new pcl::PointCloud<pcl::PointXYZRGBA>);
	// 遍历深度图
	for (int m = 0; m < depth.rows; m++)
		for (int n = 0; n < depth.cols; n++)
		{
			// 获取深度图中(m,n)处的值
			ushort d = depth.ptr<ushort>(m)[n];
			// d 可能没有值，若如此，跳过此点
			if (d == 0)
				continue;
			// d 存在值，则向点云增加一个点
			pcl::PointXYZRGBA p;

			// 计算这个点的空间坐标
			p.z = double(d) / camera_factor;
			p.x = (n - camera_cx) * p.z / camera_fx;
			p.y = (m - camera_cy) * p.z / camera_fy;

			// 从rgb图像中获取它的颜色
			// rgb是三通道的BGR格式图，所以按下面的顺序获取颜色
			p.b = rgb.ptr<uchar>(m)[n * 3];
			p.g = rgb.ptr<uchar>(m)[n * 3 + 1];
			p.r = rgb.ptr<uchar>(m)[n * 3 + 2];

			// 把p加入到点云中
			cloud->points.push_back(p);
			//cout << cloud->points.size() << endl;
		}
	cloud->height = 1;
	cloud->width = cloud->points.size();
	cout << "point cloud size = " << cloud->points.size() << endl;
	cloud->is_dense = false;

	pcl::io::savePLYFile("cloud.ply", *cloud);

	cout << "Point cloud saved." << endl;
}

//从点云文件，生成深度图像 （range image）
void pointcloud2rangeimage(pcl::RangeImage &rangeImage, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{

	cloud->width = (uint32_t)cloud->points.size();
	cloud->height = 1;                                        //设置点云对象的头信息

	// We now want to create a range image from the above point cloud, with a 1deg angular resolution
	 //angular_resolution为模拟的深度传感器的角度分辨率，即深度图像中一个像素对应的角度大小
	float angularResolution = (float)(1.0f * (M_PI / 180.0f));  //   1.0 degree in radians
	 //max_angle_width为模拟的深度传感器的水平最大采样角度，
	float maxAngleWidth = (float)(360.0f * (M_PI / 180.0f));  // 360.0 degree in radians
	//max_angle_height为模拟传感器的垂直方向最大采样角度  都转为弧度
	float maxAngleHeight = (float)(180.0f * (M_PI / 180.0f));  // 180.0 degree in radians
	 //传感器的采集位置
	Eigen::Affine3f sensorPose = (Eigen::Affine3f)Eigen::Translation3f(0.0f, 0.0f, 0.0f);
	//深度图像遵循坐标系统
	pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;
	float noiseLevel = 0.00;    //noise_level获取深度图像深度时，近邻点对查询点距离值的影响水平
	float minRange = 0.0f;     //min_range设置最小的获取距离，小于最小获取距离的位置为传感器的盲区
	int borderSize = 1;        //border_size获得深度图像的边缘的宽度

	//pcl::RangeImage rangeImage;
	rangeImage.createFromPointCloud(*cloud, angularResolution, maxAngleWidth, maxAngleHeight,
		sensorPose, coordinate_frame, noiseLevel, minRange, borderSize);

	//return rangeImage;
}

//显示1 单色显示pcl 默认白色
void pclviewer1(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::string cloud_string,
	int red = 255,
	int green = 255,
	int blue = 255,
	int point_size = 1)
{
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cloud, red, green, blue);
	viewer->setBackgroundColor(0, 0, 0);
	viewer->addPointCloud<pcl::PointXYZ>(cloud, single_color, cloud_string);                         //cloud string是显示的点云的名字
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, point_size, cloud_string);
	viewer->addCoordinateSystem(1.0);
	viewer->initCameraParameters();
	viewer->spin();     //使窗口保持
}


//染色
void colorpoints(pcl::PointCloud<pcl::PointXYZRGB>::Ptr out, pcl::PointCloud<pcl::PointXYZ>::Ptr in) {
	for (int i = 0; i < in->size(); i++) {
		pcl::PointXYZRGB colored_point;
		int red =  100+(in->points[i].x)*50;
		int green =100 + (in->points[i].y)*50;
		int blue = 100 + (in->points[i].z)*50;
		colored_point.x = in->points[i].x;
		colored_point.y = in->points[i].y;
		colored_point.z = in->points[i].z;
		colored_point.r = red;
		colored_point.g = green;
		colored_point.b = blue;
		out->push_back(colored_point);
	}

}
//显示2  显示彩色pcl XYZRGB
void pclviewer2(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, std::string cloud_string,
	int point_size = 1)
{
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
	viewer->setBackgroundColor(0, 0, 0);
	viewer->addPointCloud<pcl::PointXYZRGB>(cloud, rgb, cloud_string);                         //cloud string是显示的点云的名字
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, point_size, cloud_string);
	viewer->addCoordinateSystem(1.0);
	viewer->initCameraParameters();
	viewer->spin();     //使窗口保持
}

//一般的cloudviewer
//pcl::visualization::CloudViewer viewer1("cloud viewer");
//viewer1.showCloud(cloud3);             //窗口打开下，可以按h 获得帮助
//while (!viewer1.wasStopped())
//{

//}

//滤波1 半径滤波
pcl::PointCloud<pcl::PointXYZ>::Ptr  removal_1(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float r, int n)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_r1(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
	// build the filter
	outrem.setInputCloud(cloud);
	outrem.setRadiusSearch(r);
	outrem.setMinNeighborsInRadius(n);
	outrem.setKeepOrganized(true);                       //半径r，要求有n个邻居
	// apply filter
	outrem.filter(*cloud_r1);
	return(cloud_r1);
}

//滤波2   统计滤波 时间巨长
pcl::PointCloud<pcl::PointXYZ>::Ptr  removal_2(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, float r=50, float n=1.0, bool a=1)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_r2(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_r22(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
	sor.setInputCloud(cloud);
	sor.setMeanK(r);                //每个点要分析的邻居数设置为50，标准差乘数设置为1。r=50  n=1.0
	sor.setStddevMulThresh(n);    //这意味着所有距离查询点的平均距离均大于1标准差的所有点都将标记为异常值并删除
	sor.filter(*cloud_r2);     //时间巨长！！！！

	sor.setNegative(true);

	sor.filter(*cloud_r22);  //保存剔除掉的点
	if (a)       //a是1的时候要剩下的      a是0的时候要剔除的
	{
		return cloud_r2;
	}
	else {
		return cloud_r22;
	}
}

//滤波3 条件滤波
pcl::PointCloud<pcl::PointXYZ>::Ptr  removal_3(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, std::string xyz, float gt, float lt)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_r3(new pcl::PointCloud<pcl::PointXYZ>);
	// build the condition
	pcl::ConditionAnd<pcl::PointXYZ>::Ptr range_cond(new
		pcl::ConditionAnd<pcl::PointXYZ>());
	range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(new
		pcl::FieldComparison<pcl::PointXYZ>(xyz, pcl::ComparisonOps::GT, gt)));    //greater than
	range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(new
		pcl::FieldComparison<pcl::PointXYZ>(xyz, pcl::ComparisonOps::LT, lt)));    //lower than    // y 0-0.8  类似于裁剪
	// build the filter
	pcl::ConditionalRemoval<pcl::PointXYZ> condrem;
	condrem.setCondition(range_cond);
	condrem.setInputCloud(cloud);
	condrem.setKeepOrganized(true);
	// apply filter
	condrem.filter(*cloud_r3);
	return(cloud_r3);
}

//滤波4 体素滤波
void pclDownsize(pcl::PointCloud<pcl::PointXYZ>::Ptr in, pcl::PointCloud<pcl::PointXYZ>::Ptr out, float leaf)
{
	pcl::VoxelGrid<pcl::PointXYZ> down_filter;
	down_filter.setLeafSize(leaf, leaf, leaf);
	down_filter.setInputCloud(in);
	down_filter.filter(*out);
}


//滤波5 直通滤波
void cutcloud(pcl::PointCloud<pcl::PointXYZ>::Ptr in, std::string fieldname, double s, double e, pcl::PointCloud<pcl::PointXYZ>::Ptr out)
{
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_x_ptr(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PassThrough<pcl::PointXYZ> pass;
	pass.setInputCloud(in);
	pass.setFilterFieldName(fieldname);
	pass.setFilterLimits(s, e);
	pass.setFilterLimitsNegative(false);	// In FilterLimits [] or out
	pass.filter(*out);

}

//滤波5 投影滤波  ax+by+cz+d=0
void projecting(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr in, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr out, float a, float b, float c, float d)
{
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
	coefficients->values.resize(4);
	coefficients->values[0] = a;
	coefficients->values[1] = b;
	coefficients->values[2] = c;
	coefficients->values[3] = d;
	pcl::ProjectInliers<pcl::PointXYZRGBA> proj;
	proj.setModelType(pcl::SACMODEL_PLANE);
	proj.setInputCloud(in);
	proj.setModelCoefficients(coefficients);
	proj.filter(*out);
}

//滤波6 双边滤波 要求结构 XYZI
void Bifilter(pcl::PointCloud<pcl::PointXYZI>::Ptr in, pcl::PointCloud<pcl::PointXYZI>::Ptr out, float sigma_s, float sigma_r)
{
	pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
	pcl::BilateralFilter<pcl::PointXYZI> bf;
	bf.setInputCloud(in);
	bf.setSearchMethod(tree);
	bf.setHalfSize(sigma_s);
	bf.setStdDev(sigma_r);
	bf.filter(*out);

}

//空间剪裁，平面分割
pcl::PointCloud<pcl::PointXYZ>::Ptr plane_clip(const pcl::PointCloud<pcl::PointXYZ>::Ptr& src_cloud,
	const Eigen::Vector4f& plane, bool negative)   //平面参数 Eigen::Vector4f(1.0,1.0,1.0,1.0) ax+by+cz+d=0
{
	pcl::PlaneClipper3D<pcl::PointXYZ> clipper(plane);
	pcl::PointIndices::Ptr indices(new pcl::PointIndices);
	clipper.clipPointCloud3D(*src_cloud, indices->indices);
	pcl::PointCloud<pcl::PointXYZ>::Ptr dst_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::ExtractIndices<pcl::PointXYZ> extract;
	extract.setInputCloud(src_cloud);
	extract.setIndices(indices);
	extract.setNegative(negative);
	extract.filter(*dst_cloud);
	return dst_cloud;
}

//构造函数采用仿射变换矩阵，该矩阵也允许剪切修剪区域  ??
pcl::PointCloud<pcl::PointXYZ>::Ptr box_clip(const pcl::PointCloud<pcl::PointXYZ>::Ptr& src_cloud,
	const Eigen::Affine3f& transformation, bool negative)
	//	transformation	the 3x3 affine transformation matrix that is used to describe the unit cube 	用于描述单位立方体的3x3仿射变换矩阵??
{
	pcl::BoxClipper3D<pcl::PointXYZ> clipper(transformation);
	pcl::PointIndices::Ptr indices(new pcl::PointIndices);
	clipper.clipPointCloud3D(*src_cloud, indices->indices);
	pcl::PointCloud<pcl::PointXYZ>::Ptr dst_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::ExtractIndices<pcl::PointXYZ> extract;
	extract.setInputCloud(src_cloud);
	extract.setIndices(indices);
	extract.setNegative(negative);
	extract.filter(*dst_cloud);
	return dst_cloud;
}

//立方体分割
pcl::PointCloud<pcl::PointXYZ>::Ptr box_clip2(const pcl::PointCloud<pcl::PointXYZ>::Ptr& src_cloud,
	const Eigen::Vector4f& min, const Eigen::Vector4f& max, bool negative)
	//Eigen::Vector4f(x_min, y_min, z_min, 1.0) Eigen::Vector4f(x_max, y_max, z_max, 1.0)//Min和Max是指立方体的两个对角点。每个点由一个四维向量表示，通常最后一个是1
{

	pcl::CropBox<pcl::PointXYZ> box_filter;//滤波器对象
	box_filter.setMin(min);
	box_filter.setMax(max);
	box_filter.setNegative(negative);//是保留立方体内的点而去除其他点，还是反之。false是将盒子内的点去除，默认为false
	box_filter.setInputCloud(src_cloud);//输入源
	pcl::PointCloud<pcl::PointXYZ>::Ptr dst_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	box_filter.filter(*dst_cloud);//输出
	return dst_cloud;
}









//直线拟合
pcl::ModelCoefficients::Ptr linefitting(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr line, float Threshold)
{

	//创建一个模型参数对象，用于记录结果
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);  //inliers表示误差能容忍的点 记录的是点云的序号
	pcl::SACSegmentation<pcl::PointXYZRGBA> seg;     // 创建一个分割器
	seg.setOptimizeCoefficients(true);      // Optional，这个设置可以选定结果平面展示的点是分割掉的点还是分割剩下的点。
	seg.setModelType(pcl::SACMODEL_LINE);  // Mandatory-设置目标几何形状 直线 (x-a)/d=(y-b)/e=(z-c)/f 
	//seg.setModelType(pcl::SACMODEL_PLANE);  // Mandatory-设置目标几何形状  平面 ax+by+cz+d=0

	seg.setMethodType(pcl::SAC_RANSAC);     //分割方法：随机采样法
	seg.setDistanceThreshold(Threshold);         //设置误差容忍范围，也就是阈值0.001
	seg.setInputCloud(cloud);               //输入点云
	seg.segment(*inliers, *coefficients);   //分割点云，获得平面和法向量
	float a = coefficients->values[0];
	float b = coefficients->values[1];
	float c = coefficients->values[2];
	float d = coefficients->values[3];
	float e = coefficients->values[4];
	float f = coefficients->values[5];

	
	//打印直线方程
	//std::cout << "拟合直线方程：（--为+）" << endl;
	//std::cout << "( x -" << a << ")/" << d << "= ( y -" << b << ")/" << e << "= ( z -" << c << ")/" << f << endl;   //abcdef可能为负


	/*子集提取*/
	// 直线点获取

	for (int i = 0; i < inliers->indices.size(); ++i) {
		line->points.push_back(cloud->points.at(inliers->indices[i]));
	}

	return coefficients;

	/*单个直线时
	//直线点获取
	pcl::PointCloud<pcl::PointXYZ>::Ptr c_plane2(new pcl::PointCloud<pcl::PointXYZ>);  //存储直线点云
	pcl::ExtractIndices<pcl::PointXYZ> extract;  //创建点云提取对象
	extract.setInputCloud(cloud);    //设置输入点云
	extract.setIndices(inliers);     //设置分割后的内点为需要提取的点集
	extract.setNegative(false);      //false提取内点, true提取外点
	extract.filter(*c_plane2);        //提取输出存储到c_plane2*/


	/*// 点云可视化
	pcl::visualization::PCLVisualizer viewer;
	viewer.addPointCloud(cloud, "cloud");  // 加载比对点云


	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBA> c_plane_color(line, 255, 0,
		0);  // 设置点云颜色
	viewer.addPointCloud(line, c_plane_color, "c_line");  // 加载凹凸点云
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5,
		"c_line");  // 设置点云大小
	 //pcl::io::savePCDFile("outdoor02.pcd", *c_plane2);   //拟合的直线，可另存为pcd文件
	viewer.spin();*/
}


//平面拟合
pcl::ModelCoefficients::Ptr planefitting(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr plane, float Threshold)
{

	//创建一个模型参数对象，用于记录结果
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);  //inliers表示误差能容忍的点 记录的是点云的序号
	pcl::SACSegmentation<pcl::PointXYZRGBA> seg;     // 创建一个分割器
	//seg.segment
	seg.setOptimizeCoefficients(true);      // Optional，这个设置可以选定结果平面展示的点是分割掉的点还是分割剩下的点。
	//seg.setModelType(pcl::SACMODEL_LINE);  // Mandatory-设置目标几何形状 直线 (x-a)/d=(y-b)/e=(z-c)/f 
	seg.setModelType(pcl::SACMODEL_PLANE);  // Mandatory-设置目标几何形状  平面 ax+by+cz+d=0
	
	seg.setMethodType(pcl::SAC_RANSAC);     //分割方法：随机采样法
	seg.setDistanceThreshold(Threshold);         //设置误差容忍范围，也就是阈值0.001
	seg.setInputCloud(cloud);               //输入点云
	seg.segment(*inliers, *coefficients);   //分割点云，获得平面和法向量
	float a = coefficients->values[0];
	float b = coefficients->values[1];
	float c = coefficients->values[2];
	float d = coefficients->values[3];

	/*//打印直线方程
	std::cout << "拟合平面方程：（--为+）" << endl;
	std::cout << a << "*x+" << b << "*y+" << c << "*z+" << d << "=0" << endl;   //abcdef可能为负*/


	/*子集提取*/
	// 直线点获取

	for (int i = 0; i < inliers->indices.size(); ++i) {
		plane->points.push_back(cloud->points.at(inliers->indices[i]));
	}


	/*单个直线时
	//直线点获取
	pcl::PointCloud<pcl::PointXYZ>::Ptr c_plane2(new pcl::PointCloud<pcl::PointXYZ>);  //存储直线点云
	pcl::ExtractIndices<pcl::PointXYZ> extract;  //创建点云提取对象
	extract.setInputCloud(cloud);    //设置输入点云
	extract.setIndices(inliers);     //设置分割后的内点为需要提取的点集
	extract.setNegative(false);      //false提取内点, true提取外点
	extract.filter(*c_plane2);        //提取输出存储到c_plane2*/


	/*// 点云可视化
	pcl::visualization::PCLVisualizer viewer;
	viewer.addPointCloud(cloud, "cloud");  // 加载比对点云


	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> c_plane_color(plane, 255, 0,0);  // 设置点云颜色
	viewer.addPointCloud(plane, c_plane_color, "c_plane");  // 加载凹凸点云
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "c_plane");  // 设置点云大小
	 //pcl::io::savePCDFile("outdoor02.pcd", *c_plane2);   //拟合的直线，可另存为pcd文件
	viewer.spin();*/
	return coefficients;
}

//三维球拟合
pcl::ModelCoefficients::Ptr ballfitting(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr ball, float Threshold)
{

	//创建一个模型参数对象，用于记录结果
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);  //inliers表示误差能容忍的点 记录的是点云的序号
	pcl::SACSegmentation<pcl::PointXYZRGBA> seg;     // 创建一个分割器
	seg.setOptimizeCoefficients(true);      // Optional，这个设置可以选定结果平面展示的点是分割掉的点还是分割剩下的点。
	
	seg.setModelType(pcl::SACMODEL_SPHERE);  //
	//seg.setModelType(pcl::SACMODEL_LINE);  // Mandatory-设置目标几何形状 直线 (x-a)/d=(y-b)/e=(z-c)/f 
	//seg.setModelType(pcl::SACMODEL_PLANE);  // Mandatory-设置目标几何形状  平面 ax+by+cz+d=0

	seg.setMethodType(pcl::SAC_RANSAC);     //分割方法：随机采样法
	//seg.setRadiusLimits(15, 25);
	seg.setDistanceThreshold(Threshold);         //设置误差容忍范围，也就是阈值0.001
	seg.setInputCloud(cloud);               //输入点云
	seg.segment(*inliers, *coefficients);   //分割点云，获得平面和法向量
	float a = coefficients->values[0];
	float b = coefficients->values[1];
	float c = coefficients->values[2];
	float d = coefficients->values[3];
	//float e = coefficients->values[4];
	//float f = coefficients->values[5];

	//打印直线方程
	//std::cout << "拟合直线方程：（--为+）" << endl;
	std::cout << "( " << a << "," << b << "," << c << ")" <<std::endl;   //abcdef可能为负
	std::cout << "r= " << d <<  std::endl;   //abcdef可能为负

	/*子集提取*/
	// 直线点获取

	for (int i = 0; i < inliers->indices.size(); ++i) {
		ball->points.push_back(cloud->points.at(inliers->indices[i]));
	}
	return coefficients;

	/*单个直线时
	//直线点获取
	pcl::PointCloud<pcl::PointXYZ>::Ptr c_plane2(new pcl::PointCloud<pcl::PointXYZ>);  //存储直线点云
	pcl::ExtractIndices<pcl::PointXYZ> extract;  //创建点云提取对象
	extract.setInputCloud(cloud);    //设置输入点云
	extract.setIndices(inliers);     //设置分割后的内点为需要提取的点集
	extract.setNegative(false);      //false提取内点, true提取外点
	extract.filter(*c_plane2);        //提取输出存储到c_plane2*/


	/*// 点云可视化
	pcl::visualization::PCLVisualizer viewer;
	viewer.addPointCloud(cloud, "cloud");  // 加载比对点云


	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> c_plane_color(ball, 255, 0,
		0);  // 设置点云颜色
	viewer.addPointCloud(ball, c_plane_color, "c_line");  // 加载凹凸点云
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5,
		"c_line");  // 设置点云大小
	 //pcl::io::savePCDFile("outdoor02.pcd", *c_plane2);   //拟合的直线，可另存为pcd文件
	viewer.spin();*/
}

//边缘提取
void boundary(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudin, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr boundPoints)
{
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr noBoundPoints(new pcl::PointCloud<pcl::PointXYZRGBA>);
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	pcl::PointCloud<pcl::Boundary> boundaries;
	pcl::BoundaryEstimation<pcl::PointXYZRGBA, pcl::Normal, pcl::Boundary> est;
	pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGBA>());

	pcl::NormalEstimation<pcl::PointXYZRGBA, pcl::Normal> normEst;  //其中pcl::PointXYZ表示输入类型数据，pcl::Normal表示输出类型,且pcl::Normal前三项是法向，最后一项是曲率
	normEst.setInputCloud(cloudin);
	normEst.setSearchMethod(tree);
	// normEst.setRadiusSearch(2);  //法向估计的半径
	normEst.setKSearch(90);  //法向估计的点数        越大越好 貌似要三的倍数？？
	normEst.compute(*normals);

	//normal_est.setViewPoint(0,0,0); //这个应该会使法向一致
	est.setInputCloud(cloudin);
	est.setInputNormals(normals);
	//  est.setAngleThreshold(90);
	//   est.setSearchMethod (pcl::search::KdTree<pcl::PointXYZ>::Ptr (new pcl::search::KdTree<pcl::PointXYZ>));
	est.setSearchMethod(tree);
	est.setKSearch(100);  //一般这里的数值越高，最终边界识别的精度越好
	//est.setRadiusSearch(everagedistance);  //搜索半径
	est.compute(boundaries);

	int countBoundaries = 0;
	for (int i = 0; i < cloudin->size(); i++) {
		uint8_t x = (boundaries.points[i].boundary_point);
		int a = static_cast<int>(x); //该函数的功能是强制类型转换
		if (a == 1)
		{
			//  boundPoints.push_back(cloud->points[i]);
			(*boundPoints).push_back(cloudin->points[i]);
			countBoundaries++;
		}
		else
			(*noBoundPoints).push_back(cloudin->points[i]);

	}

}


//点云投影 三维变二维 同投影滤波
pcl::PointCloud<pcl::PointXYZ>::Ptr proj(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud3d,
	float a, float b, float c, float d)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2d(new(pcl::PointCloud<pcl::PointXYZ>));

	// Create a set of planar coefficients with X=Y=0,Z=1
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
	coefficients->values.resize(4);
	coefficients->values[0] = a;
	coefficients->values[1] = b;
	coefficients->values[2] = c;
	coefficients->values[3] = d;

	// Create the filtering object
	pcl::ProjectInliers<pcl::PointXYZ> proj;
	proj.setModelType(pcl::SACMODEL_PLANE); // 三维平面参数
	proj.setInputCloud(cloud3d);
	proj.setModelCoefficients(coefficients);
	proj.filter(*cloud2d);

	return cloud2d;
}


//封闭stl文件体积
double STLvolume(std::string stlname)
{
	vtkSmartPointer<vtkSTLReader> reader = vtkSmartPointer<vtkSTLReader>::New();
	reader->SetFileName(stlname.c_str());
	reader->Update();

	vtkSmartPointer<vtkTriangleFilter> tri = vtkSmartPointer<vtkTriangleFilter>::New();
	tri->SetInputData(reader->GetOutput());
	tri->Update();
	vtkSmartPointer<vtkMassProperties> poly = vtkSmartPointer<vtkMassProperties>::New();
	poly->SetInputData(tri->GetOutput());
	poly->Update();

	double vol = poly->GetVolume();
	return vol;
}

//封闭stl文件表面积
double STLsurface(std::string stlname)
{
	vtkSmartPointer<vtkSTLReader> reader = vtkSmartPointer<vtkSTLReader>::New();
	reader->SetFileName(stlname.c_str());
	reader->Update();

	vtkSmartPointer<vtkTriangleFilter> tri = vtkSmartPointer<vtkTriangleFilter>::New();
	tri->SetInputData(reader->GetOutput());
	tri->Update();
	vtkSmartPointer<vtkMassProperties> poly = vtkSmartPointer<vtkMassProperties>::New();
	poly->SetInputData(tri->GetOutput());
	poly->Update();

	double area = poly->GetSurfaceArea();
	return area;
}

//封闭ply文件体积
double PLYvolume(std::string plyname)
{
	vtkSmartPointer<vtkPLYReader> reader = vtkSmartPointer<vtkPLYReader>::New();
	reader->SetFileName(plyname.c_str());
	reader->Update();

	vtkSmartPointer<vtkTriangleFilter> tri = vtkSmartPointer<vtkTriangleFilter>::New();
	tri->SetInputData(reader->GetOutput());
	tri->Update();
	vtkSmartPointer<vtkMassProperties> poly = vtkSmartPointer<vtkMassProperties>::New();
	poly->SetInputData(tri->GetOutput());
	poly->Update();

	double vol = poly->GetVolume();
	return vol;
}

//封闭ply文件表面积
double PLYsurface(std::string plyname)
{
	vtkSmartPointer<vtkSTLReader> reader = vtkSmartPointer<vtkSTLReader>::New();
	reader->SetFileName(plyname.c_str());
	reader->Update();

	vtkSmartPointer<vtkTriangleFilter> tri = vtkSmartPointer<vtkTriangleFilter>::New();
	tri->SetInputData(reader->GetOutput());
	tri->Update();
	vtkSmartPointer<vtkMassProperties> poly = vtkSmartPointer<vtkMassProperties>::New();
	poly->SetInputData(tri->GetOutput());
	poly->Update();

	double area = poly->GetSurfaceArea();
	return area;
}



//计算平面度计算，导入平面点云
float pmd(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud)
{

	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr plane(new pcl::PointCloud<pcl::PointXYZRGBA>);
	float Threshold = 0.1;

	//创建一个模型参数对象，用于记录结果
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);  //inliers表示误差能容忍的点 记录的是点云的序号
	pcl::SACSegmentation<pcl::PointXYZRGBA> seg;     // 创建一个分割器
	seg.setOptimizeCoefficients(true);      // Optional，这个设置可以选定结果平面展示的点是分割掉的点还是分割剩下的点。
	//seg.setModelType(pcl::SACMODEL_LINE);  // Mandatory-设置目标几何形状 直线 (x-a)/d=(y-b)/e=(z-c)/f 
	seg.setModelType(pcl::SACMODEL_PLANE);  // Mandatory-设置目标几何形状  平面 ax+by+cz+d=0

	seg.setMethodType(pcl::SAC_RANSAC);     //分割方法：随机采样法
	seg.setDistanceThreshold(Threshold);         //设置误差容忍范围，也就是阈值0.001
	seg.setInputCloud(cloud);               //输入点云
	seg.segment(*inliers, *coefficients);   //分割点云，获得平面和法向量

	for (int i = 0; i < inliers->indices.size(); ++i)
	{
		plane->points.push_back(cloud->points.at(inliers->indices[i]));
	}

	float a = coefficients->values[0];
	float b = coefficients->values[1];
	float c = coefficients->values[2];
	float d = coefficients->values[3];

	float dd, dmax, dmin;
	dd = dmax = dmin = 0;

	for (size_t i = 0; i < plane->points.size(); i++)
	{
		dd = (a * plane->points[i].x + b * plane->points[i].y + c * plane->points[i].z + d) / (sqrtf(a * a + b * b + c * c));
		if (dmax < dd)
			dmax = dd;
		if (dmin > dd)
			dmin = dd;
	}
	float pmd;
	if (dmax * dmin < 0)
		pmd = dmax - dmin;
	else
		pmd = fabs(dmax - dmin);

	return pmd;
}

//计算平行度，已知基准平面的方程并导入平行于基准平面的理想平面方程
float pxd(pcl::PointCloud<pcl::PointXYZ>::Ptr plane, float a, float b, float c, float d)
{
	float dd, dmax, dmin;
	dd = dmax = dmin = 0;

	for (size_t i = 0; i < plane->points.size(); i++)
	{
		dd = (a * plane->points[i].x + b * plane->points[i].y + c * plane->points[i].z + d) / (sqrtf(a * a + b * b + c * c));
		if (dmax < dd)
			dmax = dd;
		if (dmin > dd)
			dmin = dd;
	}
	float pxd;
	if (dmax * dmin < 0)
		pxd = dmax - dmin;
	else
		pxd = fabs(dmax - dmin);

	return pxd;
}

//计算平行度，已知基准直线的方程并导入平行于基准直线的理想直线方程
float pxd(pcl::PointCloud<pcl::PointXYZ>::Ptr plane, float a, float b, float c, float d, float e, float f)
{
	float dd, dmax;
	dd = dmax = 0;

	for (size_t i = 0; i < plane->points.size(); i++)
	{
		dd = fabs(((plane->points[i].x - a) * d + (plane->points[i].y - b) * e + (plane->points[i].z - c) * f) / (sqrtf(d * d + e * e + f * f)));
		if (dmax < dd)
			dmax = dd;
	}

	return dmax;
}

//sac_ia点云精配准,输出目标点云和转换后的源点云，显示点云 转移矩阵 评分
void sac_ia(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_src_o, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_tgt_o, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr sac_result,Eigen::Matrix4f &sac_trans)
{
	clock_t start = clock();
	/*//去除NAN点
	std::vector<int> indices_src; //保存去除的点的索引
	pcl::removeNaNFromPointCloud(*cloud_src_o, *cloud_src_o, indices_src);
	std::cout << "remove *cloud_src_o nan" << endl;*/




	//源点云 下采样滤波 减少fpfh的计算量
	pcl::VoxelGrid<pcl::PointXYZRGBA> voxel_grid;
	voxel_grid.setLeafSize(0.01, 0.01, 0.01);
	voxel_grid.setInputCloud(cloud_src_o);
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_src(new pcl::PointCloud<pcl::PointXYZRGBA>);
	voxel_grid.filter(*cloud_src);

	//std::cout << "down size *cloud_src_o from " << cloud_src_o->size() << "to" << cloud_src->size() << endl;
	//pcl::io::savePCDFileASCII("src_down.pcd", *cloud_src);



	//计算表面法线
	pcl::NormalEstimation<pcl::PointXYZRGBA, pcl::Normal> ne_src;
	ne_src.setInputCloud(cloud_src);
	pcl::search::KdTree< pcl::PointXYZRGBA>::Ptr tree_src(new pcl::search::KdTree< pcl::PointXYZRGBA>());
	ne_src.setSearchMethod(tree_src);
	pcl::PointCloud<pcl::Normal>::Ptr cloud_src_normals(new pcl::PointCloud< pcl::Normal>);
	ne_src.setRadiusSearch(0.02);
	ne_src.compute(*cloud_src_normals);

	/*
	std::vector<int> indices_tgt;
	pcl::removeNaNFromPointCloud(*cloud_tgt_o, *cloud_tgt_o, indices_tgt);
	std::cout << "remove *cloud_tgt_o nan" << endl;*/

	//目标点云 降采样
	pcl::VoxelGrid<pcl::PointXYZRGBA> voxel_grid_2;
	voxel_grid_2.setLeafSize(0.01, 0.01, 0.01);
	voxel_grid_2.setInputCloud(cloud_tgt_o);
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_tgt(new pcl::PointCloud<pcl::PointXYZRGBA>);
	voxel_grid_2.filter(*cloud_tgt);

	//std::cout << "down size *cloud_tgt_o.pcd from " << cloud_tgt_o->size() << "to" << cloud_tgt->size() << endl;
	//pcl::io::savePCDFileASCII("tgt_down.pcd", *cloud_tgt);

	//法线计算
	pcl::NormalEstimation<pcl::PointXYZRGBA, pcl::Normal> ne_tgt;
	ne_tgt.setInputCloud(cloud_tgt);
	pcl::search::KdTree< pcl::PointXYZRGBA>::Ptr tree_tgt(new pcl::search::KdTree< pcl::PointXYZRGBA>());
	ne_tgt.setSearchMethod(tree_tgt);
	pcl::PointCloud<pcl::Normal>::Ptr cloud_tgt_normals(new pcl::PointCloud< pcl::Normal>);
	//ne_tgt.setKSearch(20);
	ne_tgt.setRadiusSearch(0.02);
	ne_tgt.compute(*cloud_tgt_normals);

	//计算FPFH
	pcl::FPFHEstimation<pcl::PointXYZRGBA, pcl::Normal, pcl::FPFHSignature33> fpfh_src;
	fpfh_src.setInputCloud(cloud_src);
	fpfh_src.setInputNormals(cloud_src_normals);
	pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree_src_fpfh(new pcl::search::KdTree<pcl::PointXYZRGBA>);
	fpfh_src.setSearchMethod(tree_src_fpfh);
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs_src(new pcl::PointCloud<pcl::FPFHSignature33>());
	fpfh_src.setRadiusSearch(0.05);
	fpfh_src.compute(*fpfhs_src);
	//std::cout << "compute *cloud_src fpfh" << endl;

	pcl::FPFHEstimation<pcl::PointXYZRGBA, pcl::Normal, pcl::FPFHSignature33> fpfh_tgt;
	fpfh_tgt.setInputCloud(cloud_tgt);
	fpfh_tgt.setInputNormals(cloud_tgt_normals);
	pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree_tgt_fpfh(new pcl::search::KdTree<pcl::PointXYZRGBA>);
	fpfh_tgt.setSearchMethod(tree_tgt_fpfh);
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs_tgt(new pcl::PointCloud<pcl::FPFHSignature33>());
	fpfh_tgt.setRadiusSearch(0.05);
	fpfh_tgt.compute(*fpfhs_tgt);
	//std::cout << "compute *cloud_tgt fpfh" << endl;

	//SAC配准
	pcl::SampleConsensusInitialAlignment<pcl::PointXYZRGBA, pcl::PointXYZRGBA, pcl::FPFHSignature33> scia;
	scia.setInputSource(cloud_src);
	scia.setInputTarget(cloud_tgt);
	scia.setSourceFeatures(fpfhs_src);
	scia.setTargetFeatures(fpfhs_tgt);
	//scia.setMinSampleDistance(1);
	//scia.setNumberOfSamples(2);
	//scia.setCorrespondenceRandomness(20);
	//pcl::PointCloud<pcl::PointXYZRGBA>::Ptr sac_result(new pcl::PointCloud<pcl::PointXYZRGBA>);
	scia.align(*sac_result);
	std::cout << "sac has converged:" << scia.hasConverged() << "  score: " << scia.getFitnessScore() << endl;
	//Eigen::Matrix4f sac_trans;
	sac_trans = scia.getFinalTransformation();
	std::cout << sac_trans << endl;
	pcl::transformPointCloud(*cloud_src_o, *sac_result, sac_trans);   //结果是源点云变换，不是降采样的结果
	//std::cout << "保存变换后的点云到： sac_result.pcd" << std::endl;
	//pcl::io::savePCDFileASCII("sac_result.pcd", *sac_result);
	
	clock_t sac_time = clock();
	//我把计算法线和点特征直方图的时间也算在SAC里面了
	cout << "sac time: " << (double)(sac_time - start) / (double)CLOCKS_PER_SEC << " s" << endl;

	/*pcl::visualization::PCLVisualizer viewer("sac registration Viewer");
	//viewer.createViewPort (0.0, 0, 0.5, 1.0, vp_1);
	//viewer.createViewPort (0.5, 0, 1.0, 1.0, vp_2);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> src_h(cloud_src_o, 0, 255, 0);  //源点云，绿色
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> tgt_h(cloud_tgt_o, 255, 0, 0);   //目标点云，红色
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> final_h(sac_result, 0, 0, 255);  //转换后源点云，蓝色
	viewer.addPointCloud(cloud_src_o, src_h, "source cloud");
	viewer.addPointCloud(cloud_tgt_o, tgt_h, "tgt cloud");
	viewer.addPointCloud(sac_result, final_h, "final cloud");
	viewer.spin();*/

	////icp配准
	//pcl::PointCloud<pcl::PointXYZ>::Ptr icp_result(new pcl::PointCloud<pcl::PointXYZ>);
	//pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
	//icp.setInputSource(sac_result);
	//icp.setInputTarget(cloud_tgt);
	////Set the max correspondence distance to 4cm (e.g., correspondences with higher distances will be ignored)
	//icp.setMaxCorrespondenceDistance(0.04);
	//// 最大迭代次数
	//icp.setMaximumIterations(50);
	//// 两次变化矩阵之间的差值
	//icp.setTransformationEpsilon(1e-10);
	//// 均方误差
	//icp.setEuclideanFitnessEpsilon(0.2);
	//icp.align(*icp_result);

	//clock_t end = clock();
	//
	//
	//cout << "icp time: " << (double)(end - sac_time) / (double)CLOCKS_PER_SEC << " s" << endl;

	//std::cout << "ICP has converged:" << icp.hasConverged()
	//	<< " score: " << icp.getFitnessScore() << std::endl;
	//Eigen::Matrix4f icp_trans;
	//icp_trans = icp.getFinalTransformation();
	////cout<<"ransformationProbability"<<icp.getTransformationProbability()<<endl;
	//std::cout << icp_trans << endl;
	//////使用创建的变换对未过滤的输入点云进行变换
	////pcl::transformPointCloud(*cloud_src_o, *icp_result, icp_trans);
	//////保存转换的输入点云
	////pcl::io::savePCDFileASCII("transformed_sac_icp.pcd", *icp_result);
	//cout << "total time: " << (double)(end - start) / (double)CLOCKS_PER_SEC << " s" << endl;
	//pcl::visualization::PCLVisualizer viewer1("icp registration Viewer");
	////viewer.createViewPort (0.0, 0, 0.5, 1.0, vp_1);
	////viewer.createViewPort (0.5, 0, 1.0, 1.0, vp_2);
	//pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> src_hh(cloud_src, 0, 255, 0);  //源点云，绿色
	//pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> tgt_hh(cloud_tgt, 255, 0, 0);   //目标点云，红色
	//pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> final_hh(icp_result, 0, 0, 255);  //转换后源点云，蓝色
	//viewer1.addPointCloud(cloud_src, src_hh, "source cloud");
	//viewer1.addPointCloud(cloud_tgt, tgt_hh, "tgt cloud");
	//viewer1.addPointCloud(icp_result, final_hh, "final cloud");
	//viewer1.spin();
	//viewer.addCoordinateSystem(1.0);
	//while (!viewer.wasStopped())
	//{
	//	/*viewer.spinOnce(100);
	//	boost::this_thread::sleep(boost::posix_time::microseconds(100000));*/
	//}

	//return sac_result;
}



void icp(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_src, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_tgt, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr icp_result, Eigen::Matrix4f &icp_trans)
{
	clock_t start = clock();
	//icp配准
	//pcl::PointCloud<pcl::PointXYZRGBA>::Ptr icp_result(new pcl::PointCloud<pcl::PointXYZRGBA>);
	pcl::IterativeClosestPoint<pcl::PointXYZRGBA, pcl::PointXYZRGBA> icp;
	icp.setInputSource(cloud_src);
	icp.setInputTarget(cloud_tgt);
	//Set the max correspondence distance to 4cm (e.g., correspondences with higher distances will be ignored)
	//icp.setMaxCorrespondenceDistance(0.04);
	// 最大迭代次数
	icp.setMaximumIterations(100);
	// 两次变化矩阵之间的差值
	//icp.setTransformationEpsilon(1e-10);
	// 均方误差
	//icp.setEuclideanFitnessEpsilon(0.2);
	icp.align(*icp_result);

	clock_t end = clock();


	cout << "icp time: " << (double)(end - start) / (double)CLOCKS_PER_SEC << " s" << endl;

	std::cout << "ICP has converged:" << icp.hasConverged()
		<< " score: " << icp.getFitnessScore() << std::endl;
	//Eigen::Matrix4f icp_trans;
	icp_trans = icp.getFinalTransformation();
	//cout<<"ransformationProbability"<<icp.getTransformationProbability()<<endl;
	std::cout << icp_trans << endl;
	////使用创建的变换对未过滤的输入点云进行变换
	//pcl::transformPointCloud(*cloud_src_o, *icp_result, icp_trans);
	////保存转换的输入点云
	//pcl::io::savePCDFileASCII("transformed_sac_icp.pcd", *icp_result);
	//cout << "total time: " << (double)(end - start) / (double)CLOCKS_PER_SEC << " s" << endl;
	/*pcl::visualization::PCLVisualizer viewer1("icp registration Viewer");
	//viewer.createViewPort (0.0, 0, 0.5, 1.0, vp_1);
	//viewer.createViewPort (0.5, 0, 1.0, 1.0, vp_2);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> src_hh(cloud_src, 0, 255, 0);  //源点云，绿色
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> tgt_hh(cloud_tgt, 255, 0, 0);   //目标点云，红色
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> final_hh(icp_result, 0, 0, 255);  //转换后源点云，蓝色
	viewer1.addPointCloud(cloud_src, src_hh, "source cloud");
	viewer1.addPointCloud(cloud_tgt, tgt_hh, "tgt cloud");
	viewer1.addPointCloud(icp_result, final_hh, "final cloud");
	viewer1.spin();*/

	//return icp_result;

}

//ransac 粗配准
void ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_sou, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_tar)
{
	pcl::SampleConsensusModelRegistration<pcl::PointXYZ>::Ptr model(new pcl::SampleConsensusModelRegistration<pcl::PointXYZ>(cloud_sou));
	model->setInputTarget(cloud_tar);

	pcl::RandomSampleConsensus<pcl::PointXYZ> sac(model, 0.02);
	sac.setMaxIterations(100000);

	if (!sac.computeModel(2))
	{
		PCL_ERROR("Could not compute a valid transformation!\n");
		return;
	}
	Eigen::Matrix4f transformation = Eigen::Matrix4f::Identity();
	Eigen::VectorXf coeff;
	sac.getModelCoefficients(coeff);
	transformation.row(0) = coeff.segment<4>(0);
	transformation.row(1) = coeff.segment<4>(4);
	transformation.row(2) = coeff.segment<4>(8);
	transformation.row(3) = coeff.segment<4>(12);

	pcl::PointCloud<pcl::PointXYZ>::Ptr ransac_result(new(pcl::PointCloud<pcl::PointXYZ>));
	pcl::transformPointCloud(*cloud_sou, *ransac_result, transformation);
	std::cout << "保存变换后的点云到： ransac_result.pcd" << std::endl;
	pcl::io::savePCDFileASCII("ransac_result.pcd", *ransac_result);
}




//匹配模板 已知模板，输入原点云，输出匹配结果
void temp_icp(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_sou)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_tar(new(pcl::PointCloud<pcl::PointXYZ>));
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_sou2(new(pcl::PointCloud<pcl::PointXYZ>));

	////读取模板
	if (pcl::io::loadPLYFile<pcl::PointXYZ>("C:/Users/aemc-intel/source/repos/ply/test3.ply", *cloud_tar) == -1) //* load the file 
	{
		std::cout << "couldn't read file ply \n";
		system("pause");
		return;
	}


	pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
	icp.setInputSource(cloud_sou);

	icp.setInputTarget(cloud_tar);

	icp.setMaximumIterations(10000);

	pcl::PointCloud<pcl::PointXYZ> Final;

	icp.align(Final);

	std::cout << "has converged:" << icp.hasConverged() << "\n\r" << " score: " <<
		icp.getFitnessScore() << std::endl;
	std::cout << icp.getFinalTransformation() << std::endl;

	Eigen::Matrix4f transform_1 = Eigen::Matrix4f::Identity();
	transform_1 = icp.getFinalTransformation();
	pcl::transformPointCloud(*cloud_sou, *cloud_sou2, transform_1);

	/*pcl::visualization::PCLVisualizer viewer("ICP配准");

	// 为点云定义 R,G,B 颜色
	//pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> source_cloud_color_handler(cloud_sou, 255, 255, 255);
	// 输出点云到查看器，使用颜色管理
	viewer.addPointCloud(cloud_sou, "original_cloud");

	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> transformed_cloud_color_handler(cloud_sou2, 255, 0, 0); // 红
	viewer.addPointCloud(cloud_sou2, transformed_cloud_color_handler, "transformed_cloud");
	viewer.addPointCloud(cloud_tar, "target_cloud");

	//viewer.addCoordinateSystem(1.0, "cloud", 0);
	viewer.setBackgroundColor(0.05, 0.05, 0.05, 0); // 设置背景为深灰
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "original_cloud");
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "transformed_cloud");
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "target_cloud");
	viewer.spin();*/
}


//点云坐标变换 默认的单位阵
pcl::PointCloud<pcl::PointXYZ>::Ptr trans(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_start, Eigen::Matrix4f transform_1 = Eigen::Matrix4f::Identity())
{
	//Eigen::Matrix4f transform_1 = Eigen::Matrix4f::Identity();
	//float theta = M_PI / 4; // 弧度角
	//transform_1(0, 0) = cos(theta);
	//transform_1(0, 1) = -sin(theta);
	//transform_1(1, 0) = sin(theta);
	//transform_1(1, 1) = cos(theta);
	////    //    	(行, 列)
	//transform_1(0, 3) = 0.5;
	//transform_1(1, 3) = 0.2;
	//transform_1(2, 3) = 0.1;

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_end(new(pcl::PointCloud<pcl::PointXYZ>));
	pcl::transformPointCloud(*cloud_start, *cloud_end, transform_1);
	return cloud_end;
}


//点云网格化 降采样 统计滤波 重采样 生成法线 网格化
pcl::PolygonMesh grid(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
	// Load input file
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_downSampled(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_smoothed(new pcl::PointCloud<pcl::PointXYZ>);


	// ----------------------开始你的代码--------------------------//
	// 请参考之前文章中点云下采样，滤波、平滑等内容，以及PCL官网实现以下功能。代码不难。

	// 下采样
	pcl::VoxelGrid<pcl::PointXYZ> downSampled;  //创建滤波对象
	downSampled.setInputCloud(cloud);            //设置需要过滤的点云给滤波对象
	downSampled.setLeafSize(0.01f, 0.01f, 0.01f);  //设置滤波时创建的体素体积为1cm的立方体
	downSampled.filter(*cloud_downSampled);           //执行滤波处理，存储输出
   // pcl::io::savePCDFile("/home/xiaohu/learn_SLAM/zuoye15/作业15-点云平滑及法线估计及显示/data/downsampledPC.pcd", *cloud_downSampled);

	// 统计滤波
	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> statisOutlierRemoval;       //创建滤波器对象
	statisOutlierRemoval.setInputCloud(cloud_downSampled);            //设置待滤波的点云
	statisOutlierRemoval.setMeanK(50);                                //设置在进行统计时考虑查询点临近点数
	statisOutlierRemoval.setStddevMulThresh(3.0);                     //设置判断是否为离群点的阀值:均值+1.0*标准差
	statisOutlierRemoval.filter(*cloud_filtered);                     //滤波结果存储到cloud_filtered
   // pcl::io::savePCDFile("/home/xiaohu/learn_SLAM/zuoye15/作业15-点云平滑及法线估计及显示/data/filteredPC.pcd", *cloud_filtered);

	// 对点云重采样  
	pcl::search::KdTree<pcl::PointXYZ>::Ptr treeSampling(new pcl::search::KdTree<pcl::PointXYZ>);// 创建用于最近邻搜索的KD-Tree
	pcl::PointCloud<pcl::PointXYZ> mls_point;    //输出MLS
	pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ> mls; // 定义最小二乘实现的对象mls
	mls.setComputeNormals(false);  //设置在最小二乘计算中是否需要存储计算的法线
	mls.setInputCloud(cloud_filtered);         //设置待处理点云
	mls.setPolynomialOrder(2);            // 拟合2阶多项式拟合
	//mls.setPolynomialFit(false);     // 设置为false可以 加速 smooth
	mls.setSearchMethod(treeSampling);         // 设置KD-Tree作为搜索方法
	mls.setSearchRadius(0.05);           // 单位m.设置用于拟合的K近邻半径
	mls.process(mls_point);                 //输出
	// 输出重采样结果
	cloud_smoothed = mls_point.makeShared();
	//std::cout << "cloud_smoothed: " << cloud_smoothed->size() << std::endl;
	//save cloud_smoothed
  //  pcl::io::savePCDFileASCII("/home/xiaohu/learn_SLAM/zuoye15/作业15-点云平滑及法线估计及显示/data/cloud_smoothed.pcd", *cloud_smoothed);


	// 法线估计
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimation;             //创建法线估计的对象
	normalEstimation.setInputCloud(cloud_smoothed);                         //输入点云
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);// 创建用于最近邻搜索的KD-Tree
	normalEstimation.setSearchMethod(tree);
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>); // 定义输出的点云法线
	// K近邻确定方法，使用k个最近点，或者确定一个以r为半径的圆内的点集来确定都可以，两者选1即可
	normalEstimation.setKSearch(10);// 使用当前点周围最近的10个点
	//normalEstimation.setRadiusSearch(0.03);            //对于每一个点都用半径为3cm的近邻搜索方式
	normalEstimation.compute(*normals);               //计算法线
	// 输出法线
	//std::cout << "normals: " << normals->size() << ", " << "normals fields: " << pcl::getFieldsList(*normals) << std::endl;
	//  pcl::io::savePCDFileASCII("/home/xiaohu/learn_SLAM/zuoye15/作业15-点云平滑及法线估计及显示/data/normals.pcd", *normals);

	  // 将点云位姿、颜色、法线信息连接到一起
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
	pcl::concatenateFields(*cloud_smoothed, *normals, *cloud_with_normals);
	// pcl::io::savePCDFileASCII("/home/xiaohu/learn_SLAM/zuoye15/作业15-点云平滑及法线估计及显示/data/cloud_with_normals.pcd", *cloud_with_normals);

	 // 贪心投影三角化
	 //定义搜索树对象
	pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
	tree2->setInputCloud(cloud_with_normals);

	// 三角化
	pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;   // 定义三角化对象
	pcl::PolygonMesh triangles; //存储最终三角化的网络模型

	// 设置三角化参数
	gp3.setSearchRadius(0.1);  //设置搜索时的半径，也就是KNN的球半径
	gp3.setMu(2.5);  //设置样本点搜索其近邻点的最远距离为2.5倍（典型值2.5-3），这样使得算法自适应点云密度的变化
	gp3.setMaximumNearestNeighbors(100);    //设置样本点最多可搜索的邻域个数，典型值是50-100

	gp3.setMinimumAngle(M_PI / 18); // 设置三角化后得到的三角形内角的最小的角度为10°
	gp3.setMaximumAngle(2 * M_PI / 3); // 设置三角化后得到的三角形内角的最大角度为120°

	gp3.setMaximumSurfaceAngle(M_PI / 4); // 设置某点法线方向偏离样本点法线的最大角度45°，如果超过，连接时不考虑该点
	gp3.setNormalConsistency(false);  //设置该参数为true保证法线朝向一致，设置为false的话不会进行法线一致性检查

	gp3.setInputCloud(cloud_with_normals);     //设置输入点云为有向点云
	gp3.setSearchMethod(tree2);   //设置搜索方式
	gp3.reconstruct(triangles);  //重建提取三角化

	// 显示网格化结果
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setBackgroundColor(0, 0, 0);  //
	viewer->addPolygonMesh(triangles, "wangge");  //
	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
	//pcl::io::savePLYFile("1", triangles);  //可存为ply文件
	return triangles;
}


//网格转换为点云
void mesh2cloud(pcl::PolygonMesh mesh, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
	//pcl::io::loadPolygonFilePLY("bunny.ply", mesh);
	fromPCLPointCloud2(mesh.cloud, *cloud);
}

//预处理，降采样+滤波，保存到cr、cl 
void pre(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2, std::string xyz, std::string xyz1)
{
	pcl::PolygonMesh mesh;
	pcl::PolygonMesh mesh1;


	if (pcl::io::loadPolygonFileOBJ(xyz, mesh) == -1)//*打开点云文件  xyz为ply文件名
	{
		PCL_ERROR("Couldn't read file\n");
		exit(0);
	}
	pcl::fromPCLPointCloud2(mesh.cloud, *cloud1);

	if (pcl::io::loadPolygonFileOBJ(xyz1, mesh1) == -1)//*打开点云文件  xyz为ply文件名
	{
		PCL_ERROR("Couldn't read file\n");
		exit(0);
	}
	pcl::fromPCLPointCloud2(mesh1.cloud, *cloud2);


	//下采样
	pcl::VoxelGrid<pcl::PointXYZ> down_filter;
	down_filter.setLeafSize(0.01f, 0.01f, 0.01f);
	down_filter.setInputCloud(cloud1);
	down_filter.filter(*cloud1);

	pcl::VoxelGrid<pcl::PointXYZ> down_filter2;
	down_filter2.setLeafSize(0.01f, 0.01f, 0.01f);
	down_filter2.setInputCloud(cloud2);
	down_filter2.filter(*cloud2);


	//统计滤波

	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
	sor.setInputCloud(cloud1);
	sor.setMeanK(80);                //每个点要分析的邻居数设置为50，标准差乘数设置为1。r=50  n=1.0
	sor.setStddevMulThresh(0.8);    //这意味着所有距离查询点的平均距离均大于1标准差的所有点都将标记为异常值并删除
	sor.filter(*cloud1);     //时间巨长！！！！
	//sor.setNegative(true);


	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor2;
	sor2.setInputCloud(cloud2);
	sor2.setMeanK(30);                //每个点要分析的邻居数设置为50，标准差乘数设置为1。r=50  n=1.0
	sor2.setStddevMulThresh(0.05);    //这意味着所有距离查询点的平均距离均大于1标准差的所有点都将标记为异常值并删除
	sor2.filter(*cloud2);     //时间巨长！！！！


}

//边缘提取+滤波
void boundary(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr boundPoints)
{
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	pcl::PointCloud<pcl::Boundary> boundaries;
	pcl::BoundaryEstimation<pcl::PointXYZ, pcl::Normal, pcl::Boundary> est;
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());

	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normEst;  //其中pcl::PointXYZ表示输入类型数据，pcl::Normal表示输出类型,且pcl::Normal前三项是法向，最后一项是曲率
	normEst.setInputCloud(cloud);
	normEst.setSearchMethod(tree);
	// normEst.setRadiusSearch(2);  //法向估计的半径
	normEst.setKSearch(90);  //法向估计的点数        越大越好 貌似要三的倍数？？
	normEst.compute(*normals);

	//normal_est.setViewPoint(0,0,0); //这个应该会使法向一致
	est.setInputCloud(cloud);
	est.setInputNormals(normals);
	//  est.setAngleThreshold(90);
	//   est.setSearchMethod (pcl::search::KdTree<pcl::PointXYZ>::Ptr (new pcl::search::KdTree<pcl::PointXYZ>));
	est.setSearchMethod(tree);
	est.setKSearch(200);  //一般这里的数值越高，最终边界识别的精度越好
	//est.setRadiusSearch(everagedistance);  //搜索半径
	est.compute(boundaries);

	int countBoundaries = 0;
	for (int i = 0; i < cloud->size(); i++)
	{
		uint8_t x = (boundaries.points[i].boundary_point);
		int a = static_cast<int>(x); //该函数的功能是强制类型转换
		if (a == 1)
		{
			//  boundPoints.push_back(cloud->points[i]);
			(*boundPoints).push_back(cloud->points[i]);
			countBoundaries++;
		}
		/*else
			(*noboundPoints).push_back(cloud->points[i]);*/

	}


	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
	sor.setInputCloud(boundPoints);
	sor.setMeanK(30);                //每个点要分析的邻居数设置为50，标准差乘数设置为1。r=50  n=1.0
	sor.setStddevMulThresh(0.05);    //这意味着所有距离查询点的平均距离均大于1标准差的所有点都将标记为异常值并删除
	sor.filter(*boundPoints);     //时间巨长！！！！边缘滤波后点云 

}

//拟合三维圆并使原点云重合
void match(pcl::PointCloud<pcl::PointXYZ>::Ptr source, pcl::PointCloud<pcl::PointXYZ>::Ptr target, pcl::PointCloud<pcl::PointXYZ>::Ptr source1, Eigen::Matrix4f& transform_ltr)
{
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);  //inliers表示误差能容忍的点 记录的是点云的序号
	pcl::SACSegmentation<pcl::PointXYZ> seg;     // 创建一个分割器
	seg.setOptimizeCoefficients(true);      // Optional，这个设置可以选定结果平面展示的点是分割掉的点还是分割剩下的点。
	//seg.setModelType(pcl::SACMODEL_LINE);  // Mandatory-设置目标几何形状 直线 (x-a)/d=(y-b)/e=(z-c)/f 
	seg.setModelType(pcl::SACMODEL_CIRCLE3D);  // Mandatory-设置目标几何形状  3D的圆
	//seg.setModelType(pcl::SACMODEL_PLANE);

	seg.setMethodType(pcl::SAC_RANSAC);     //分割方法：随机采样法
	seg.setDistanceThreshold(0.1);         //设置误差容忍范围，也就是阈值0.001
	seg.setInputCloud(source);               //输入点云
	seg.segment(*inliers, *coefficients);   //分割点云，获得平面和法向量
	float x1 = coefficients->values[0];
	float y1 = coefficients->values[1];
	float z1 = coefficients->values[2];
	float r = coefficients->values[3];
	float x2 = coefficients->values[4];
	float y2 = coefficients->values[5];
	float z2 = coefficients->values[6];

	/*//打印直线方程
	std::cout << "(" << x1 << "," << y1 << "," << z1 << ")" << std::endl;
	std::cout << "半径：" << r << std::endl;
	std::cout << "(" << x2 << "," << y2 << "," << z2 << ")" << std::endl;*/


	Eigen::Vector3d r1(0, 0, 0);   //图1圆心坐标
	r1(0) = x1; r1(1) = y1; r1(2) = z1;

	seg.setInputCloud(target);               //输入点云
	seg.segment(*inliers, *coefficients);   //分割点云，获得平面和法向量
	float x3 = coefficients->values[0];
	float y3 = coefficients->values[1];
	float z3 = coefficients->values[2];
	float rr = coefficients->values[3];
	float x4 = coefficients->values[4];
	float y4 = coefficients->values[5];
	float z4 = coefficients->values[6];

	Eigen::Vector3d r2(0, 0, 0);   //图1圆心坐标
	r2(0) = x3; r2(1) = y3; r2(2) = z3;

	Eigen::Matrix4f transform_1;      //平移矩阵
	transform_1 << 1, 0, 0, 0,
		0, 1, 0, 0,
		0, 0, 1, 0,
		0, 0, 0, 1;
	transform_1(3, 0) = r2(0) - r1(0); transform_1(3, 1) = r2(1) - r1(1); transform_1(3, 2) = r2(2) - r1(2);
	pcl::transformPointCloud(*source1, *source1, transform_1.transpose());

	Eigen::Vector3d vectorBefore(0, 0, 1);
	Eigen::Vector3d vectorAfter(0, 0, 1);
	vectorBefore(0) = x2; vectorBefore(1) = y2; vectorBefore(2) = z2;
	vectorAfter(0) = x4; vectorAfter(1) = y4; vectorAfter(2) = z4;
	Eigen::Matrix3d rotMatrix;
	rotMatrix = Eigen::Quaterniond::FromTwoVectors(vectorBefore, vectorAfter).toRotationMatrix();
	Eigen::Matrix4f transform_3;
	transform_3 << 0, 0, 0, 0,
		0, 0, 0, 0,
		0, 0, 0, 0,
		0, 0, 0, 1;
	transform_3(0, 0) = rotMatrix(0, 0); transform_3(0, 1) = rotMatrix(0, 1);	transform_3(0, 2) = rotMatrix(0, 2);
	transform_3(1, 0) = rotMatrix(1, 0);  transform_3(1, 1) = rotMatrix(1, 1); transform_3(1, 2) = rotMatrix(1, 2);
	transform_3(2, 0) = rotMatrix(2, 0); transform_3(2, 1) = rotMatrix(2, 1);  transform_3(2, 2) = rotMatrix(2, 2);
	pcl::transformPointCloud(*source1, *source1, transform_3);

	transform_ltr = transform_ltr * transform_1.transpose() * transform_3;
}

//icp配准
void icpmatch(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_sou, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_tar, Eigen::Matrix4f transform_ltr)
{
	pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
	icp.setInputSource(cloud_sou);

	icp.setInputTarget(cloud_tar);

	icp.setMaximumIterations(10000);

	pcl::PointCloud<pcl::PointXYZ> Final;

	icp.align(Final);

	//std::cout << "has converged:" << icp.hasConverged() << "\n\r" << " score: " <<icp.getFitnessScore() << std::endl;
	//std::cout << icp.getFinalTransformation() << std::endl;

	Eigen::Matrix4f transform_1 = Eigen::Matrix4f::Identity();
	transform_1 = icp.getFinalTransformation();
	pcl::transformPointCloud(*cloud_sou, *cloud_sou, transform_1);

	transform_ltr = transform_ltr * transform_1;

}

/*//计算平面度
float pmd(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{

	pcl::PointCloud<pcl::PointXYZ>::Ptr plane(new pcl::PointCloud<pcl::PointXYZ>);
	float Threshold = 0.1;

	//创建一个模型参数对象，用于记录结果
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);  //inliers表示误差能容忍的点 记录的是点云的序号
	pcl::SACSegmentation<pcl::PointXYZ> seg;     // 创建一个分割器
	seg.setOptimizeCoefficients(true);      // Optional，这个设置可以选定结果平面展示的点是分割掉的点还是分割剩下的点。
	//seg.setModelType(pcl::SACMODEL_LINE);  // Mandatory-设置目标几何形状 直线 (x-a)/d=(y-b)/e=(z-c)/f 
	seg.setModelType(pcl::SACMODEL_PLANE);  // Mandatory-设置目标几何形状  平面 ax+by+cz+d=0

	seg.setMethodType(pcl::SAC_RANSAC);     //分割方法：随机采样法
	seg.setDistanceThreshold(Threshold);         //设置误差容忍范围，也就是阈值0.001
	seg.setInputCloud(cloud);               //输入点云
	seg.segment(*inliers, *coefficients);   //分割点云，获得平面和法向量

	for (int i = 0; i < inliers->indices.size(); ++i)
	{
		plane->points.push_back(cloud->points.at(inliers->indices[i]));
	}

	float a = coefficients->values[0];
	float b = coefficients->values[1];
	float c = coefficients->values[2];
	float d = coefficients->values[3];

	float dd, dmax, dmin;
	dd = dmax = dmin = 0;

	for (size_t i = 0; i < plane->points.size(); i++)
	{
		dd = (a * plane->points[i].x + b * plane->points[i].y + c * plane->points[i].z + d) / (sqrtf(a * a + b * b + c * c));
		if (dmax < dd)
			dmax = dd;
		if (dmin > dd)
			dmin = dd;
	}
	float pmd;
	if (dmax * dmin < 0)
		pmd = dmax - dmin;
	else
		pmd = fabs(dmax - dmin);

	return pmd;
}*/

void circle3D(pcl::PointCloud<pcl::PointXYZ>::Ptr circle)
{
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);  //inliers表示误差能容忍的点 记录的是点云的序号
	pcl::SACSegmentation<pcl::PointXYZ> seg;     // 创建一个分割器
	seg.setOptimizeCoefficients(true);      // Optional，这个设置可以选定结果平面展示的点是分割掉的点还是分割剩下的点。
	//seg.setModelType(pcl::SACMODEL_LINE);  // Mandatory-设置目标几何形状 直线 (x-a)/d=(y-b)/e=(z-c)/f 
	seg.setModelType(pcl::SACMODEL_CIRCLE3D);  // Mandatory-设置目标几何形状  3D的圆
	//seg.setModelType(pcl::SACMODEL_PLANE);

	seg.setMethodType(pcl::SAC_RANSAC);     //分割方法：随机采样法
	seg.setDistanceThreshold(0.1);         //设置误差容忍范围，也就是阈值0.001
	seg.setInputCloud(circle);               //输入点云
	seg.segment(*inliers, *coefficients);   //分割点云，获得平面和法向量
	float x1 = coefficients->values[0];
	float y1 = coefficients->values[1];
	float z1 = coefficients->values[2];
	float r = coefficients->values[3];
	float x2 = coefficients->values[4];
	float y2 = coefficients->values[5];
	float z2 = coefficients->values[6];

	//打印直线方程
	std::cout << "圆心：(" << x1 << "," << y1 << "," << z1 << ")" << std::endl;
	std::cout << "半径：" << r << std::endl;
	std::cout << "法向：(" << x2 << "," << y2 << "," << z2 << ")" << std::endl;

}

////网格流形检测 0为非流形 1为流形
//bool liuxingjiance(std::string document)
//{
//	//GPP::TriMesh* tri = GPP::Parser::ImportTriMesh("C:/Users/sjtuzhy/Desktop/ply文件/哆啦A梦2h.ply");
//	GPP::TriMesh* tri = GPP::Parser::ImportTriMesh(document);
//	bool isTopo = 0;
//	isTopo = GPP::ComputeMeshTopology::IsTriMeshManifold(tri);
//	return isTopo;
//
//}