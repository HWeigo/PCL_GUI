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

//��ȡpcd�����ļ�
pcl::PointCloud<pcl::PointXYZ>::Ptr  pcdload()
{
	std::string xyz;
	std::cout << "����pcd�ļ���ַ������" << std::endl;
	std::cin >> xyz;
	pcl::PointCloud<pcl::PointXYZ>::Ptr pcdcloud(new pcl::PointCloud<pcl::PointXYZ>);

	if (pcl::io::loadPCDFile<pcl::PointXYZ>(xyz, *pcdcloud) == -1)//*�򿪵����ļ�  xyzΪpcd�ļ���
	{
		PCL_ERROR("Couldn't read file\n");
		exit(0);
	}
	return(pcdcloud);
}

//��ȡply�����ļ�
pcl::PointCloud<pcl::PointXYZ>::Ptr  plyload()
{
	std::string xyz;
	std::cout << "����ply�ļ���ַ������" << std::endl;
	std::cin >> xyz;
	pcl::PointCloud<pcl::PointXYZ>::Ptr plycloud(new pcl::PointCloud<pcl::PointXYZ>);

	if (pcl::io::loadPLYFile<pcl::PointXYZ>(xyz, *plycloud) == -1)//*�򿪵����ļ�  xyzΪply�ļ���
	{
		PCL_ERROR("Couldn't read file\n");
		exit(0);
	}
	return(plycloud);
}


//��ȡobj�����ļ� ��ת��PointXYZ����
pcl::PointCloud<pcl::PointXYZ>::Ptr  objload()
{
	std::string xyz;
	std::cout << "����obj�ļ���ַ������" << std::endl;
	std::cin >> xyz;
	pcl::PolygonMesh mesh;
	;

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

	if (pcl::io::loadPolygonFileOBJ(xyz, mesh) == -1)//*�򿪵����ļ�  xyzΪply�ļ���
	{
		PCL_ERROR("Couldn't read file\n");
		exit(0);
	}
	pcl::fromPCLPointCloud2(mesh.cloud, *cloud);
	//pcl::io::savePCDFileASCII("xxxx.pcd", *cloud);     //��PointXYZ��Ϊpcd ASCII����ʽ  ��xxxxΪ��ַ�����ơ�
	return(cloud);
}

//����obj�����ļ� 
void  objsave(pcl::PolygonMesh mesh,std::string path)
{
	
	pcl::io::saveOBJFile(path, mesh);
	
}



//��pcd�ļ�ת��Ϊply�ļ�
void PCDtoPLYconvertor(std::string input_filename, std::string output_filename)
{
	pcl::PCLPointCloud2 cloud;
	if (pcl::io::loadPCDFile(input_filename, cloud) < 0)
	{
		cout << "Error: cannot load the PCD file!!!" << endl;
		exit(0);
	}
	pcl::PLYWriter writer;
	writer.write(output_filename, cloud, Eigen::Vector4f::Zero(), Eigen::Quaternionf::Identity(), false, true); //plyΪASCII �����ڶ�����������

}

//��ply�ļ�ת��Ϊpcd�ļ�
void PLYtoPCDconvertor(std::string input_filename, std::string output_filename)
{
	pcl::PCLPointCloud2 cloud;
	if (pcl::io::loadPLYFile(input_filename, cloud) < 0)
	{
		cout << "Error: cannot load the PLY file!!!" << endl;
		exit(0);
	}
	pcl::PCDWriter writer;
	writer.write(output_filename, cloud, Eigen::Vector4f::Zero(), Eigen::Quaternionf::Identity(), false); //pcdΪASCII ������һ����������

}



//���ͼתcloud ��Ϊply  ���������֪,��Ҫ���ͼ��ɫ�������Ϣ
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
	// �������ͼ
	for (int m = 0; m < depth.rows; m++)
		for (int n = 0; n < depth.cols; n++)
		{
			// ��ȡ���ͼ��(m,n)����ֵ
			ushort d = depth.ptr<ushort>(m)[n];
			// d ����û��ֵ������ˣ������˵�
			if (d == 0)
				continue;
			// d ����ֵ�������������һ����
			pcl::PointXYZRGBA p;

			// ���������Ŀռ�����
			p.z = double(d) / camera_factor;
			p.x = (n - camera_cx) * p.z / camera_fx;
			p.y = (m - camera_cy) * p.z / camera_fy;

			// ��rgbͼ���л�ȡ������ɫ
			// rgb����ͨ����BGR��ʽͼ�����԰������˳���ȡ��ɫ
			p.b = rgb.ptr<uchar>(m)[n * 3];
			p.g = rgb.ptr<uchar>(m)[n * 3 + 1];
			p.r = rgb.ptr<uchar>(m)[n * 3 + 2];

			// ��p���뵽������
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

//�ӵ����ļ����������ͼ�� ��range image��
void pointcloud2rangeimage(pcl::RangeImage &rangeImage, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{

	cloud->width = (uint32_t)cloud->points.size();
	cloud->height = 1;                                        //���õ��ƶ����ͷ��Ϣ

	// We now want to create a range image from the above point cloud, with a 1deg angular resolution
	 //angular_resolutionΪģ�����ȴ������ĽǶȷֱ��ʣ������ͼ����һ�����ض�Ӧ�ĽǶȴ�С
	float angularResolution = (float)(1.0f * (M_PI / 180.0f));  //   1.0 degree in radians
	 //max_angle_widthΪģ�����ȴ�������ˮƽ�������Ƕȣ�
	float maxAngleWidth = (float)(360.0f * (M_PI / 180.0f));  // 360.0 degree in radians
	//max_angle_heightΪģ�⴫�����Ĵ�ֱ�����������Ƕ�  ��תΪ����
	float maxAngleHeight = (float)(180.0f * (M_PI / 180.0f));  // 180.0 degree in radians
	 //�������Ĳɼ�λ��
	Eigen::Affine3f sensorPose = (Eigen::Affine3f)Eigen::Translation3f(0.0f, 0.0f, 0.0f);
	//���ͼ����ѭ����ϵͳ
	pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;
	float noiseLevel = 0.00;    //noise_level��ȡ���ͼ�����ʱ�����ڵ�Բ�ѯ�����ֵ��Ӱ��ˮƽ
	float minRange = 0.0f;     //min_range������С�Ļ�ȡ���룬С����С��ȡ�����λ��Ϊ��������ä��
	int borderSize = 1;        //border_size������ͼ��ı�Ե�Ŀ��

	//pcl::RangeImage rangeImage;
	rangeImage.createFromPointCloud(*cloud, angularResolution, maxAngleWidth, maxAngleHeight,
		sensorPose, coordinate_frame, noiseLevel, minRange, borderSize);

	//return rangeImage;
}

//��ʾ1 ��ɫ��ʾpcl Ĭ�ϰ�ɫ
void pclviewer1(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::string cloud_string,
	int red = 255,
	int green = 255,
	int blue = 255,
	int point_size = 1)
{
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cloud, red, green, blue);
	viewer->setBackgroundColor(0, 0, 0);
	viewer->addPointCloud<pcl::PointXYZ>(cloud, single_color, cloud_string);                         //cloud string����ʾ�ĵ��Ƶ�����
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, point_size, cloud_string);
	viewer->addCoordinateSystem(1.0);
	viewer->initCameraParameters();
	viewer->spin();     //ʹ���ڱ���
}


//Ⱦɫ
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
//��ʾ2  ��ʾ��ɫpcl XYZRGB
void pclviewer2(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, std::string cloud_string,
	int point_size = 1)
{
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
	viewer->setBackgroundColor(0, 0, 0);
	viewer->addPointCloud<pcl::PointXYZRGB>(cloud, rgb, cloud_string);                         //cloud string����ʾ�ĵ��Ƶ�����
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, point_size, cloud_string);
	viewer->addCoordinateSystem(1.0);
	viewer->initCameraParameters();
	viewer->spin();     //ʹ���ڱ���
}

//һ���cloudviewer
//pcl::visualization::CloudViewer viewer1("cloud viewer");
//viewer1.showCloud(cloud3);             //���ڴ��£����԰�h ��ð���
//while (!viewer1.wasStopped())
//{

//}

//�˲�1 �뾶�˲�
pcl::PointCloud<pcl::PointXYZ>::Ptr  removal_1(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float r, int n)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_r1(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
	// build the filter
	outrem.setInputCloud(cloud);
	outrem.setRadiusSearch(r);
	outrem.setMinNeighborsInRadius(n);
	outrem.setKeepOrganized(true);                       //�뾶r��Ҫ����n���ھ�
	// apply filter
	outrem.filter(*cloud_r1);
	return(cloud_r1);
}

//�˲�2   ͳ���˲� ʱ��޳�
pcl::PointCloud<pcl::PointXYZ>::Ptr  removal_2(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, float r=50, float n=1.0, bool a=1)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_r2(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_r22(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
	sor.setInputCloud(cloud);
	sor.setMeanK(r);                //ÿ����Ҫ�������ھ�������Ϊ50����׼���������Ϊ1��r=50  n=1.0
	sor.setStddevMulThresh(n);    //����ζ�����о����ѯ���ƽ�����������1��׼������е㶼�����Ϊ�쳣ֵ��ɾ��
	sor.filter(*cloud_r2);     //ʱ��޳���������

	sor.setNegative(true);

	sor.filter(*cloud_r22);  //�����޳����ĵ�
	if (a)       //a��1��ʱ��Ҫʣ�µ�      a��0��ʱ��Ҫ�޳���
	{
		return cloud_r2;
	}
	else {
		return cloud_r22;
	}
}

//�˲�3 �����˲�
pcl::PointCloud<pcl::PointXYZ>::Ptr  removal_3(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, std::string xyz, float gt, float lt)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_r3(new pcl::PointCloud<pcl::PointXYZ>);
	// build the condition
	pcl::ConditionAnd<pcl::PointXYZ>::Ptr range_cond(new
		pcl::ConditionAnd<pcl::PointXYZ>());
	range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(new
		pcl::FieldComparison<pcl::PointXYZ>(xyz, pcl::ComparisonOps::GT, gt)));    //greater than
	range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(new
		pcl::FieldComparison<pcl::PointXYZ>(xyz, pcl::ComparisonOps::LT, lt)));    //lower than    // y 0-0.8  �����ڲü�
	// build the filter
	pcl::ConditionalRemoval<pcl::PointXYZ> condrem;
	condrem.setCondition(range_cond);
	condrem.setInputCloud(cloud);
	condrem.setKeepOrganized(true);
	// apply filter
	condrem.filter(*cloud_r3);
	return(cloud_r3);
}

//�˲�4 �����˲�
void pclDownsize(pcl::PointCloud<pcl::PointXYZ>::Ptr in, pcl::PointCloud<pcl::PointXYZ>::Ptr out, float leaf)
{
	pcl::VoxelGrid<pcl::PointXYZ> down_filter;
	down_filter.setLeafSize(leaf, leaf, leaf);
	down_filter.setInputCloud(in);
	down_filter.filter(*out);
}


//�˲�5 ֱͨ�˲�
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

//�˲�5 ͶӰ�˲�  ax+by+cz+d=0
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

//�˲�6 ˫���˲� Ҫ��ṹ XYZI
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

//�ռ���ã�ƽ��ָ�
pcl::PointCloud<pcl::PointXYZ>::Ptr plane_clip(const pcl::PointCloud<pcl::PointXYZ>::Ptr& src_cloud,
	const Eigen::Vector4f& plane, bool negative)   //ƽ����� Eigen::Vector4f(1.0,1.0,1.0,1.0) ax+by+cz+d=0
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

//���캯�����÷���任���󣬸þ���Ҳ��������޼�����  ??
pcl::PointCloud<pcl::PointXYZ>::Ptr box_clip(const pcl::PointCloud<pcl::PointXYZ>::Ptr& src_cloud,
	const Eigen::Affine3f& transformation, bool negative)
	//	transformation	the 3x3 affine transformation matrix that is used to describe the unit cube 	����������λ�������3x3����任����??
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

//������ָ�
pcl::PointCloud<pcl::PointXYZ>::Ptr box_clip2(const pcl::PointCloud<pcl::PointXYZ>::Ptr& src_cloud,
	const Eigen::Vector4f& min, const Eigen::Vector4f& max, bool negative)
	//Eigen::Vector4f(x_min, y_min, z_min, 1.0) Eigen::Vector4f(x_max, y_max, z_max, 1.0)//Min��Max��ָ������������Խǵ㡣ÿ������һ����ά������ʾ��ͨ�����һ����1
{

	pcl::CropBox<pcl::PointXYZ> box_filter;//�˲�������
	box_filter.setMin(min);
	box_filter.setMax(max);
	box_filter.setNegative(negative);//�Ǳ����������ڵĵ��ȥ�������㣬���Ƿ�֮��false�ǽ������ڵĵ�ȥ����Ĭ��Ϊfalse
	box_filter.setInputCloud(src_cloud);//����Դ
	pcl::PointCloud<pcl::PointXYZ>::Ptr dst_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	box_filter.filter(*dst_cloud);//���
	return dst_cloud;
}









//ֱ�����
pcl::ModelCoefficients::Ptr linefitting(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr line, float Threshold)
{

	//����һ��ģ�Ͳ����������ڼ�¼���
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);  //inliers��ʾ��������̵ĵ� ��¼���ǵ��Ƶ����
	pcl::SACSegmentation<pcl::PointXYZRGBA> seg;     // ����һ���ָ���
	seg.setOptimizeCoefficients(true);      // Optional��������ÿ���ѡ�����ƽ��չʾ�ĵ��Ƿָ���ĵ㻹�Ƿָ�ʣ�µĵ㡣
	seg.setModelType(pcl::SACMODEL_LINE);  // Mandatory-����Ŀ�꼸����״ ֱ�� (x-a)/d=(y-b)/e=(z-c)/f 
	//seg.setModelType(pcl::SACMODEL_PLANE);  // Mandatory-����Ŀ�꼸����״  ƽ�� ax+by+cz+d=0

	seg.setMethodType(pcl::SAC_RANSAC);     //�ָ�������������
	seg.setDistanceThreshold(Threshold);         //����������̷�Χ��Ҳ������ֵ0.001
	seg.setInputCloud(cloud);               //�������
	seg.segment(*inliers, *coefficients);   //�ָ���ƣ����ƽ��ͷ�����
	float a = coefficients->values[0];
	float b = coefficients->values[1];
	float c = coefficients->values[2];
	float d = coefficients->values[3];
	float e = coefficients->values[4];
	float f = coefficients->values[5];

	
	//��ӡֱ�߷���
	//std::cout << "���ֱ�߷��̣���--Ϊ+��" << endl;
	//std::cout << "( x -" << a << ")/" << d << "= ( y -" << b << ")/" << e << "= ( z -" << c << ")/" << f << endl;   //abcdef����Ϊ��


	/*�Ӽ���ȡ*/
	// ֱ�ߵ��ȡ

	for (int i = 0; i < inliers->indices.size(); ++i) {
		line->points.push_back(cloud->points.at(inliers->indices[i]));
	}

	return coefficients;

	/*����ֱ��ʱ
	//ֱ�ߵ��ȡ
	pcl::PointCloud<pcl::PointXYZ>::Ptr c_plane2(new pcl::PointCloud<pcl::PointXYZ>);  //�洢ֱ�ߵ���
	pcl::ExtractIndices<pcl::PointXYZ> extract;  //����������ȡ����
	extract.setInputCloud(cloud);    //�����������
	extract.setIndices(inliers);     //���÷ָ����ڵ�Ϊ��Ҫ��ȡ�ĵ㼯
	extract.setNegative(false);      //false��ȡ�ڵ�, true��ȡ���
	extract.filter(*c_plane2);        //��ȡ����洢��c_plane2*/


	/*// ���ƿ��ӻ�
	pcl::visualization::PCLVisualizer viewer;
	viewer.addPointCloud(cloud, "cloud");  // ���رȶԵ���


	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBA> c_plane_color(line, 255, 0,
		0);  // ���õ�����ɫ
	viewer.addPointCloud(line, c_plane_color, "c_line");  // ���ذ�͹����
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5,
		"c_line");  // ���õ��ƴ�С
	 //pcl::io::savePCDFile("outdoor02.pcd", *c_plane2);   //��ϵ�ֱ�ߣ������Ϊpcd�ļ�
	viewer.spin();*/
}


//ƽ�����
pcl::ModelCoefficients::Ptr planefitting(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr plane, float Threshold)
{

	//����һ��ģ�Ͳ����������ڼ�¼���
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);  //inliers��ʾ��������̵ĵ� ��¼���ǵ��Ƶ����
	pcl::SACSegmentation<pcl::PointXYZRGBA> seg;     // ����һ���ָ���
	//seg.segment
	seg.setOptimizeCoefficients(true);      // Optional��������ÿ���ѡ�����ƽ��չʾ�ĵ��Ƿָ���ĵ㻹�Ƿָ�ʣ�µĵ㡣
	//seg.setModelType(pcl::SACMODEL_LINE);  // Mandatory-����Ŀ�꼸����״ ֱ�� (x-a)/d=(y-b)/e=(z-c)/f 
	seg.setModelType(pcl::SACMODEL_PLANE);  // Mandatory-����Ŀ�꼸����״  ƽ�� ax+by+cz+d=0
	
	seg.setMethodType(pcl::SAC_RANSAC);     //�ָ�������������
	seg.setDistanceThreshold(Threshold);         //����������̷�Χ��Ҳ������ֵ0.001
	seg.setInputCloud(cloud);               //�������
	seg.segment(*inliers, *coefficients);   //�ָ���ƣ����ƽ��ͷ�����
	float a = coefficients->values[0];
	float b = coefficients->values[1];
	float c = coefficients->values[2];
	float d = coefficients->values[3];

	/*//��ӡֱ�߷���
	std::cout << "���ƽ�淽�̣���--Ϊ+��" << endl;
	std::cout << a << "*x+" << b << "*y+" << c << "*z+" << d << "=0" << endl;   //abcdef����Ϊ��*/


	/*�Ӽ���ȡ*/
	// ֱ�ߵ��ȡ

	for (int i = 0; i < inliers->indices.size(); ++i) {
		plane->points.push_back(cloud->points.at(inliers->indices[i]));
	}


	/*����ֱ��ʱ
	//ֱ�ߵ��ȡ
	pcl::PointCloud<pcl::PointXYZ>::Ptr c_plane2(new pcl::PointCloud<pcl::PointXYZ>);  //�洢ֱ�ߵ���
	pcl::ExtractIndices<pcl::PointXYZ> extract;  //����������ȡ����
	extract.setInputCloud(cloud);    //�����������
	extract.setIndices(inliers);     //���÷ָ����ڵ�Ϊ��Ҫ��ȡ�ĵ㼯
	extract.setNegative(false);      //false��ȡ�ڵ�, true��ȡ���
	extract.filter(*c_plane2);        //��ȡ����洢��c_plane2*/


	/*// ���ƿ��ӻ�
	pcl::visualization::PCLVisualizer viewer;
	viewer.addPointCloud(cloud, "cloud");  // ���رȶԵ���


	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> c_plane_color(plane, 255, 0,0);  // ���õ�����ɫ
	viewer.addPointCloud(plane, c_plane_color, "c_plane");  // ���ذ�͹����
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "c_plane");  // ���õ��ƴ�С
	 //pcl::io::savePCDFile("outdoor02.pcd", *c_plane2);   //��ϵ�ֱ�ߣ������Ϊpcd�ļ�
	viewer.spin();*/
	return coefficients;
}

//��ά�����
pcl::ModelCoefficients::Ptr ballfitting(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr ball, float Threshold)
{

	//����һ��ģ�Ͳ����������ڼ�¼���
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);  //inliers��ʾ��������̵ĵ� ��¼���ǵ��Ƶ����
	pcl::SACSegmentation<pcl::PointXYZRGBA> seg;     // ����һ���ָ���
	seg.setOptimizeCoefficients(true);      // Optional��������ÿ���ѡ�����ƽ��չʾ�ĵ��Ƿָ���ĵ㻹�Ƿָ�ʣ�µĵ㡣
	
	seg.setModelType(pcl::SACMODEL_SPHERE);  //
	//seg.setModelType(pcl::SACMODEL_LINE);  // Mandatory-����Ŀ�꼸����״ ֱ�� (x-a)/d=(y-b)/e=(z-c)/f 
	//seg.setModelType(pcl::SACMODEL_PLANE);  // Mandatory-����Ŀ�꼸����״  ƽ�� ax+by+cz+d=0

	seg.setMethodType(pcl::SAC_RANSAC);     //�ָ�������������
	//seg.setRadiusLimits(15, 25);
	seg.setDistanceThreshold(Threshold);         //����������̷�Χ��Ҳ������ֵ0.001
	seg.setInputCloud(cloud);               //�������
	seg.segment(*inliers, *coefficients);   //�ָ���ƣ����ƽ��ͷ�����
	float a = coefficients->values[0];
	float b = coefficients->values[1];
	float c = coefficients->values[2];
	float d = coefficients->values[3];
	//float e = coefficients->values[4];
	//float f = coefficients->values[5];

	//��ӡֱ�߷���
	//std::cout << "���ֱ�߷��̣���--Ϊ+��" << endl;
	std::cout << "( " << a << "," << b << "," << c << ")" <<std::endl;   //abcdef����Ϊ��
	std::cout << "r= " << d <<  std::endl;   //abcdef����Ϊ��

	/*�Ӽ���ȡ*/
	// ֱ�ߵ��ȡ

	for (int i = 0; i < inliers->indices.size(); ++i) {
		ball->points.push_back(cloud->points.at(inliers->indices[i]));
	}
	return coefficients;

	/*����ֱ��ʱ
	//ֱ�ߵ��ȡ
	pcl::PointCloud<pcl::PointXYZ>::Ptr c_plane2(new pcl::PointCloud<pcl::PointXYZ>);  //�洢ֱ�ߵ���
	pcl::ExtractIndices<pcl::PointXYZ> extract;  //����������ȡ����
	extract.setInputCloud(cloud);    //�����������
	extract.setIndices(inliers);     //���÷ָ����ڵ�Ϊ��Ҫ��ȡ�ĵ㼯
	extract.setNegative(false);      //false��ȡ�ڵ�, true��ȡ���
	extract.filter(*c_plane2);        //��ȡ����洢��c_plane2*/


	/*// ���ƿ��ӻ�
	pcl::visualization::PCLVisualizer viewer;
	viewer.addPointCloud(cloud, "cloud");  // ���رȶԵ���


	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> c_plane_color(ball, 255, 0,
		0);  // ���õ�����ɫ
	viewer.addPointCloud(ball, c_plane_color, "c_line");  // ���ذ�͹����
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5,
		"c_line");  // ���õ��ƴ�С
	 //pcl::io::savePCDFile("outdoor02.pcd", *c_plane2);   //��ϵ�ֱ�ߣ������Ϊpcd�ļ�
	viewer.spin();*/
}

//��Ե��ȡ
void boundary(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudin, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr boundPoints)
{
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr noBoundPoints(new pcl::PointCloud<pcl::PointXYZRGBA>);
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	pcl::PointCloud<pcl::Boundary> boundaries;
	pcl::BoundaryEstimation<pcl::PointXYZRGBA, pcl::Normal, pcl::Boundary> est;
	pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGBA>());

	pcl::NormalEstimation<pcl::PointXYZRGBA, pcl::Normal> normEst;  //����pcl::PointXYZ��ʾ�����������ݣ�pcl::Normal��ʾ�������,��pcl::Normalǰ�����Ƿ������һ��������
	normEst.setInputCloud(cloudin);
	normEst.setSearchMethod(tree);
	// normEst.setRadiusSearch(2);  //������Ƶİ뾶
	normEst.setKSearch(90);  //������Ƶĵ���        Խ��Խ�� ò��Ҫ���ı�������
	normEst.compute(*normals);

	//normal_est.setViewPoint(0,0,0); //���Ӧ�û�ʹ����һ��
	est.setInputCloud(cloudin);
	est.setInputNormals(normals);
	//  est.setAngleThreshold(90);
	//   est.setSearchMethod (pcl::search::KdTree<pcl::PointXYZ>::Ptr (new pcl::search::KdTree<pcl::PointXYZ>));
	est.setSearchMethod(tree);
	est.setKSearch(100);  //һ���������ֵԽ�ߣ����ձ߽�ʶ��ľ���Խ��
	//est.setRadiusSearch(everagedistance);  //�����뾶
	est.compute(boundaries);

	int countBoundaries = 0;
	for (int i = 0; i < cloudin->size(); i++) {
		uint8_t x = (boundaries.points[i].boundary_point);
		int a = static_cast<int>(x); //�ú����Ĺ�����ǿ������ת��
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


//����ͶӰ ��ά���ά ͬͶӰ�˲�
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
	proj.setModelType(pcl::SACMODEL_PLANE); // ��άƽ�����
	proj.setInputCloud(cloud3d);
	proj.setModelCoefficients(coefficients);
	proj.filter(*cloud2d);

	return cloud2d;
}


//���stl�ļ����
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

//���stl�ļ������
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

//���ply�ļ����
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

//���ply�ļ������
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



//����ƽ��ȼ��㣬����ƽ�����
float pmd(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud)
{

	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr plane(new pcl::PointCloud<pcl::PointXYZRGBA>);
	float Threshold = 0.1;

	//����һ��ģ�Ͳ����������ڼ�¼���
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);  //inliers��ʾ��������̵ĵ� ��¼���ǵ��Ƶ����
	pcl::SACSegmentation<pcl::PointXYZRGBA> seg;     // ����һ���ָ���
	seg.setOptimizeCoefficients(true);      // Optional��������ÿ���ѡ�����ƽ��չʾ�ĵ��Ƿָ���ĵ㻹�Ƿָ�ʣ�µĵ㡣
	//seg.setModelType(pcl::SACMODEL_LINE);  // Mandatory-����Ŀ�꼸����״ ֱ�� (x-a)/d=(y-b)/e=(z-c)/f 
	seg.setModelType(pcl::SACMODEL_PLANE);  // Mandatory-����Ŀ�꼸����״  ƽ�� ax+by+cz+d=0

	seg.setMethodType(pcl::SAC_RANSAC);     //�ָ�������������
	seg.setDistanceThreshold(Threshold);         //����������̷�Χ��Ҳ������ֵ0.001
	seg.setInputCloud(cloud);               //�������
	seg.segment(*inliers, *coefficients);   //�ָ���ƣ����ƽ��ͷ�����

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

//����ƽ�жȣ���֪��׼ƽ��ķ��̲�����ƽ���ڻ�׼ƽ�������ƽ�淽��
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

//����ƽ�жȣ���֪��׼ֱ�ߵķ��̲�����ƽ���ڻ�׼ֱ�ߵ�����ֱ�߷���
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

//sac_ia���ƾ���׼,���Ŀ����ƺ�ת�����Դ���ƣ���ʾ���� ת�ƾ��� ����
void sac_ia(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_src_o, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_tgt_o, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr sac_result,Eigen::Matrix4f &sac_trans)
{
	clock_t start = clock();
	/*//ȥ��NAN��
	std::vector<int> indices_src; //����ȥ���ĵ������
	pcl::removeNaNFromPointCloud(*cloud_src_o, *cloud_src_o, indices_src);
	std::cout << "remove *cloud_src_o nan" << endl;*/




	//Դ���� �²����˲� ����fpfh�ļ�����
	pcl::VoxelGrid<pcl::PointXYZRGBA> voxel_grid;
	voxel_grid.setLeafSize(0.01, 0.01, 0.01);
	voxel_grid.setInputCloud(cloud_src_o);
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_src(new pcl::PointCloud<pcl::PointXYZRGBA>);
	voxel_grid.filter(*cloud_src);

	//std::cout << "down size *cloud_src_o from " << cloud_src_o->size() << "to" << cloud_src->size() << endl;
	//pcl::io::savePCDFileASCII("src_down.pcd", *cloud_src);



	//������淨��
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

	//Ŀ����� ������
	pcl::VoxelGrid<pcl::PointXYZRGBA> voxel_grid_2;
	voxel_grid_2.setLeafSize(0.01, 0.01, 0.01);
	voxel_grid_2.setInputCloud(cloud_tgt_o);
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_tgt(new pcl::PointCloud<pcl::PointXYZRGBA>);
	voxel_grid_2.filter(*cloud_tgt);

	//std::cout << "down size *cloud_tgt_o.pcd from " << cloud_tgt_o->size() << "to" << cloud_tgt->size() << endl;
	//pcl::io::savePCDFileASCII("tgt_down.pcd", *cloud_tgt);

	//���߼���
	pcl::NormalEstimation<pcl::PointXYZRGBA, pcl::Normal> ne_tgt;
	ne_tgt.setInputCloud(cloud_tgt);
	pcl::search::KdTree< pcl::PointXYZRGBA>::Ptr tree_tgt(new pcl::search::KdTree< pcl::PointXYZRGBA>());
	ne_tgt.setSearchMethod(tree_tgt);
	pcl::PointCloud<pcl::Normal>::Ptr cloud_tgt_normals(new pcl::PointCloud< pcl::Normal>);
	//ne_tgt.setKSearch(20);
	ne_tgt.setRadiusSearch(0.02);
	ne_tgt.compute(*cloud_tgt_normals);

	//����FPFH
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

	//SAC��׼
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
	pcl::transformPointCloud(*cloud_src_o, *sac_result, sac_trans);   //�����Դ���Ʊ任�����ǽ������Ľ��
	//std::cout << "����任��ĵ��Ƶ��� sac_result.pcd" << std::endl;
	//pcl::io::savePCDFileASCII("sac_result.pcd", *sac_result);
	
	clock_t sac_time = clock();
	//�ҰѼ��㷨�ߺ͵�����ֱ��ͼ��ʱ��Ҳ����SAC������
	cout << "sac time: " << (double)(sac_time - start) / (double)CLOCKS_PER_SEC << " s" << endl;

	/*pcl::visualization::PCLVisualizer viewer("sac registration Viewer");
	//viewer.createViewPort (0.0, 0, 0.5, 1.0, vp_1);
	//viewer.createViewPort (0.5, 0, 1.0, 1.0, vp_2);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> src_h(cloud_src_o, 0, 255, 0);  //Դ���ƣ���ɫ
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> tgt_h(cloud_tgt_o, 255, 0, 0);   //Ŀ����ƣ���ɫ
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> final_h(sac_result, 0, 0, 255);  //ת����Դ���ƣ���ɫ
	viewer.addPointCloud(cloud_src_o, src_h, "source cloud");
	viewer.addPointCloud(cloud_tgt_o, tgt_h, "tgt cloud");
	viewer.addPointCloud(sac_result, final_h, "final cloud");
	viewer.spin();*/

	////icp��׼
	//pcl::PointCloud<pcl::PointXYZ>::Ptr icp_result(new pcl::PointCloud<pcl::PointXYZ>);
	//pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
	//icp.setInputSource(sac_result);
	//icp.setInputTarget(cloud_tgt);
	////Set the max correspondence distance to 4cm (e.g., correspondences with higher distances will be ignored)
	//icp.setMaxCorrespondenceDistance(0.04);
	//// ����������
	//icp.setMaximumIterations(50);
	//// ���α仯����֮��Ĳ�ֵ
	//icp.setTransformationEpsilon(1e-10);
	//// �������
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
	//////ʹ�ô����ı任��δ���˵�������ƽ��б任
	////pcl::transformPointCloud(*cloud_src_o, *icp_result, icp_trans);
	//////����ת�����������
	////pcl::io::savePCDFileASCII("transformed_sac_icp.pcd", *icp_result);
	//cout << "total time: " << (double)(end - start) / (double)CLOCKS_PER_SEC << " s" << endl;
	//pcl::visualization::PCLVisualizer viewer1("icp registration Viewer");
	////viewer.createViewPort (0.0, 0, 0.5, 1.0, vp_1);
	////viewer.createViewPort (0.5, 0, 1.0, 1.0, vp_2);
	//pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> src_hh(cloud_src, 0, 255, 0);  //Դ���ƣ���ɫ
	//pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> tgt_hh(cloud_tgt, 255, 0, 0);   //Ŀ����ƣ���ɫ
	//pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> final_hh(icp_result, 0, 0, 255);  //ת����Դ���ƣ���ɫ
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
	//icp��׼
	//pcl::PointCloud<pcl::PointXYZRGBA>::Ptr icp_result(new pcl::PointCloud<pcl::PointXYZRGBA>);
	pcl::IterativeClosestPoint<pcl::PointXYZRGBA, pcl::PointXYZRGBA> icp;
	icp.setInputSource(cloud_src);
	icp.setInputTarget(cloud_tgt);
	//Set the max correspondence distance to 4cm (e.g., correspondences with higher distances will be ignored)
	//icp.setMaxCorrespondenceDistance(0.04);
	// ����������
	icp.setMaximumIterations(100);
	// ���α仯����֮��Ĳ�ֵ
	//icp.setTransformationEpsilon(1e-10);
	// �������
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
	////ʹ�ô����ı任��δ���˵�������ƽ��б任
	//pcl::transformPointCloud(*cloud_src_o, *icp_result, icp_trans);
	////����ת�����������
	//pcl::io::savePCDFileASCII("transformed_sac_icp.pcd", *icp_result);
	//cout << "total time: " << (double)(end - start) / (double)CLOCKS_PER_SEC << " s" << endl;
	/*pcl::visualization::PCLVisualizer viewer1("icp registration Viewer");
	//viewer.createViewPort (0.0, 0, 0.5, 1.0, vp_1);
	//viewer.createViewPort (0.5, 0, 1.0, 1.0, vp_2);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> src_hh(cloud_src, 0, 255, 0);  //Դ���ƣ���ɫ
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> tgt_hh(cloud_tgt, 255, 0, 0);   //Ŀ����ƣ���ɫ
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> final_hh(icp_result, 0, 0, 255);  //ת����Դ���ƣ���ɫ
	viewer1.addPointCloud(cloud_src, src_hh, "source cloud");
	viewer1.addPointCloud(cloud_tgt, tgt_hh, "tgt cloud");
	viewer1.addPointCloud(icp_result, final_hh, "final cloud");
	viewer1.spin();*/

	//return icp_result;

}

//ransac ����׼
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
	std::cout << "����任��ĵ��Ƶ��� ransac_result.pcd" << std::endl;
	pcl::io::savePCDFileASCII("ransac_result.pcd", *ransac_result);
}




//ƥ��ģ�� ��֪ģ�壬����ԭ���ƣ����ƥ����
void temp_icp(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_sou)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_tar(new(pcl::PointCloud<pcl::PointXYZ>));
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_sou2(new(pcl::PointCloud<pcl::PointXYZ>));

	////��ȡģ��
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

	/*pcl::visualization::PCLVisualizer viewer("ICP��׼");

	// Ϊ���ƶ��� R,G,B ��ɫ
	//pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> source_cloud_color_handler(cloud_sou, 255, 255, 255);
	// ������Ƶ��鿴����ʹ����ɫ����
	viewer.addPointCloud(cloud_sou, "original_cloud");

	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> transformed_cloud_color_handler(cloud_sou2, 255, 0, 0); // ��
	viewer.addPointCloud(cloud_sou2, transformed_cloud_color_handler, "transformed_cloud");
	viewer.addPointCloud(cloud_tar, "target_cloud");

	//viewer.addCoordinateSystem(1.0, "cloud", 0);
	viewer.setBackgroundColor(0.05, 0.05, 0.05, 0); // ���ñ���Ϊ���
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "original_cloud");
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "transformed_cloud");
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "target_cloud");
	viewer.spin();*/
}


//��������任 Ĭ�ϵĵ�λ��
pcl::PointCloud<pcl::PointXYZ>::Ptr trans(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_start, Eigen::Matrix4f transform_1 = Eigen::Matrix4f::Identity())
{
	//Eigen::Matrix4f transform_1 = Eigen::Matrix4f::Identity();
	//float theta = M_PI / 4; // ���Ƚ�
	//transform_1(0, 0) = cos(theta);
	//transform_1(0, 1) = -sin(theta);
	//transform_1(1, 0) = sin(theta);
	//transform_1(1, 1) = cos(theta);
	////    //    	(��, ��)
	//transform_1(0, 3) = 0.5;
	//transform_1(1, 3) = 0.2;
	//transform_1(2, 3) = 0.1;

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_end(new(pcl::PointCloud<pcl::PointXYZ>));
	pcl::transformPointCloud(*cloud_start, *cloud_end, transform_1);
	return cloud_end;
}


//�������� ������ ͳ���˲� �ز��� ���ɷ��� ����
pcl::PolygonMesh grid(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
	// Load input file
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_downSampled(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_smoothed(new pcl::PointCloud<pcl::PointXYZ>);


	// ----------------------��ʼ��Ĵ���--------------------------//
	// ��ο�֮ǰ�����е����²������˲���ƽ�������ݣ��Լ�PCL����ʵ�����¹��ܡ����벻�ѡ�

	// �²���
	pcl::VoxelGrid<pcl::PointXYZ> downSampled;  //�����˲�����
	downSampled.setInputCloud(cloud);            //������Ҫ���˵ĵ��Ƹ��˲�����
	downSampled.setLeafSize(0.01f, 0.01f, 0.01f);  //�����˲�ʱ�������������Ϊ1cm��������
	downSampled.filter(*cloud_downSampled);           //ִ���˲������洢���
   // pcl::io::savePCDFile("/home/xiaohu/learn_SLAM/zuoye15/��ҵ15-����ƽ�������߹��Ƽ���ʾ/data/downsampledPC.pcd", *cloud_downSampled);

	// ͳ���˲�
	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> statisOutlierRemoval;       //�����˲�������
	statisOutlierRemoval.setInputCloud(cloud_downSampled);            //���ô��˲��ĵ���
	statisOutlierRemoval.setMeanK(50);                                //�����ڽ���ͳ��ʱ���ǲ�ѯ���ٽ�����
	statisOutlierRemoval.setStddevMulThresh(3.0);                     //�����ж��Ƿ�Ϊ��Ⱥ��ķ�ֵ:��ֵ+1.0*��׼��
	statisOutlierRemoval.filter(*cloud_filtered);                     //�˲�����洢��cloud_filtered
   // pcl::io::savePCDFile("/home/xiaohu/learn_SLAM/zuoye15/��ҵ15-����ƽ�������߹��Ƽ���ʾ/data/filteredPC.pcd", *cloud_filtered);

	// �Ե����ز���  
	pcl::search::KdTree<pcl::PointXYZ>::Ptr treeSampling(new pcl::search::KdTree<pcl::PointXYZ>);// �������������������KD-Tree
	pcl::PointCloud<pcl::PointXYZ> mls_point;    //���MLS
	pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ> mls; // ������С����ʵ�ֵĶ���mls
	mls.setComputeNormals(false);  //��������С���˼������Ƿ���Ҫ�洢����ķ���
	mls.setInputCloud(cloud_filtered);         //���ô��������
	mls.setPolynomialOrder(2);            // ���2�׶���ʽ���
	//mls.setPolynomialFit(false);     // ����Ϊfalse���� ���� smooth
	mls.setSearchMethod(treeSampling);         // ����KD-Tree��Ϊ��������
	mls.setSearchRadius(0.05);           // ��λm.����������ϵ�K���ڰ뾶
	mls.process(mls_point);                 //���
	// ����ز������
	cloud_smoothed = mls_point.makeShared();
	//std::cout << "cloud_smoothed: " << cloud_smoothed->size() << std::endl;
	//save cloud_smoothed
  //  pcl::io::savePCDFileASCII("/home/xiaohu/learn_SLAM/zuoye15/��ҵ15-����ƽ�������߹��Ƽ���ʾ/data/cloud_smoothed.pcd", *cloud_smoothed);


	// ���߹���
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimation;             //�������߹��ƵĶ���
	normalEstimation.setInputCloud(cloud_smoothed);                         //�������
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);// �������������������KD-Tree
	normalEstimation.setSearchMethod(tree);
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>); // ��������ĵ��Ʒ���
	// K����ȷ��������ʹ��k������㣬����ȷ��һ����rΪ�뾶��Բ�ڵĵ㼯��ȷ�������ԣ�����ѡ1����
	normalEstimation.setKSearch(10);// ʹ�õ�ǰ����Χ�����10����
	//normalEstimation.setRadiusSearch(0.03);            //����ÿһ���㶼�ð뾶Ϊ3cm�Ľ���������ʽ
	normalEstimation.compute(*normals);               //���㷨��
	// �������
	//std::cout << "normals: " << normals->size() << ", " << "normals fields: " << pcl::getFieldsList(*normals) << std::endl;
	//  pcl::io::savePCDFileASCII("/home/xiaohu/learn_SLAM/zuoye15/��ҵ15-����ƽ�������߹��Ƽ���ʾ/data/normals.pcd", *normals);

	  // ������λ�ˡ���ɫ��������Ϣ���ӵ�һ��
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
	pcl::concatenateFields(*cloud_smoothed, *normals, *cloud_with_normals);
	// pcl::io::savePCDFileASCII("/home/xiaohu/learn_SLAM/zuoye15/��ҵ15-����ƽ�������߹��Ƽ���ʾ/data/cloud_with_normals.pcd", *cloud_with_normals);

	 // ̰��ͶӰ���ǻ�
	 //��������������
	pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
	tree2->setInputCloud(cloud_with_normals);

	// ���ǻ�
	pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;   // �������ǻ�����
	pcl::PolygonMesh triangles; //�洢�������ǻ�������ģ��

	// �������ǻ�����
	gp3.setSearchRadius(0.1);  //��������ʱ�İ뾶��Ҳ����KNN����뾶
	gp3.setMu(2.5);  //������������������ڵ����Զ����Ϊ2.5��������ֵ2.5-3��������ʹ���㷨����Ӧ�����ܶȵı仯
	gp3.setMaximumNearestNeighbors(100);    //���������������������������������ֵ��50-100

	gp3.setMinimumAngle(M_PI / 18); // �������ǻ���õ����������ڽǵ���С�ĽǶ�Ϊ10��
	gp3.setMaximumAngle(2 * M_PI / 3); // �������ǻ���õ����������ڽǵ����Ƕ�Ϊ120��

	gp3.setMaximumSurfaceAngle(M_PI / 4); // ����ĳ�㷨�߷���ƫ�������㷨�ߵ����Ƕ�45�㣬�������������ʱ�����Ǹõ�
	gp3.setNormalConsistency(false);  //���øò���Ϊtrue��֤���߳���һ�£�����Ϊfalse�Ļ�������з���һ���Լ��

	gp3.setInputCloud(cloud_with_normals);     //�����������Ϊ�������
	gp3.setSearchMethod(tree2);   //����������ʽ
	gp3.reconstruct(triangles);  //�ؽ���ȡ���ǻ�

	// ��ʾ���񻯽��
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setBackgroundColor(0, 0, 0);  //
	viewer->addPolygonMesh(triangles, "wangge");  //
	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
	//pcl::io::savePLYFile("1", triangles);  //�ɴ�Ϊply�ļ�
	return triangles;
}


//����ת��Ϊ����
void mesh2cloud(pcl::PolygonMesh mesh, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
	//pcl::io::loadPolygonFilePLY("bunny.ply", mesh);
	fromPCLPointCloud2(mesh.cloud, *cloud);
}

//Ԥ����������+�˲������浽cr��cl 
void pre(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2, std::string xyz, std::string xyz1)
{
	pcl::PolygonMesh mesh;
	pcl::PolygonMesh mesh1;


	if (pcl::io::loadPolygonFileOBJ(xyz, mesh) == -1)//*�򿪵����ļ�  xyzΪply�ļ���
	{
		PCL_ERROR("Couldn't read file\n");
		exit(0);
	}
	pcl::fromPCLPointCloud2(mesh.cloud, *cloud1);

	if (pcl::io::loadPolygonFileOBJ(xyz1, mesh1) == -1)//*�򿪵����ļ�  xyzΪply�ļ���
	{
		PCL_ERROR("Couldn't read file\n");
		exit(0);
	}
	pcl::fromPCLPointCloud2(mesh1.cloud, *cloud2);


	//�²���
	pcl::VoxelGrid<pcl::PointXYZ> down_filter;
	down_filter.setLeafSize(0.01f, 0.01f, 0.01f);
	down_filter.setInputCloud(cloud1);
	down_filter.filter(*cloud1);

	pcl::VoxelGrid<pcl::PointXYZ> down_filter2;
	down_filter2.setLeafSize(0.01f, 0.01f, 0.01f);
	down_filter2.setInputCloud(cloud2);
	down_filter2.filter(*cloud2);


	//ͳ���˲�

	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
	sor.setInputCloud(cloud1);
	sor.setMeanK(80);                //ÿ����Ҫ�������ھ�������Ϊ50����׼���������Ϊ1��r=50  n=1.0
	sor.setStddevMulThresh(0.8);    //����ζ�����о����ѯ���ƽ�����������1��׼������е㶼�����Ϊ�쳣ֵ��ɾ��
	sor.filter(*cloud1);     //ʱ��޳���������
	//sor.setNegative(true);


	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor2;
	sor2.setInputCloud(cloud2);
	sor2.setMeanK(30);                //ÿ����Ҫ�������ھ�������Ϊ50����׼���������Ϊ1��r=50  n=1.0
	sor2.setStddevMulThresh(0.05);    //����ζ�����о����ѯ���ƽ�����������1��׼������е㶼�����Ϊ�쳣ֵ��ɾ��
	sor2.filter(*cloud2);     //ʱ��޳���������


}

//��Ե��ȡ+�˲�
void boundary(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr boundPoints)
{
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	pcl::PointCloud<pcl::Boundary> boundaries;
	pcl::BoundaryEstimation<pcl::PointXYZ, pcl::Normal, pcl::Boundary> est;
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());

	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normEst;  //����pcl::PointXYZ��ʾ�����������ݣ�pcl::Normal��ʾ�������,��pcl::Normalǰ�����Ƿ������һ��������
	normEst.setInputCloud(cloud);
	normEst.setSearchMethod(tree);
	// normEst.setRadiusSearch(2);  //������Ƶİ뾶
	normEst.setKSearch(90);  //������Ƶĵ���        Խ��Խ�� ò��Ҫ���ı�������
	normEst.compute(*normals);

	//normal_est.setViewPoint(0,0,0); //���Ӧ�û�ʹ����һ��
	est.setInputCloud(cloud);
	est.setInputNormals(normals);
	//  est.setAngleThreshold(90);
	//   est.setSearchMethod (pcl::search::KdTree<pcl::PointXYZ>::Ptr (new pcl::search::KdTree<pcl::PointXYZ>));
	est.setSearchMethod(tree);
	est.setKSearch(200);  //һ���������ֵԽ�ߣ����ձ߽�ʶ��ľ���Խ��
	//est.setRadiusSearch(everagedistance);  //�����뾶
	est.compute(boundaries);

	int countBoundaries = 0;
	for (int i = 0; i < cloud->size(); i++)
	{
		uint8_t x = (boundaries.points[i].boundary_point);
		int a = static_cast<int>(x); //�ú����Ĺ�����ǿ������ת��
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
	sor.setMeanK(30);                //ÿ����Ҫ�������ھ�������Ϊ50����׼���������Ϊ1��r=50  n=1.0
	sor.setStddevMulThresh(0.05);    //����ζ�����о����ѯ���ƽ�����������1��׼������е㶼�����Ϊ�쳣ֵ��ɾ��
	sor.filter(*boundPoints);     //ʱ��޳�����������Ե�˲������ 

}

//�����άԲ��ʹԭ�����غ�
void match(pcl::PointCloud<pcl::PointXYZ>::Ptr source, pcl::PointCloud<pcl::PointXYZ>::Ptr target, pcl::PointCloud<pcl::PointXYZ>::Ptr source1, Eigen::Matrix4f& transform_ltr)
{
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);  //inliers��ʾ��������̵ĵ� ��¼���ǵ��Ƶ����
	pcl::SACSegmentation<pcl::PointXYZ> seg;     // ����һ���ָ���
	seg.setOptimizeCoefficients(true);      // Optional��������ÿ���ѡ�����ƽ��չʾ�ĵ��Ƿָ���ĵ㻹�Ƿָ�ʣ�µĵ㡣
	//seg.setModelType(pcl::SACMODEL_LINE);  // Mandatory-����Ŀ�꼸����״ ֱ�� (x-a)/d=(y-b)/e=(z-c)/f 
	seg.setModelType(pcl::SACMODEL_CIRCLE3D);  // Mandatory-����Ŀ�꼸����״  3D��Բ
	//seg.setModelType(pcl::SACMODEL_PLANE);

	seg.setMethodType(pcl::SAC_RANSAC);     //�ָ�������������
	seg.setDistanceThreshold(0.1);         //����������̷�Χ��Ҳ������ֵ0.001
	seg.setInputCloud(source);               //�������
	seg.segment(*inliers, *coefficients);   //�ָ���ƣ����ƽ��ͷ�����
	float x1 = coefficients->values[0];
	float y1 = coefficients->values[1];
	float z1 = coefficients->values[2];
	float r = coefficients->values[3];
	float x2 = coefficients->values[4];
	float y2 = coefficients->values[5];
	float z2 = coefficients->values[6];

	/*//��ӡֱ�߷���
	std::cout << "(" << x1 << "," << y1 << "," << z1 << ")" << std::endl;
	std::cout << "�뾶��" << r << std::endl;
	std::cout << "(" << x2 << "," << y2 << "," << z2 << ")" << std::endl;*/


	Eigen::Vector3d r1(0, 0, 0);   //ͼ1Բ������
	r1(0) = x1; r1(1) = y1; r1(2) = z1;

	seg.setInputCloud(target);               //�������
	seg.segment(*inliers, *coefficients);   //�ָ���ƣ����ƽ��ͷ�����
	float x3 = coefficients->values[0];
	float y3 = coefficients->values[1];
	float z3 = coefficients->values[2];
	float rr = coefficients->values[3];
	float x4 = coefficients->values[4];
	float y4 = coefficients->values[5];
	float z4 = coefficients->values[6];

	Eigen::Vector3d r2(0, 0, 0);   //ͼ1Բ������
	r2(0) = x3; r2(1) = y3; r2(2) = z3;

	Eigen::Matrix4f transform_1;      //ƽ�ƾ���
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

//icp��׼
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

/*//����ƽ���
float pmd(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{

	pcl::PointCloud<pcl::PointXYZ>::Ptr plane(new pcl::PointCloud<pcl::PointXYZ>);
	float Threshold = 0.1;

	//����һ��ģ�Ͳ����������ڼ�¼���
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);  //inliers��ʾ��������̵ĵ� ��¼���ǵ��Ƶ����
	pcl::SACSegmentation<pcl::PointXYZ> seg;     // ����һ���ָ���
	seg.setOptimizeCoefficients(true);      // Optional��������ÿ���ѡ�����ƽ��չʾ�ĵ��Ƿָ���ĵ㻹�Ƿָ�ʣ�µĵ㡣
	//seg.setModelType(pcl::SACMODEL_LINE);  // Mandatory-����Ŀ�꼸����״ ֱ�� (x-a)/d=(y-b)/e=(z-c)/f 
	seg.setModelType(pcl::SACMODEL_PLANE);  // Mandatory-����Ŀ�꼸����״  ƽ�� ax+by+cz+d=0

	seg.setMethodType(pcl::SAC_RANSAC);     //�ָ�������������
	seg.setDistanceThreshold(Threshold);         //����������̷�Χ��Ҳ������ֵ0.001
	seg.setInputCloud(cloud);               //�������
	seg.segment(*inliers, *coefficients);   //�ָ���ƣ����ƽ��ͷ�����

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
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);  //inliers��ʾ��������̵ĵ� ��¼���ǵ��Ƶ����
	pcl::SACSegmentation<pcl::PointXYZ> seg;     // ����һ���ָ���
	seg.setOptimizeCoefficients(true);      // Optional��������ÿ���ѡ�����ƽ��չʾ�ĵ��Ƿָ���ĵ㻹�Ƿָ�ʣ�µĵ㡣
	//seg.setModelType(pcl::SACMODEL_LINE);  // Mandatory-����Ŀ�꼸����״ ֱ�� (x-a)/d=(y-b)/e=(z-c)/f 
	seg.setModelType(pcl::SACMODEL_CIRCLE3D);  // Mandatory-����Ŀ�꼸����״  3D��Բ
	//seg.setModelType(pcl::SACMODEL_PLANE);

	seg.setMethodType(pcl::SAC_RANSAC);     //�ָ�������������
	seg.setDistanceThreshold(0.1);         //����������̷�Χ��Ҳ������ֵ0.001
	seg.setInputCloud(circle);               //�������
	seg.segment(*inliers, *coefficients);   //�ָ���ƣ����ƽ��ͷ�����
	float x1 = coefficients->values[0];
	float y1 = coefficients->values[1];
	float z1 = coefficients->values[2];
	float r = coefficients->values[3];
	float x2 = coefficients->values[4];
	float y2 = coefficients->values[5];
	float z2 = coefficients->values[6];

	//��ӡֱ�߷���
	std::cout << "Բ�ģ�(" << x1 << "," << y1 << "," << z1 << ")" << std::endl;
	std::cout << "�뾶��" << r << std::endl;
	std::cout << "����(" << x2 << "," << y2 << "," << z2 << ")" << std::endl;

}

////�������μ�� 0Ϊ������ 1Ϊ����
//bool liuxingjiance(std::string document)
//{
//	//GPP::TriMesh* tri = GPP::Parser::ImportTriMesh("C:/Users/sjtuzhy/Desktop/ply�ļ�/����A��2h.ply");
//	GPP::TriMesh* tri = GPP::Parser::ImportTriMesh(document);
//	bool isTopo = 0;
//	isTopo = GPP::ComputeMeshTopology::IsTriMeshManifold(tri);
//	return isTopo;
//
//}