#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/visualization/pcl_visualizer.h>

int main(int argc, char** argv)
{
	//��ply��ʽת��Ϊpcd��ʽ
	pcl::PLYReader reader;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudyjp(new pcl::PointCloud<pcl::PointXYZ>);
	reader.read<pcl::PointXYZ>("E:/example/greedy_projection/greedy_projection/greedy_projection/bun000.ply", *cloudyjp);
	pcl::io::savePCDFile("E:/example/greedy_projection/greedy_projection/greedy_projection/bun0.pcd", *cloudyjp);

	//���������ļ�����ʽҪƥ��
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PCLPointCloud2 cloud_blob;
	pcl::io::loadPCDFile("bun0.pcd", cloud_blob);
	pcl::fromPCLPointCloud2(cloud_blob, *cloud);
	//* ������Ҫ�ȶ���pclpointcloud2�У����õĵ���cloud
	
	//���߹���
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud(cloud);
	n.setInputCloud(cloud);
	n.setSearchMethod(tree);
	n.setKSearch(20);
	n.compute(*normals);
	//* ���߲���������

	//ƴ��XYZ�ͷ���
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
	pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);
	//* cloud_with_normals=cloud +normals

	//����������
	pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
	tree2->setInputCloud(cloud_with_normals);

	//��ʼ��Ŀ��
	pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
	pcl::PolygonMesh triangles;

	//���������뾶(���߳���)
	gp3.setSearchRadius(0.025);
	//����̰������ͶӰ�㷨�Ĳ���ֵ
	gp3.setMu(2.5);
	gp3.setMaximumNearestNeighbors(100);
	gp3.setMaximumSurfaceAngle(M_PI / 4);//45��
	gp3.setMinimumAngle(M_PI / 18);//10��
	gp3.setMaximumAngle(2 * M_PI / 3);//120��
	gp3.setNormalConsistency(false);

	//��ȡ���
	gp3.setInputCloud(cloud_with_normals);
	gp3.setSearchMethod(tree2);
	gp3.reconstruct(triangles);

	//����Ķ�����Ϣ
	std::vector<int> parts = gp3.getPartIDs();
	std::vector<int> states = gp3.getPointStates();

	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setBackgroundColor(0, 0, 0);

	viewer->addPolygonMesh(triangles, "triangles");
	viewer->addCoordinateSystem(1.0);
	viewer->initCameraParameters();
	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}

	return 0;
}