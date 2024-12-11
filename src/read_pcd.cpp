#include<ros/ros.h>
#include<pcl/point_cloud.h>
#include<pcl_conversions/pcl_conversions.h>
#include<sensor_msgs/PointCloud2.h>
#include<pcl/io/pcd_io.h>
 
int main(int argc,char **argv){
    ros::init(argc,argv,"pcd_pub");
    ros::NodeHandle nh;
    ros::Publisher pcl_pub=nh.advertise<sensor_msgs::PointCloud2> ("pcl_output",1);
    pcl::PointCloud<pcl::PointXYZ> cloud;
    sensor_msgs::PointCloud2 output;
    std::string file_path;
    nh.param<std::string>("file_path", file_path, "/pcd/data_1/0000000001.pcd");
    pcl::io::loadPCDFile(file_path,cloud);//通过launch文件修改路径即可
 
    pcl::toROSMsg(cloud,output);
    output.header.frame_id="world";// map表示rviz的fixed frame
 
    ros::Rate loop_rate(1);
    while (ros::ok())
    {
        pcl_pub.publish(output);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}

