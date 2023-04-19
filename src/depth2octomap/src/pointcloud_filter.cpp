#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
 
// PCL 库
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/point_cloud.h>
// #include <pcl/visualization/pcl_visualizer.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/transforms.h>

#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <Eigen/Eigen>
 
// 定义点云类型
// typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;


// camera_info话题读取的相机内参
const double camera_factor = 1000;
const double camera_cx = 331.2972717285156;
const double camera_cy = 243.7368927001953;
const double camera_fx = 606.3751831054688;
const double camera_fy = 604.959716796875;
 
// 全局变量：图像矩阵和点云
cv_bridge::CvImagePtr color_ptr, depth_ptr;
cv::Mat color_pic, depth_pic;

void color_Callback(const sensor_msgs::ImageConstPtr& color_msg);
void depth_Callback(const sensor_msgs::ImageConstPtr& depth_msg);
void pic2cloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud);
void cam2base(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cam_pc, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &base_pc);
void cull_self(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &full_pc, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cull_pc);


int main(int argc, char **argv)
{
    ros::init(argc, argv, "publish_octomap");
    ros::NodeHandle nh;
    
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe("/camera/color/image_raw", 1, color_Callback);
    image_transport::Subscriber sub1 = it.subscribe("/camera/depth/image_rect_raw", 1, depth_Callback);
    ros::Publisher pointcloud_publisher = nh.advertise<sensor_msgs::PointCloud2>("/pointcloud/output", 1);
    
    // 点云变量
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr raw_pc(new pcl::PointCloud<pcl::PointXYZRGB>); // 原始有色点云
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr vox_pc(new pcl::PointCloud<pcl::PointXYZRGB>); // 体素滤波后的点云
    // pcl::PointCloud<pcl::PointXYZRGB>::Ptr noNaN_pc(new pcl::PointCloud<pcl::PointXYZRGB>); // 之后会直通滤波
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr base_pc(new pcl::PointCloud<pcl::PointXYZRGB>); // 世界坐标系的点云
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pass_pc1(new pcl::PointCloud<pcl::PointXYZRGB>); // 直通滤波后的点云
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pass_pc2(new pcl::PointCloud<pcl::PointXYZRGB>); // 直通滤波后的点云
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pass_pc3(new pcl::PointCloud<pcl::PointXYZRGB>); // 直通滤波后的点云
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cull_pc(new pcl::PointCloud<pcl::PointXYZRGB>); // 剔除自身机械臂后的点云

    sensor_msgs::PointCloud2 pub_pointcloud;

    ros::Rate loop_rate(100.0); // use to regulate loop rate
 
    while (ros::ok()) {
        
        // 图像转化为点云
        pic2cloud(raw_pc);
 
        // 设置点云
        raw_pc->height = 1;
        raw_pc->width = raw_pc->points.size();
        ROS_INFO("point cloud size = %i",raw_pc->width);
        raw_pc->is_dense = false;

        // // 体素滤波
        // pcl::VoxelGrid<pcl::PointXYZRGB> vox;
        // vox.setInputCloud(raw_pc);            // 输入点云
        // vox.setLeafSize(0.02f, 0.02f, 0.02f); // 体素滤波器，单位m
        // vox.filter(*vox_pc);                  // 体素滤波后的点云
        
        ROS_INFO("PointCloud before Voxel Filtering: %i data points.",(raw_pc->width * raw_pc->height));
        // ROS_INFO("PointCloud after Voxel Filtering: %i data points.",(vox_pc->width * vox_pc->height));
        
        // 坐标系转换：/camera_link to /base_link
        // cam2base(raw_pc,base_pc);
        // cam2base(vox_pc,base_pc);

        // // 直通滤波
        // pcl::PassThrough<pcl::PointXYZRGB> pass1;
        // pass1.setInputCloud(base_pc);
        // pass1.setFilterFieldName("z");
        // pass1.setFilterLimits(0.2,2.2);
        // pass1.filter(*pass_pc1);

        // pcl::PassThrough<pcl::PointXYZRGB> pass2;
        // pass2.setInputCloud(base_pc);
        // pass2.setFilterFieldName("x");
        // pass2.setFilterLimits(-0.2,1.0);
        // pass2.filter(*pass_pc2);

        // pcl::PassThrough<pcl::PointXYZRGB> pass3;
        // pass3.setInputCloud(base_pc);
        // pass3.setFilterFieldName("y");
        // pass3.setFilterLimits(-1.2,1.2);
        // pass3.filter(*pass_pc3);

        // 机械臂自身点云剔除
        // cull_self(pass_pc3,cull_pc);

        pcl::toROSMsg(*raw_pc,pub_pointcloud); // ***
        // pcl::toROSMsg(*pass_pc3,pub_pointcloud);
        pub_pointcloud.header.frame_id = "camera_link";
        pub_pointcloud.header.stamp = ros::Time::now();
        // 发布合成点云
        pointcloud_publisher.publish(pub_pointcloud);
        // 清除数据并退出
        raw_pc->points.clear();  // ***
 
        ros::spinOnce(); // call all of the awaiting callback() functions
        loop_rate.sleep(); // wait for remainder of specified period;
    }
}


/***  RGB处理  ***/
void color_Callback(const sensor_msgs::ImageConstPtr& color_msg)
{
    try
    {
        color_ptr = cv_bridge::toCvCopy(color_msg, sensor_msgs::image_encodings::BGR8);
        cv::waitKey(1050); // 不断刷新图像，频率时间为int delay，单位为ms
    }
    catch (cv_bridge::Exception& e )
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", color_msg->encoding.c_str());
    }
    color_pic = color_ptr->image;
}
 
/***  Depth处理  ***/
void depth_Callback(const sensor_msgs::ImageConstPtr& depth_msg)
{
    try
    {
        depth_ptr = cv_bridge::toCvCopy(depth_msg, sensor_msgs::image_encodings::TYPE_32FC1);
        cv::waitKey(1050);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'mono16'.", depth_msg->encoding.c_str());
    }
 
    depth_pic = depth_ptr->image;
}

/*** 点云读取 ***/
void pic2cloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud)
{
    for (int m = 0; m < depth_pic.rows; m++)
    {
        for (int n = 0; n < depth_pic.cols; n++)
        {
            // 获取深度图中(m,n)处的值
            float d = depth_pic.ptr<float>(m)[n];//ushort d = depth_pic.ptr<ushort>(m)[n];
            // d 可能没有值，若如此，跳过此点
            if (d == 0)
                continue;
            // d 存在值，则向点云增加一个点
            pcl::PointXYZRGB p;

            // // 相机模型是垂直的
            // p.x = double(d) / camera_factor;
            // p.y = -(n - camera_cx) * p.x / camera_fx;
            // p.z = -(m - camera_cy) * p.x / camera_fy;

            // 考虑坐标系转换
            p.z = double(d) / camera_factor;
            p.x = (n - camera_cx) * p.z / camera_fx;
            p.y = (m - camera_cy) * p.z / camera_fy;

            // 从rgb图像中获取它的颜色
            // rgb是三通道的BGR格式图，所以按下面的顺序获取颜色
            p.b = color_pic.ptr<uchar>(m)[n*3];
            p.g = color_pic.ptr<uchar>(m)[n*3+1];
            p.r = color_pic.ptr<uchar>(m)[n*3+2];

            // 把p加入到点云中
            cloud->points.push_back( p );
        }
    }
}

/*** 坐标转换 ***/
void cam2base(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cam_pc, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &base_pc)
{
    tf::TransformListener cam_listener; // not sure should be static or not
    tf::StampedTransform cam_trans;

    Eigen::Matrix4d cam_trans_mat;

    try{
        cam_listener.waitForTransform("/base_link", "/camera_link",ros::Time(0.0),ros::Duration(1.0));
        cam_listener.lookupTransform("/base_link", "/camera_link",ros::Time(0.0),cam_trans);
    }
    catch(tf::TransformException &ex)
    {
        ROS_ERROR("camera_link: %s",ex.what());
        ros::Duration(0.5).sleep();
        return;
    }

    cam_trans_mat.block<3,1>(0,3) << cam_trans.getOrigin().getX(), cam_trans.getOrigin().getY(), cam_trans.getOrigin().getZ();
    
    Eigen::Quaterniond eigen_cam_quat(cam_trans.getRotation().getW(),cam_trans.getRotation().getX(),cam_trans.getRotation().getY(),cam_trans.getRotation().getZ());
    
    cam_trans_mat.block<3,3>(0,0) << eigen_cam_quat.toRotationMatrix();

    // Eigen::Translation3d tl_btol(cam_trans.getOrigin().getX(), cam_trans.getOrigin().getY(), cam_trans.getOrigin().getZ());
    // double roll, pitch, yaw;
    // tf::Matrix3x3(cam_trans.getRotation()).getEulerYPR(yaw, pitch, roll);
    // Eigen::AngleAxisd rot_x_btol(roll, Eigen::Vector3d::UnitX());
    // Eigen::AngleAxisd rot_y_btol(pitch, Eigen::Vector3d::UnitY());
    // Eigen::AngleAxisd rot_z_btol(yaw, Eigen::Vector3d::UnitZ());
    // cam_trans_mat = (tl_btol * rot_z_btol * rot_y_btol * rot_x_btol).matrix();
    // pcl::transformPointCloud(*cam_pc, *base_pc, cam_trans_mat);

}

/*** 机械臂自身点云剔除 ***/
void cull_self(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &full_pc, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cull_pc)
{
    return;
}