#include <PlaneSegmentation/PlaneSegmentation.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/sac_model.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl_ros/transforms.h>
using namespace std;
using namespace cv;
//! Callback for processing the Point Cloud data
void PlaneSegmentation::processCloud(const sensor_msgs::PointCloud2ConstPtr &var)
{

    pcl::PointCloud< pcl::PointXYZ > pc; // internal data
    pcl::fromROSMsg(*var,pc); //#>>>>TODO: Convert the data to the internal var (pc) using pcl function: fromROSMsg
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = pc.makeShared(); // cloud to operate
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered( new pcl::PointCloud<pcl::PointXYZ> ); // cloud to store the filter the data



    std::cout << "PointCloud before filtering has: " << pc.points.size() << " data points." << std::endl; 
    std::cout << "width: " << pc.width << "height: " << pc.height << std::endl;


    //#>>>>TODO: Down sample the pointcloud using VoxelGrid
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    vg.setInputCloud(cloud); //original pointcloud
    vg.setLeafSize(0.005f, 0.005f,0.005f);
    vg.filter(*cloud_filtered);//save

    //#>>>>TODO: Trim points lower than z=0.1 to remove the floor from the point cloud.
    // Hint: use pcl::PassThrough filter.
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud (cloud_filtered);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (0, 1.3);//not sure about the zmax.
    pass.filter (*cloud_filtered);
    
    std::cout << "PointCloud after filtering has: " << cloud_filtered->points.size()  << " data points." << std::endl;
    // Create the segmentation object for the plane model and set all the parameters using pcl::SACSegmentation<pcl::PointXYZ>
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    pcl::PointIndices::Ptr inliers( new pcl::PointIndices );
    pcl::ModelCoefficients::Ptr coefficients( new pcl::ModelCoefficients );
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane( new pcl::PointCloud<pcl::PointXYZ>() );

    //#>>>>TODO: set parameters of the SACS segmentation
    seg.setOptimizeCoefficients(true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.01); // need to be adjusted
    seg.setMaxIterations(1000);  // need to be adjusted
    // Create pointcloud to publish inliers
    // Segment the planes 
    seg.setInputCloud (cloud_filtered);
    seg.segment (*inliers, *coefficients);
    pcl::ExtractIndices<pcl::PointXYZ> ext;

    ext.setInputCloud (cloud_filtered);
    ext.setIndices (inliers);
    ext.setNegative (false);
    ext.filter (curr_table_pc);
    ext.setNegative (true);
    ext.setKeepOrganized (true);
    ext.filter (curr_clusters_pc);
    
    //#>>>>TODO: Publish biggest plane
    pub_plane_pc_.publish(curr_table_pc);
    //#>>>>TODO: Publish other clusters
    pub_clusters_pc_.publish(curr_clusters_pc);
    return;

}

