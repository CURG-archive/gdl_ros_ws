#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <boost/shared_ptr.hpp>
#include <pcl_ros/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/surface/organized_fast_mesh.h>
#include "std_msgs/String.h"
#include "sensor_msgs/PointCloud2.h"
#include "mesh_builder/MeshCloud.h"
#include <pcl/common/projection_matrix.h>
//#include "perception_msgs/SegmentedObject.h"
//#include "perception_msgs/SegmentedObjectList.h"
//#include "perception_msgs/ObjectCenterProperty.h"
#include <sstream>
#include "cluster_extractor.h"


namespace mesh_builder_node
{
    class MeshBuilderNode
    {
        private:
            ros::NodeHandle node_handle;

            ros::ServiceServer service;

            bool serviceCallback( mesh_builder::MeshCloud::Request &req,
                                  mesh_builder::MeshCloud::Response &res );

        public:
            MeshBuilderNode();
    };


    MeshBuilderNode::MeshBuilderNode(): node_handle("")
    {

        service = node_handle.advertiseService("/meshCloud", &MeshBuilderNode::serviceCallback, this);

        ROS_INFO("mesh_builder_node ready");
    }


    bool MeshBuilderNode::serviceCallback( mesh_builder::MeshCloud::Request &req,
                                           mesh_builder::MeshCloud::Response &res )
    {
        ROS_INFO("recieved message");

        pcl::PCLPointCloud2::Ptr pcl_pc (new pcl::PCLPointCloud2());
        pcl_conversions::toPCL(req.input_cloud, *pcl_pc);

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>());
        pcl::fromPCLPointCloud2(*pcl_pc, *cloud);

        //scale the cloud for GraspIt!
        pcl::PointCloud<pcl::PointXYZRGB>::iterator p;
        for (p = cloud->points.begin(); p < cloud->points.end(); p++)
        {
            p->x *= 1000;
            p->y *= 1000;
            p->z *= 1000;
        }

        //extract clusters
        ClusterExtractor *clusterExtractor = new ClusterExtractor();
        clusterExtractor->setCloud(cloud);
        clusterExtractor->computeClusters();
        std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> cloudClusters = clusterExtractor->getCloudClusters();
        delete clusterExtractor;


        for(int i = 0; i < cloudClusters.size(); i++)
        {

            std::ostringstream model_name;
            model_name << "model_" << i;

            pcl::OrganizedFastMesh<pcl::PointXYZRGB > orgMesh;
            pcl::PolygonMesh triangles;

            orgMesh.setTriangulationType(pcl::OrganizedFastMesh<pcl::PointXYZRGB>::TRIANGLE_ADAPTIVE_CUT  );
            orgMesh.setInputCloud(cloudClusters.at(i));
            orgMesh.reconstruct(triangles);

            pcl::io::savePolygonFileSTL(req.output_filepath + model_name.str() + ".stl", triangles);

            res.segmented_mesh_filenames.push_back(model_name.str());
        }

        // This will output cloud as one large mesh
//        pcl::OrganizedFastMesh<pcl::PointXYZRGB > orgMesh;
//        pcl::PolygonMesh triangles;

//        orgMesh.setTriangulationType(pcl::OrganizedFastMesh<pcl::PointXYZRGB>::TRIANGLE_ADAPTIVE_CUT  );
//        orgMesh.setInputCloud(cloud);
//        orgMesh.reconstruct(triangles);

//        pcl::io::savePolygonFileSTL(req.output_filepath + "out.stl", triangles);

//        res.segmented_mesh_filenames.push_back("out");



        ROS_INFO("saved_file");
        return true;
    }


}



int main(int argc, char **argv)
{

  ros::init(argc, argv, "mesh_builder_node");
  ros::NodeHandle nh;

  mesh_builder_node::MeshBuilderNode node;



  ros::spin();

  return 0;
}
