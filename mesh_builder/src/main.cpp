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
#include <pcl/common/centroid.h>
#include "geometry_msgs/Vector3.h"



//#include <pcl/kdtree/kdtree_flann.h>
//#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>


namespace mesh_builder_node
{
    class MeshBuilderNode
    {
        private:
            ros::NodeHandle node_handle;
            bool extract_clusters;

            ros::ServiceServer service;

            bool serviceCallback( mesh_builder::MeshCloud::Request &req,
                                  mesh_builder::MeshCloud::Response &res );

        public:
            MeshBuilderNode();
    };


    MeshBuilderNode::MeshBuilderNode():
        node_handle(""),
        extract_clusters(true)
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

        // This will output cloud as one large mesh
        pcl::OrganizedFastMesh<pcl::PointXYZRGB > orgMesh;
        pcl::PolygonMesh triangles;

        orgMesh.setTriangulationType(pcl::OrganizedFastMesh<pcl::PointXYZRGB>::TRIANGLE_ADAPTIVE_CUT  );
        orgMesh.setInputCloud(cloud);
        orgMesh.reconstruct(triangles);



        if(extract_clusters)
        {
            //extract clusters
            ClusterExtractor *clusterExtractor = new ClusterExtractor();
            clusterExtractor->setCloud(cloud);
            clusterExtractor->computeClusters();
            std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> cloudClusters = clusterExtractor->getCloudClusters();
            delete clusterExtractor;


            int mesh_index = 0;
            for(int i = 0; i < cloudClusters.size(); i++)
            {
                pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (cloudClusters.at(i));


                if (cloud->size() > 10)
                {
                    std::ostringstream model_name;
                    model_name << "model_" << mesh_index;

                    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz (new pcl::PointCloud<pcl::PointXYZ>());
                    // [...]
                    pcl::copyPointCloud(*cloud,*cloud_xyz);

                    // Normal estimation*
                     pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
                     pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
                     pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
                     tree->setInputCloud (cloud_xyz);
                     n.setInputCloud (cloud_xyz);
                     n.setSearchMethod (tree);
                     n.setKSearch (20);
                     n.compute (*normals);
                     n.setViewPoint(0,0,0);

                     //* normals should not contain the point normals + surface curvatures

                     // Concatenate the XYZ and normal fields*
                     pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
                     pcl::concatenateFields (*cloud_xyz, *normals, *cloud_with_normals);
                     //* cloud_with_normals = cloud + normals


                     Eigen::Vector4f centroid (0.f, 0.f, 0.f, 1.f);
                     pcl::compute3DCentroid (*cloud, centroid); centroid.w () = 1.f;

                     pcl::PointCloud<pcl::PointNormal>::Ptr centered_cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);

                     pcl::PointCloud<pcl::PointNormal>::iterator p;
                     for (p = cloud_with_normals->points.begin(); p < cloud_with_normals->points.end(); p++)
                     {
                         pcl::PointNormal *point = new pcl::PointNormal;
                         point->x = p->x - centroid.x();
                         point->y = p->y - centroid.y();
                         point->z = p->z - centroid.z();
                         centered_cloud_with_normals->points.push_back(*point);
                     }

                     // Create search tree*
                     pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
                     tree2->setInputCloud (centered_cloud_with_normals);

                     // Initialize objects
                     pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
                     pcl::PolygonMesh triangles;

                     // Set the maximum distance between connected points (maximum edge length)
                     gp3.setSearchRadius (25);

                     // Set typical values for the parameters
                     gp3.setMu (2.5);
                     gp3.setMaximumNearestNeighbors (100);
                     gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
                     gp3.setMinimumAngle(M_PI/18); // 10 degrees
                     gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
                     gp3.setNormalConsistency(true);
                     gp3.setConsistentVertexOrdering(true);

                     // Get result
                     gp3.setInputCloud (centered_cloud_with_normals);
                     gp3.setSearchMethod (tree2);
                     gp3.reconstruct (triangles);

                    pcl::io::savePolygonFileSTL(req.output_filepath + model_name.str() + ".stl", triangles);

                    res.segmented_mesh_filenames.push_back(model_name.str());
                    geometry_msgs::Vector3 *offset = new geometry_msgs::Vector3;
                    offset->x = -centroid.x();
                    offset->y = -centroid.y();
                    offset->z = -centroid.z();
                    res.offsets.push_back(*offset);

                    mesh_index++;
                }

            }
        }

        //save scene as one big mesh as well.
        pcl::io::savePolygonFileSTL(req.output_filepath + "single_mesh.stl", triangles);
        res.segmented_mesh_filenames.push_back("single_mesh");
        geometry_msgs::Vector3 *offset = new geometry_msgs::Vector3;
        offset->x = 0;
        offset->y = 0;
        offset->z = 0;
        res.offsets.push_back(*offset);




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
