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

#include "moveit_msgs/PlanningSceneWorld.h"
#include "moveit_msgs/PlanningScene.h"
#include "shape_msgs/Mesh.h"
#include "geometric_shapes/shapes.h"
#include "geometric_shapes/mesh_operations.h"


//#include <pcl/kdtree/kdtree_flann.h>
//#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <sys/time.h>

namespace mesh_builder_node
{
    class MeshBuilderNode
    {
        private:
            ros::NodeHandle node_handle;
            bool extract_clusters;

            //ros::Publisher planning_scene_diff_publisher;
            ros::ServiceServer service;

            bool serviceCallback( mesh_builder::MeshCloud::Request &req,
                                  mesh_builder::MeshCloud::Response &res );

        public:
            MeshBuilderNode();
    };


    MeshBuilderNode::MeshBuilderNode():
        node_handle("mesh_builder_node"),
        extract_clusters(true)
    {

        //planning_scene_diff_publisher = node_handle.advertise<moveit_msgs::PlanningScene>("/planning_scene", 1);
        service = node_handle.advertiseService("/meshCloud", &MeshBuilderNode::serviceCallback, this);

        ROS_INFO("mesh_builder_node ready");
    }


    bool MeshBuilderNode::serviceCallback( mesh_builder::MeshCloud::Request &req,
                                           mesh_builder::MeshCloud::Response &res )
    {
        ROS_INFO("recieved message");

        struct timeval startTime;
        gettimeofday(&startTime, NULL);
        long int startTimeMS = startTime.tv_sec * 1000 + startTime.tv_usec / 1000;
:


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
                pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudCluster (cloudClusters.at(i));


                if (cloudCluster->size() > 10)
                {
                    std::ostringstream model_name;
                    model_name << "model_" << mesh_index;

                    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_xyz (new pcl::PointCloud<pcl::PointXYZRGB>());
                    // [...]
                    pcl::copyPointCloud(*cloudCluster,*cloud_xyz);

                    // Normal estimation*
                     pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> n;
                     pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
                     pcl::search::KdTree<pcl::PointXYZRGB>::Ptr normalEstimationTree (new pcl::search::KdTree<pcl::PointXYZRGB>);
                     normalEstimationTree->setInputCloud (cloud_xyz);
                     n.setInputCloud (cloud_xyz);
                     n.setSearchMethod (normalEstimationTree);
                     n.setKSearch (20);
                     n.compute (*normals);
                     n.setViewPoint(0,0,0);

                     //* normals should not contain the point normals + surface curvatures

                     // Concatenate the XYZ and normal fields*
                     pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
                     pcl::concatenateFields (*cloud_xyz, *normals, *cloud_with_normals);
                     //* cloud_with_normals = cloud + normals


                     Eigen::Vector4f centroid (0.f, 0.f, 0.f, 1.f);
                     pcl::compute3DCentroid (*cloudCluster, centroid); centroid.w () = 1.f;

                     pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr centered_cloud_with_normals (new pcl::PointCloud<pcl::PointXYZRGBNormal>);

                     pcl::PointCloud<pcl::PointXYZRGBNormal>::iterator p;
                     for (p = cloud_with_normals->points.begin(); p < cloud_with_normals->points.end(); p++)
                     {
                         pcl::PointXYZRGBNormal *point = new pcl::PointXYZRGBNormal;

                         point->x = p->x - centroid.x();
                         point->y = p->y - centroid.y();
                         point->z = p->z - centroid.z();
                         point->r = 200;
                         point->g = 0;
                         point->b = 0;
                         point->a = p->a;
                         point->normal_x = p->normal_x;
                         point->normal_y = p->normal_y;
                         point->normal_z = p->normal_z;

                         centered_cloud_with_normals->points.push_back(*point);
                     }

                     // Create search tree*
                     pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr meshTree (new pcl::search::KdTree<pcl::PointXYZRGBNormal>);
                     meshTree->setInputCloud (centered_cloud_with_normals);

                     // Initialize objects
                     pcl::GreedyProjectionTriangulation<pcl::PointXYZRGBNormal> gp3;
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
                     gp3.setSearchMethod (meshTree);
                     gp3.reconstruct (triangles);

                    pcl::io::savePolygonFileSTL(req.output_filepath + model_name.str() + ".stl", triangles);

                    res.segmented_mesh_filenames.push_back(model_name.str());

                    geometry_msgs::Vector3 *offset = new geometry_msgs::Vector3;
                    offset->x = centroid.x();
                    offset->y = centroid.y();
                    offset->z = centroid.z();

                    res.offsets.push_back(*offset);


                    moveit_msgs::CollisionObject partial_mesh_model;
                    partial_mesh_model.header.frame_id = "camera_depth_optical_frame";
                    partial_mesh_model.id = model_name.str();
                    geometry_msgs::Pose pose;
                    pose.orientation.w = 1.0;
                    pose.position.x = centroid.x()/ 1000.0;
                    pose.position.y = centroid.y()/ 1000.0;
                    pose.position.z = centroid.z()/ 1000.0;

                    shape_msgs::Mesh mesh;
                    //shapes::Mesh* m = shapes::createMeshFromResource(req.output_filepath + model_name.str() + ".stl");
                    for (int triangle_index = 0; triangle_index < triangles.polygons.size(); triangle_index++)
                    {
                        pcl::Vertices p = triangles.polygons.at(triangle_index);

                        shape_msgs::MeshTriangle shape_msg_triangle;
                        shape_msg_triangle.vertex_indices[0] = p.vertices.at(0);
                        shape_msg_triangle.vertex_indices[1] = p.vertices.at(1);
                        shape_msg_triangle.vertex_indices[2] = p.vertices.at(2);

                        mesh.triangles.push_back(shape_msg_triangle);
                    }


                    for (p = centered_cloud_with_normals->points.begin(); p < centered_cloud_with_normals->points.end(); p++)
                    {
                        geometry_msgs::Point msg_p;
                        msg_p.x = p->x / 1000.0;
                        msg_p.y = p->y / 1000.0;
                        msg_p.z = p->z / 1000.0;

                        mesh.vertices.push_back(msg_p);
                    }
                    //mesh.triangles =  m->triangles;
                    //mesh.vertices = m->vertices;

                    partial_mesh_model.meshes.push_back(mesh);
                    partial_mesh_model.mesh_poses.push_back(pose);
                    partial_mesh_model.operation = partial_mesh_model.ADD;

                    //planning_scene.allowed_collision_matrix.entry_names.push_back( model_name.str());
                    res.planning_scene.allowed_collision_matrix.default_entry_names.push_back( model_name.str());
                    res.planning_scene.allowed_collision_matrix.default_entry_values.push_back(true);


                    res.planning_scene.world.collision_objects.push_back(partial_mesh_model);
                    mesh_index++;
                }

            }
        }


        res.planning_scene.is_diff = true;

        //save scene as one big mesh as well.
        pcl::io::savePolygonFileSTL(req.output_filepath + "single_mesh.stl", triangles);

        geometry_msgs::Vector3 *offset = new geometry_msgs::Vector3;
        offset->x = 0;
        offset->y = 0;
        offset->z = 0;

        res.offsets.push_back(*offset);
        res.segmented_mesh_filenames.push_back("single_mesh");


        ROS_INFO("saved_file");


        struct timeval endTime;
        gettimeofday(&endTime, NULL);
        long int endTimeMs = endTime.tv_sec * 1000 + endTime.tv_usec / 1000;
        long int totalTime = endTimeMs - startTimeMS;
        std::cout << "Total Time in milliseconds is " << totalTime <<  std::endl;
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
