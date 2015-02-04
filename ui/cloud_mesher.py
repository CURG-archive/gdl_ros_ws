from mesh_builder.srv import MeshCloudRequest, MeshCloud
from sensor_msgs.msg import PointCloud2
import rospkg
rospack = rospkg.RosPack()
import os
import time
import subprocess
import rospy


class CloudMesher():

    def __init__(self, pc_topic="/camera/depth_registered/points"):
        self.pc_topic = pc_topic
        self.pc = None
        self.publisher = None
        self.is_capturing = True

        #this is the string of the time in ms where all the generated files are to be placed
        self.time_dir = None

        #this is the fullpath upto and including the time_dir
        self.time_dir_full_filepath = None

        self.mesh_path = '/home/jvarley/grasp_deep_learning/gdl/src/graspit_bci/models/captured_meshes/'

    def point_cloud_callback(self, data):
        if self.is_capturing:
            self.pc = data

    def build_graspit_model_xml(self, model_name):

        model_xml = ""
        model_xml += '<?xml version="1.0" ?>\n'
        model_xml += '  <root>\n'
        model_xml += '     <geometryFile type="Inventor">' + model_name + ".iv" + '</geometryFile>\n'
        model_xml += '  </root>'

        f = open(self.time_dir_full_filepath + model_name + ".xml", 'w')
        f.write(model_xml)
        f.close()

    def build_graspit_model_iv(self, model_name):

        cmd = ""
        cmd +=  "rosrun ivcon ivcon "
        cmd +=  self.time_dir_full_filepath + model_name + ".stl "
        cmd +=  self.time_dir_full_filepath + model_name + ".iv"

        self.run_subprocess(cmd)

    def run_subprocess(self, cmd_string):
        print "running: " + cmd_string

        process = subprocess.Popen(cmd_string.split(), stdout=subprocess.PIPE)
        output = process.communicate()[0]

        print output

    def build_world_file(self, model_names, offsets):

        world_xml = ""
        world_xml += "  <?xml version=\"1.0\" ?>\n"
        world_xml += "  <world> \n"

        for i in range(len(model_names)):
            model_name = model_names[i]
            offset = offsets[i]
            if model_name is not "single_mesh":
                world_xml += " 	<graspableBody> \n"
                world_xml += " 		<filename>" "models/captured_meshes/" + self.time_dir + model_name + ".xml</filename>\n"
                world_xml += " 		<transform>\n"
                world_xml += " 			<fullTransform>(+1 +0 +0 +0)[+" + str(offset.x) + " +" + str(offset.y) + " +" + str(offset.z) + "]</fullTransform>\n"
                world_xml += " 		</transform>\n"
                world_xml += " 	</graspableBody>\n"

        world_xml += " 	<robot>\n"
        world_xml += " 		<filename>models/robots/NewBarrett/NewBarrett.xml</filename>\n"
        world_xml += " 		<dofValues>+0 +1.38064 +0 +0 +1.97002 +1 +1.36752 +0 +1.39189 +0 +0 </dofValues>\n"
        world_xml += " 		<transform>\n"
        world_xml += " 			<fullTransform>(-0.280913 -0.714488 +0.00500968 +0.640757)[+39.5943 +25.7277 +54.0391]</fullTransform>\n"
        world_xml += " 		</transform>\n"
        world_xml += " 	</robot>\n"
        world_xml += " 	<camera>\n"
        world_xml += " 		<position>-360.061 +993.574 -9.2951</position>\n"
        world_xml += " 		<orientation>+0.161447 -0.728796 -0.652878 +0.128612</orientation>\n"
        world_xml += " 		<focalDistance>+1154.76</focalDistance>\n"
        world_xml += " 	</camera>\n"
        world_xml += " </world>\n"

        f = open(self.time_dir_full_filepath + "world.xml", 'w')
        f.write(world_xml)
        f.close()

    def run_service(self):
        self.time_dir = str(int(time.time())) + '/'
        self.time_dir_full_filepath = self.mesh_path + self.time_dir
        if not os.path.exists(self.time_dir_full_filepath):
            os.mkdir(self.time_dir_full_filepath)

        req = MeshCloudRequest()
        req.input_cloud = self.pc
        req.output_filepath = self.time_dir_full_filepath

        response = self.service_proxy(req)
        model_names = response.segmented_mesh_filenames
        offsets = response.offsets

        for model_name in model_names:
            self.build_graspit_model_iv(model_name)
            self.build_graspit_model_xml(model_name)

        self.build_world_file(model_names, offsets)

        print response

    def listen(self, init_node=False):
        if init_node:
            rospy.init_node('listener', anonymous=True)

        rospy.Subscriber(self.pc_topic, PointCloud2, self.point_cloud_callback, queue_size=1)
        self.service_proxy = rospy.ServiceProxy("/meshCloud", MeshCloud)