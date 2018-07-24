'''
This module define and load all modules for Baxter visualization
'''
import rospy
import rospkg

from geometry_msgs.msg import (
    Pose,
    Point,
    Quaternion
)

from gazebo_msgs.srv import (
    SpawnModel,
    DeleteModel,
)

def render_camera(camera1_pose = Pose(position=Point(x=2.03,y=-0.304,z=2.04),
                   orientation=Quaternion(w=0.012, x=-0.238,y=0.0122,z=0.97)),
                  camera2_pose = Pose(position=Point(x=0.48,y=-2.36,z=1.77),
                   orientation=Quaternion(w=0.69, x=-0.147, y=0.168, z=0.69))):
    '''
    This function loads two cameras in the environment to capture images and
    store them in the folder defined in the .xml file.
    '''
    # Get camera 1
    camera_path = '/home/arclab/model_editor_models/camera_new/model.sdf'
    camera1_xml = ''
    with open(camera_path,"r") as camera_file:
        camera1_xml = camera_file.read().replace('\n', '')

    # Spawn camera 1
    rospy.wait_for_service('/gazebo/spawn_sdf_model')
    try:
        spawn_urdf = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        resp_urdf = spawn_urdf("camera_new", camera1_xml, "/",
                               camera1_pose, "world")
    except rospy.ServiceException, e:
        rospy.logerr("Spawn SDF service call failed: {0}".format(e))


def delete_camera():
    '''
    This deletes the camera from the environment.
    '''
    try:
        delete_model = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
        resp_delete = delete_model("camera_new")
        #resp_delete = delete_model("camera_new2")
    except rospy.ServiceException, e:
        rospy.loginfo("Delete Model service call failed: {0}".format(e))

def load_gazebo_models(table_pose=Pose(position=Point(x=0.75, y=-.504, z=0)),
                       table_reference_frame="world",
                       block_pose=Pose(position=Point(
                           x=0.6725, y=-0.25, z=0.91)),
                       block_reference_frame="world"):
    '''
    This function loads the table and block to the Gazebo environment
    '''
    # Get Models' Path
    model_path = rospkg.RosPack().get_path('baxter_sim_examples')+"/models/"
    # Load Table SDF
    table_xml = ''
    with open(model_path + "cafe_table/model.sdf", "r") as table_file:
        table_xml = table_file.read().replace('\n', '')
    # Load Block URDF
    block_xml = ''
    with open(model_path + "block/model.urdf", "r") as block_file:
        block_xml = block_file.read().replace('\n', '')
    #Load Maze Table SDF
    maze_table_xml = ''
    with open("/home/arclab/model_editor_models/L_table_0_0/model.sdf", "r") as maze_table_file:
        maze_table_xml = maze_table_file.read().replace('\n', '')

    # Spawn Table SDF
    rospy.wait_for_service('/gazebo/spawn_sdf_model')
    try:
        spawn_sdf = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        resp_sdf = spawn_sdf("L_table_0_0", maze_table_xml, "/",
                             table_pose, table_reference_frame)
    except rospy.ServiceException, e:
        rospy.logerr("Spawn SDF service call failed: {0}".format(e))
    # Spawn Block URDF
    rospy.wait_for_service('/gazebo/spawn_urdf_model')
    try:
        spawn_urdf = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
        resp_urdf = spawn_urdf("block", block_xml, "/",
                               block_pose, block_reference_frame)
    except rospy.ServiceException, e:
        rospy.logerr("Spawn URDF service call failed: {0}".format(e))


def delete_gazebo_models():
    # This will be called on ROS Exit, deleting Gazebo models
    # Do not wait for the Gazebo Delete Model service, since
    # Gazebo should already be running. If the service is not
    # available since Gazebo has been killed, it is fine to error out
    try:
        delete_model = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
        resp_delete = delete_model("L_table_0_0")
        resp_delete = delete_model("block")
    except rospy.ServiceException, e:
        rospy.loginfo("Delete Model service call failed: {0}".format(e))
