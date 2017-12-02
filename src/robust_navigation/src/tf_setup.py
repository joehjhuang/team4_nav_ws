import numpy as np
import tf.transformations as tfm
import rospy
import tf
"""
2017 Base_Link_to_Map: supporting the pose estimation function. This function do initialized setup
Prereq:
    Measure locations of all the apriltags and set it up in the follow
    Measure the initial location of the robot base_link and set the following variables correctly:
    init_T_base_link_to_map; init_P_base_link_to_map
Note: P is [x,y,theta]
Joe Huang Oct 2017
"""
class Base_Link_to_Map:
    def __init__(self):
        """
        The class to calculate the transformation from base link to map
        First have to setup base_link to camera_link and apriltags to map
        Input:
            No Input
        """
        # camera_link to base_link transformation homogenous matrix
        self.T_camera_link_to_base_link = np.array(
            [[0., 0., 1., -0.27],
             [-1., 0., 0., -0.05],
             [0., -1., 0., 0.305],
             [0., 0., 0., 1.]]
        )
        self.T_base_link_to_camera_link = np.linalg.inv(self.T_camera_link_to_base_link)

        # initial transformation between tags and maps, key:id, vlue: transformation matrix
        T_tag1_to_map = np.array(
            [[1., 0., 0., 1.845],
             [0., 0., -1.,2.36],
             [0., 1., 0., 0.405],
             [0., 0., 0., 1.]]
        )
        T_tag4_to_map = np.array(
            [[0., 0., -1., 2.365],
             [-1., 0., 0.,1.852],
             [0., 1., 0., 0.41],
             [0., 0., 0., 1.]]
        )
        T_tag5_to_map = np.array(
            [[-1., 0., 0., 2.03],
             [0., 0., 1.,1.301],
             [0., 1., 0., 0.2],
             [0., 0., 0., 1.]]
        )
        T_tag2_to_map = np.array(
            [[0., 0., 1., 0.],
             [1., 0., 0,1.118],
             [0., 1., 0., 0.405],
             [0., 0., 0., 1.]]
        )
        T_tag7_to_map = np.array(
            [[-1., 0., 0., 1.245],
             [0., 0., 1.,0.],
             [0., 1., 0., 0.405],
             [0., 0., 0., 1.]]
        )
        T_tag8_to_map = np.array(
            [[0., 0., -1., 1.695],
             [-1., 0., 0.,0.65],
             [0., 1., 0., 0.2],
             [0., 0., 0., 1.]]
        )
        T_tag3_to_map = np.array(
            [[0., 0., 1., 0.42],
             [1., 0., 0,2.20],
             [0., 1., 0., 0.54],
             [0., 0., 0., 1.]]
        )
        T_tag6_to_map = np.array(
            [[1., 0., 0., 0.2],
             [0., 0., -1.,1.94],
             [0., 1., 0., 0.55],
             [0., 0., 0., 1.]]
        )        

        self.apriltag_transform_dict = {
            1: T_tag1_to_map,
            4: T_tag4_to_map,
            5: T_tag5_to_map,
            2: T_tag2_to_map,
            3: T_tag3_to_map,
            7: T_tag7_to_map,
            8: T_tag8_to_map,
	    6: T_tag6_to_map
        }

    def get_tf(self, pose_tag_to_camera_link, quat_tag_to_camera_link, april_tag_id):
        """
        Given the tf from april tag to camera link, calculate the tf from base link to map
        Input:
            pose_tag_to_camera_link: np(3,); the pose from tag to camera link
            quat_tag_to_camera_link: np(4,); the quaternion from tag to camera link
            april_tag_id: int; the id of the apriltag
        Output:
            P_base_link_to_map: np(3,); the P from base link to map
        """
        T_pose_tag_to_camera_link = np.array(
            [[1., 0., 0., pose_tag_to_camera_link[0]],
             [0., 1., 0., pose_tag_to_camera_link[1]],
             [0., 0., 1., pose_tag_to_camera_link[2]],
             [0., 0., 0., 1.]]
        )
        T_quat_tag_to_camera_link = tfm.quaternion_matrix(quat_tag_to_camera_link)
        T_tag_to_camera_link = np.dot(T_pose_tag_to_camera_link, T_quat_tag_to_camera_link)
        T_camera_link_to_tag = np.linalg.inv(T_tag_to_camera_link)

        T_tag_to_map = self.apriltag_transform_dict[april_tag_id]
        T_base_link_to_map = np.dot(np.dot(
            T_tag_to_map, T_camera_link_to_tag), self.T_base_link_to_camera_link)

        theta = tfm.euler_from_matrix(T_base_link_to_map,axes='sxyz')[2]
        P_base_link_to_map = np.array([T_base_link_to_map[0][3], T_base_link_to_map[1][3], theta])
        return P_base_link_to_map
        
        
# initial base_link to map transformation
init_T_base_link_to_map = np.array(
    [[0., 1., 0., 0.25],
     [-1., 0., 0., 0.5],
     [0., 0., 1., 1.57],
     [0., 0., 0., 1.]]
)
init_P_base_link_to_map = np.array([0.25, 0.5, 1.57]) #remember to change the angle!!!!!!

# weight of apriltags location 
apriltag_w = 0.99
threshold = 0.25

if __name__ == '__main__':
    blm = Base_Link_to_Map()
    T_tag_to_camera_link = np.array(
        [[1., 0., 0., -0.5],
         [0., -1., 0., 0.],
         [0., 0., -1., 0.5],
         [0., 0., 0., 1.]]
    )
    pose_tag_to_camera_link = T_tag_to_camera_link[0:3,3]
    quat_tag_to_camera_link = tfm.quaternion_from_matrix(T_tag_to_camera_link)
    P = blm.get_tf(pose_tag_to_camera_link, quat_tag_to_camera_link, 6)
    print P
    print "should print 0.45, 1.77, 1.57"




