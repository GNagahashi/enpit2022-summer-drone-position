#!/usr/bin/env python


import rospy
from test_srv.srv import CheckPos, CheckPosResponse
from nav_msgs.msg import Odometry
from gazebo_msgs.srv import DeleteModel, DeleteModelResponse
"""
Define of msg/srv

CheckPos:
    string request
    ---
    bool response
Odometry:
    geometry_msgs/PoseWithConvariance pose
        geometry_msgs/Pose pose
            geometry_msgs/Point position
                float64 x
                float64 y
                float64 z
        float64[36] convariance
    geometry_msgs/TwistWithConvariance twist
        ...
DeleteModel:
    string model_name
    ---
    bool success
    string status_message
"""


def main():
    server_funcs = SrvFuncs()
    server_funcs.position = dict(
        box1 = (1.5024, 3.92435),
        box2 = (7.9082, 2.42908),
        cylinder1 = (7.62584, 5.74044),
        sphere1 = (2.8558, 1.83817),
    )

    # Create node
    rospy.init_node('check_position_server')

    # Create service server
    rospy.loginfo('Creating service server, please wait a minute...')
    rospy.Service('check_position', CheckPos, server_funcs.server_callback)
    # Subscribe to /mavros/global_position/local (for get drone position)
    rospy.loginfo('Subscribing to "/mavros/global_position/local", please wait a minute...')
    rospy.Subscriber('/mavros/global_position/local', Odometry, server_funcs.update_drone_position)
    # Create client to /gazebo/delete_model (for delete object from gazebo)
    rospy.loginfo('Creating client to "/gazebo/delete_model", please wait a minute...')
    rospy.wait_for_service('/gazebo/delete_model')
    server_funcs.handler_for_gazebo_delete = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)

    rospy.loginfo('Ready to server')
    rospy.spin()



class SrvFuncs(object):
    def __init__(self):
        self.__position = dict()
        self.__drone_x = float(0)
        self.__drone_y = float(0)
        self.__handler_for_gazebo_delete = None

    @property
    def position(self):
        return self.__position

    @position.setter
    def position(self, new_position):
        if type(new_position) == dict:
            self.__position = new_position

    @property
    def drone_x(self):
        return self.__drone_x

    @drone_x.setter
    def drone_x(self, x):
        self.__drone_x = x

    @property
    def drone_y(self):
        return self.__drone_y

    @drone_y.setter
    def drone_y(self, y):
        self.__drone_y = y

    @property
    def handler_for_gazebo_delete(self):
        return self.__handler_for_gazebo_delete

    @handler_for_gazebo_delete.setter
    def handler_for_gazebo_delete(self, handler):
        self.__handler_for_gazebo_delete = handler

    def server_callback(self, req):
        rospy.loginfo('Server has received request: {}'.format(req.request))
        res_client = CheckPosResponse()
        res_server = DeleteModelResponse()
        from time import sleep; sleep(3)
        for key in self.position.keys():
            obj_xy = self.position.get(key)  # obj_xy = self.__position.get(key)
            if (obj_xy[0] - 1) <= self.drone_x <= (obj_xy[0] + 1) and (obj_xy[1] - 1) <= self.drone_y <= (obj_xy[1] + 1):
                res_client.response = True
                res_server = self.handler_for_gazebo_delete(key)
                break
            else:
                res_client.response = False
        rospy.loginfo('Server send response: {}'.format(res_client.response))
        if res_server.success:
            rospy.loginfo('Delete object: {}'.format(key))
        return res_client

    def update_drone_position(self, msg):
        self.drone_x = msg.pose.pose.position.x
        self.drone_y = msg.pose.pose.position.y


if __name__ == '__main__':
    main()