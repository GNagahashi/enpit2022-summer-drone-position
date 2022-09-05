#!/usr/bin/env python
# >>> import sys; sys.version
# '2.7.17 (default, Jul  1 2022, 15:56:32) \n[GCC 7.5.0]'


import rospy
from drone_position.srv import CheckPos, CheckPosResponse
from nav_msgs.msg import Odometry
from gazebo_msgs.srv import DeleteModel, DeleteModelResponse
"""
Define of msg/srv

- CheckPos:
    string request
    ---
    bool response
- Odometry:
    geometry_msgs/PoseWithConvariance pose
        geometry_msgs/Pose pose
            geometry_msgs/Point position
                float64 x
                float64 y
                float64 z
        float64[36] convariance
    geometry_msgs/TwistWithConvariance twist
        ...
- DeleteModel:
    string model_name
    ---
    bool success
    string status_message
"""


def main():
    # Name used by ROS (service, topic)
    check_position = 'check_position'
    mavros_grobal_position_local = '/mavros/global_position/local'
    gazebo_delete_model = '/gazebo/delete_model'

    # Create instance and register object(prize)
    # gazebo (x, y, z) -> program (-y, x, z)
    # gazebo x -> program y(program y = gazebo x), gazebo y = program x and reverse sign(program x = gazebo -y)
    server_funcs = SrvFuncs()
    # test_world.world
    # server_funcs.position = dict(
    #     box1 = (1.5024, 3.92435),
    #     box2 = (7.9082, 2.42908),
    #     cylinder1 = (7.62584, 5.74044),
    #     sphere1 = (2.8558, 1.83817),
    # )
    # main_world.world
    server_funcs.position = dict(
        car = (-1.66340595333, 4.0369),
        chipster = (-0.49655, 3.46187),
        figure1 = (-1.10845, 2.23614),
        figure2 = (0.53832, 3.73998),
        snackbag = (0.889952, 2.21655),
    )

    # Create node
    rospy.init_node('check_position_server', anonymous = True)

    # Create service server
    rospy.loginfo('Creating service server({}), please wait a minute...'.format(check_position))
    rospy.Service(check_position, CheckPos, server_funcs.server_callback)
    # Subscribe to /mavros/global_position/local (for get drone position)
    rospy.loginfo('Create subscriber for {}, please wait a minute...'.format(mavros_grobal_position_local))
    rospy.Subscriber(mavros_grobal_position_local, Odometry, server_funcs.update_drone_position)
    # Create client to /gazebo/delete_model (for delete object from gazebo)
    rospy.loginfo('Create client for {}, please wait a minute...'.format(gazebo_delete_model))
    rospy.wait_for_service(gazebo_delete_model)
    server_funcs.handler_for_gazebo_delete = rospy.ServiceProxy(gazebo_delete_model, DeleteModel)

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
        from time import sleep; sleep(3)  # wait 3 sec
        for key in self.position.keys():
            obj_xy = self.position.get(key)
            if (obj_xy[0] - 0.5) <= self.drone_x <= (obj_xy[0] + 0.5) and (obj_xy[1] - 0.5) <= self.drone_y <= (obj_xy[1] + 0.5):
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