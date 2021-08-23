#!/usr/bin/env python3

import rospy
import json
import yaml
import time
import os.path

from duckietown_msgs.msg import WheelsCmdStamped#, Twist2DStamped
from std_srvs.srv import EmptyResponse#, Empty

from duckietown.dtros import DTROS, NodeType, TopicType, DTParam#, ParamType


class ControllerInterfaceNode(DTROS):
    """
    """

    def __init__(self, node_name, AI):

        # Initialize the DTROS parent class
        super(ControllerInterfaceNode, self).__init__(node_name=node_name, node_type=NodeType.CONTROL)
        
        # Get the vehicle name
        self.veh_name = rospy.get_namespace().strip("/")

        #TODO
        # Read parameters from a robot-specific yaml file if such exists
        # self.read_params_from_calibration_file()

        #TODO
        # Get static parameters
        # self._k = rospy.get_param("~k")
        # Get editable parameters
        # self._gain = DTParam("~gain", param_type=ParamType.FLOAT, min_value=0.1, max_value=1.0)
        # self._trim = DTParam("~trim", param_type=ParamType.FLOAT, min_value=0.1, max_value=1.0)
        # self._limit = DTParam("~limit", param_type=ParamType.FLOAT, min_value=0.1, max_value=1.0)
        # self._baseline = DTParam("~baseline", param_type=ParamType.FLOAT, min_value=0.05, max_value=0.2)
        # self._radius = DTParam("~radius", param_type=ParamType.FLOAT, min_value=0.01, max_value=0.1)
        # self._v_max = DTParam("~v_max", param_type=ParamType.FLOAT, min_value=0.01, max_value=2.0)
        # self._omega_max = DTParam("~omega_max", param_type=ParamType.FLOAT, min_value=1.0, max_value=10.0)

        # subscrib to the image topic
        self.image = DTParam("~image/compressed")

        #TODO
        # Prepare the save calibration service
        # self.srv_save = rospy.Service("~save_calibration", Empty, self.srv_save_calibration)

        # Setup publishers
        self.pub_wheels_cmd = rospy.Publisher(
            "~wheels_cmd", WheelsCmdStamped, queue_size=1, dt_topic_type=TopicType.CONTROL
        )
        #self.pub_velocity = rospy.Publisher(
        #    "~velocity", Twist2DStamped, queue_size=1, dt_topic_type=TopicType.CONTROL
        #)
        
        self.AI = AI
        
        self.log("Initialized with: \n%s" % self.get_configuration_as_str())
        
    def car_cmd_call(self):
        # Create message to add command to:
        msg_wheels_cmd = WheelsCmdStamped()
        msg_wheels_cmd.header.stamp = rospy.get_rostime()
        
        # Calculate the movement:
        [u_l,u_r] = self.AI.run(self.image.value)
        
        # limiting output to limit, which is 1.0 for the duckiebot
        u_r_limited = self.trim(u_r, -self._limit.value, self._limit.value)
        u_l_limited = self.trim(u_l, -self._limit.value, self._limit.value)
        
        
        msg_wheels_cmd.vel_right = u_r_limited
        msg_wheels_cmd.vel_left = u_l_limited
        self.pub_wheels_cmd.publish(msg_wheels_cmd)

    def get_current_configuration(self):
        return {"confituration not implemented": True}
        #return {
        #    "gain": rospy.get_param("~gain"),
        #    "trim": rospy.get_param("~trim"),
        #    "baseline": rospy.get_param("~baseline"),
        #    "radius": rospy.get_param("~radius"),
        #    "k": rospy.get_param("~k"),
        #    "limit": rospy.get_param("~limit"),
        #    "v_max": rospy.get_param("~v_max"),
        #    "omega_max": rospy.get_param("~omega_max"),
        #}

    def get_configuration_as_str(self) -> str:
        return json.dumps(self.get_current_configuration(), sort_keys=True, indent=4)

    def read_params_from_calibration_file(self):
        """
        Reads the saved parameters from `/data/config/calibrations/kinematics/DUCKIEBOTNAME.yaml`
        or uses the default values if the file doesn't exist. Adjsuts the ROS paramaters for the
        node with the new values.
        """
        # Check file existence
        fname = self.get_calibration_filepath(self.veh_name)
        # Use the default values from the config folder if a robot-specific file does not exist.
        if not os.path.isfile(fname):
            self.logwarn("Kinematics calibration %s not found! Using default instead." % fname)
        else:
            with open(fname, "r") as in_file:
                try:
                    yaml_dict = yaml.load(in_file, Loader=yaml.FullLoader)
                except yaml.YAMLError as exc:
                    self.logfatal("YAML syntax error. File: %s fname. Exc: %s" % (fname, exc))
                    rospy.signal_shutdown()
                    return

            # Set parameters using value in yaml file
            if yaml_dict is None:
                # Empty yaml file
                return
            for param_name in self.get_current_configuration().keys():
                param_value = yaml_dict.get(param_name)
                if param_name is not None and param_value is not None:
                    rospy.set_param("~" + param_name, param_value)
                else:
                    # Skip if not defined, use default value instead.
                    pass

    def srv_save_calibration(self, req=None):
        """
        Saves the current kinematics paramaters to a robot-specific file at
        `/data/config/calibrations/kinematics/DUCKIEBOTNAME.yaml`.
        Args:
            req: Not used.
        Returns:
            EmptyResponse
        """

        # Write to a yaml file
        data = {"calibration_time": time.strftime("%Y-%m-%d-%H-%M-%S"), **self.get_current_configuration()}

        # Write to file
        file_name = self.get_calibration_filepath(self.veh_name)
        with open(file_name, "w") as outfile:
            outfile.write(yaml.dump(data, default_flow_style=False))

        # ---
        self.log(
            "Saved kinematic calibration to %s with values: \n%s"
            % (file_name, self.get_configuration_as_str())
        )

        return EmptyResponse()

    @staticmethod
    def trim(value, low, high):
        """
        Trims a value to be between some bounds.
        Args:
            value: the value to be trimmed
            low: the minimum bound
            high: the maximum bound
        Returns:
            the trimmed value
        """
        return max(min(value, high), low)

    @staticmethod
    def get_calibration_filepath(name):
        """
        Returns the path to the robot-specific configuration file,
        i.e. `/data/config/calibrations/kinematics/DUCKIEBOTNAME.yaml`.
        Args:
            name (:obj:`str`): the Duckiebot name
        Returns:
            :obj:`str`: the full path to the robot-specific calibration file
        """
        cali_file_folder = "/data/config/calibrations/kinematics/"
        cali_file = cali_file_folder + name + ".yaml"
        return cali_file

    def on_shutdown(self):
        pass # shutdown the AI runner if it is running in this node

if __name__ == "__main__":
    # Initialize the node
    print("Node Running (or at least started)")
    ControllerInterface = ControllerInterfaceNode("controller_interface_code", lambda x :(0,0))
    # Keep it spinning to keep the node alive
    rospy.spin()