from pyexpat import model
from statistics import mode
from gym_ignition.scenario import model_wrapper, model_with_file
from gym_ignition.utils.scenario import get_unique_model_name
from matplotlib.pyplot import get
from scenario import core as scenario
from scenario import gazebo as scenario_gazebo
from typing import List, Tuple
from os import path

class Panda(model_wrapper.ModelWrapper,
            model_with_file.ModelWithFile):
    
    """
    A Class that helps in setting up Panda Robot's model in ignition using gym_ignition
    Inserts the model into ignition gazebo, and sets the robot's initial joint positions
    Also adds joint state publisher and joint trajectory controller using the associated 
    class methods

    Parameters
    ----------
    world : scenario World Object
        Allows to access the ignition world using scenario
    name : str
        Name of the robot model
    position : List[float]
        Specifies robot model base link location
    orientation : List[float]
        Specifies robot model base link orientation
    use_fuel : bool
        If true downloads the robot model from fuel dataset
    arm_collision : bool
        If true the arm collision elements will be used for collision check
    hand_collision : bool
        If true the hand collision elements will be used for collision check
    separate_gripper_controller : bool
        If true uses a separate controller for the gripper
    initial_joint_positions : List[float]
        Initial joint positions for the robot

    Attributes
    ----------
    __separate_gripper_controller : bool
        If true uses a separate controller for the gripper
    __initial_joint_positions : List[float]
        Initial joint positions the robot starts with in the simulator

    """
    def __init__(self, 
                world: scenario.World,
                name: str = 'panda,',
                position: List[float] = (0,0,0),
                orientation: List[float] = (1,0,0,0),
                model_file: str = None,
                use_fuel: bool = True,
                arm_collision: bool = True,
                hand_collision: bool = True,
                separate_gripper_controller: bool = True,
                initial_joint_positions: List[float] = (0,
                                                        0,
                                                        0,
                                                        -1.57,
                                                        0,
                                                        1.57,
                                                        0.79,
                                                        0,
                                                        0)):

        # Get a unique model name
        model_name = get_unique_model_name(world, name)

        # Initial Pose
        initial_pose = scenario.Pose(position, orientation)

        # Get the default model description (URDF or SDF) 
        # allowing to pass a custom model
        if model_file is None:
            model_file = self.get_model_file(fuel=use_fuel)

        if not arm_collision or not hand_collision:
            model_file = self.disable_collision(model_file=model_file, 
                                                arm_collision=arm_collision,
                                                hand_collision=hand_collision)

        # Insert the model
        ok_model = world.to_gazebo().insert_model(model_file,
                                                    initial_pose,
                                                    model_name)

        if not ok_model:
            raise RuntimeError("Failed to insert " + model_name)
        
        # Get the model
        model = world.get_model(model_name)

        self.__separate_gripper_controller = separate_gripper_controller

        # Set initial joint configuration
        self.__set_initial_joint_positions(initial_joint_positions)
        if not model.to_gazebo().reset_joint_positions(self.get_initial_joint_positions(),
                                                       self.get_joint_names()):
            raise RuntimeError("Failed to set initial robot joint positions")

        # Add JointStatePublisher to Panda
        self.__add_joint_state_publisher(model)

        # Add JointTrajectoryController to Panda
        self.__add_joint_trajectory_controller(model)

        # Initialize base class
        super().__init__(model=model)

    @classmethod
    def get_model_file(self, fuel=True) -> str:
    """
    Retrives the file name of the robot model.

    Parameters
    ----------
    fuel : bool
        If True gets the sdf model file from the fuel ignitionrobotics website
        If False returns 'panda' by default.

    Returns
    -------
    string
        file name of the sdf model of the robot. 
        
    """
        if fuel:
            return scenario_gazebo.get_model_file_from_fuel(
                "https://fuel.ignitionrobotics.org/1.0/AndrejOrsula/models/panda")
        else:
            return "panda"

    @classmethod
    def get_joint_names(self) -> List[str]:
    """

    Returns
    -------
    List[string]
        List of joint names of the panda robot 
        
    """       return ["panda_joint1",
                "panda_joint2",
                "panda_joint3",
                "panda_joint4",
                "panda_joint5",
                "panda_joint6",
                "panda_joint7",
                "panda_finger_joint1",
                "panda_finger_joint2"]

    @classmethod
    def get_joint_limits(self) -> List[Tuple[float, float]]:
    """

    Returns
    -------
    List[Tuple[float, float]]
        List of joint limits for the panda robot
    """
        return [(-2.897246558310587, 2.897246558310587),
                (-1.762782544514273, 1.762782544514273),
                (-2.897246558310587, 2.897246558310587),
                (-3.07177948351002, -0.06981317007977318),
                (-2.897246558310587, 2.897246558310587),
                (-0.0174532925199433, 3.752457891787809),
                (-2.897246558310587, 2.897246558310587),
                (0.0, 0.04),
                (0.0, 0.04)]

    @classmethod
    def get_base_link_name(self) -> str:
    """

    Returns
    -------
    string
        Returns the name of the base link of panda robot
    """
        return "panda_link0"

    @classmethod
    def get_ee_link_name(self) -> str:
    """

    Returns
    -------
    string
        Returns the name of the end effector link of panda robot
    """

        return "panda_link8"

    @classmethod
    def get_gripper_link_names(self) -> List[str]:
    """

    Returns
    -------
    List[string]
        List of gripper link names
    """

        return ["panda_leftfinger",
                "panda_rightfinger"]

    @classmethod
    def get_finger_count(self) -> int:
    """

    Returns
    -------
    int
        The number of fingers in the gripper of the panda robot
    """

        return 2

    def get_initial_joint_positions(self) -> List[float]:
    """

    Returns
    -------
    List[float] 
        The initial joint positions the panda robot starts with in the simulation
    """
        return self.__initial_joint_positions

    def __set_initial_joint_positions(self, initial_joint_positions):
    """
    Sets the initial joint positions in the attribute __initial_joint_positions

    Parameters
    ----------
    initial_joint_positions : List[float]
        The initial joint positions the panda robot starts with in the simulation
    """
        self.__initial_joint_positions = initial_joint_positions

    def __add_joint_state_publisher(self, model) -> bool:
    """
    Adds a joint state publisher plugin to gazebo

    Parameters
    ----------
    model : object of Model Class from Scenario
       Contains model object of panda robot 
    """
        model.to_gazebo().insert_model_plugin(
            "libignition-gazebo-joint-state-publisher-system.so",
            "ignition::gazebo::systems::JointStatePublisher",
            self.__get_joint_state_publisher_config()
        )

    @classmethod
    def __get_joint_state_publisher_config(self) -> str:
    """
    
    Get the joint state publisher configurations

    Returns
    -------
    string
        List of Joint Names in SDF Format
    """
        return \
            """
            <sdf version="1.7">
            %s
            </sdf>
            """ \
            % " ".join(("<joint_name>" + joint + "</joint_name>" for joint in self.get_joint_names()))

    def __add_joint_trajectory_controller(self, model) -> bool:
    """
    
    Adds joint trajectory controller plugins into ignition gazebo

    Parameters
    ----------
    model : object of Model Class from Scenario
       Contains model object of panda robot 

    Returns
    -------
    bool
        True if plugin was added successfully
        False if plugin could not be added to ignition gazebo
    """
        if self.__separate_gripper_controller:
            model.to_gazebo().insert_model_plugin(
                "libignition-gazebo-joint-trajectory-controller-system.so",
                "ignition::gazebo::systems::JointTrajectoryController",
                self.__get_joint_trajectory_controller_config_joints_only()
            )
            model.to_gazebo().insert_model_plugin(
                "libignition-gazebo-joint-trajectory-controller-system.so",
                "ignition::gazebo::systems::JointTrajectoryController",
                self.__get_joint_trajectory_controller_config_gripper_only()
            )
        else:
            model.to_gazebo().insert_model_plugin(
                "libignition-gazebo-joint-trajectory-controller-system.so",
                "ignition::gazebo::systems::JointTrajectoryController",
                self.__get_joint_trajectory_controller_config()
            )

    def __get_joint_trajectory_controller_config(self) -> str:
        # TODO: refactor into something more sensible
        return \
            """
            <sdf version="1.7">
            <topic>joint_trajectory</topic>
            
            <joint_name>%s</joint_name>
            <initial_position>%s</initial_position>
            <position_p_gain>3000</position_p_gain>
            <position_d_gain>15</position_d_gain>
            <position_i_gain>1650</position_i_gain>
            <position_i_min>-15</position_i_min>
            <position_i_max>15</position_i_max>
            <position_cmd_min>-87</position_cmd_min>
            <position_cmd_max>87</position_cmd_max>
            <joint_name>%s</joint_name>
            <initial_position>%s</initial_position>
            <position_p_gain>9500</position_p_gain>
            <position_d_gain>47.5</position_d_gain>
            <position_i_gain>5225</position_i_gain>
            <position_i_min>-47.5</position_i_min>
            <position_i_max>47.5</position_i_max>
            <position_cmd_min>-87</position_cmd_min>
            <position_cmd_max>87</position_cmd_max>
            <joint_name>%s</joint_name>
            <initial_position>%s</initial_position>
            <position_p_gain>6500</position_p_gain>
            <position_d_gain>32.5</position_d_gain>
            <position_i_gain>3575</position_i_gain>
            <position_i_min>-32.5</position_i_min>
            <position_i_max>32.5</position_i_max>
            <position_cmd_min>-87</position_cmd_min>
            <position_cmd_max>87</position_cmd_max>
            <joint_name>%s</joint_name>
            <initial_position>%s1.57</initial_position>
            <position_p_gain>6000</position_p_gain>
            <position_d_gain>30</position_d_gain>
            <position_i_gain>3300</position_i_gain>
            <position_i_min>-30</position_i_min>
            <position_i_max>30</position_i_max>
            <position_cmd_min>-87</position_cmd_min>
            <position_cmd_max>87</position_cmd_max>
            <joint_name>%s</joint_name>
            <initial_position>%s</initial_position>
            <position_p_gain>2750</position_p_gain>
            <position_d_gain>2.75</position_d_gain>
            <position_i_gain>1515</position_i_gain>
            <position_i_min>-6.88</position_i_min>
            <position_i_max>6.88</position_i_max>
            <position_cmd_min>-12</position_cmd_min>
            <position_cmd_max>12</position_cmd_max>
            <joint_name>%s</joint_name>
            <initial_position>%s</initial_position>
            <position_p_gain>2500</position_p_gain>
            <position_d_gain>2.5</position_d_gain>
            <position_i_gain>1375</position_i_gain>
            <position_i_min>-6.25</position_i_min>
            <position_i_max>6.25</position_i_max>
            <position_cmd_min>-12</position_cmd_min>
            <position_cmd_max>12</position_cmd_max>
            <joint_name>%s</joint_name>
            <initial_position>%s</initial_position>
            <position_p_gain>2000</position_p_gain>
            <position_d_gain>2</position_d_gain>
            <position_i_gain>1100</position_i_gain>
            <position_i_min>-5</position_i_min>
            <position_i_max>5</position_i_max>
            <position_cmd_min>-12</position_cmd_min>
            <position_cmd_max>12</position_cmd_max>
            <joint_name>%s</joint_name>
            <initial_position>%s</initial_position>
            <position_p_gain>250</position_p_gain>
            <position_d_gain>0.2</position_d_gain>
            <position_i_gain>50</position_i_gain>
            <position_i_min>-10</position_i_min>
            <position_i_max>10</position_i_max>
            <position_cmd_min>-20</position_cmd_min>
            <position_cmd_max>20</position_cmd_max>
            <joint_name>%s</joint_name>
            <initial_position>%s</initial_position>
            <position_p_gain>250</position_p_gain>
            <position_d_gain>0.2</position_d_gain>
            <position_i_gain>50</position_i_gain>
            <position_i_min>-10</position_i_min>
            <position_i_max>10</position_i_max>
            <position_cmd_min>-20</position_cmd_min>
            <position_cmd_max>20</position_cmd_max>
            </sdf>
            """ % \
            (self.get_joint_names()[0],
             str(self.get_initial_joint_positions()[0]),
             self.get_joint_names()[1],
             str(self.get_initial_joint_positions()[1]),
             self.get_joint_names()[2],
             str(self.get_initial_joint_positions()[2]),
             self.get_joint_names()[3],
             str(self.get_initial_joint_positions()[3]),
             self.get_joint_names()[4],
             str(self.get_initial_joint_positions()[4]),
             self.get_joint_names()[5],
             str(self.get_initial_joint_positions()[5]),
             self.get_joint_names()[6],
             str(self.get_initial_joint_positions()[6]),
             self.get_joint_names()[7],
             str(self.get_initial_joint_positions()[7]),
             self.get_joint_names()[8],
             str(self.get_initial_joint_positions()[8]))

    def __get_joint_trajectory_controller_config_joints_only(self) -> str:
        # TODO: refactor into something more sensible
        return \
            """
            <sdf version="1.7">
            <topic>joint_trajectory</topic>
            
            <joint_name>%s</joint_name>
            <initial_position>%s</initial_position>
            <position_p_gain>3000</position_p_gain>
            <position_d_gain>15</position_d_gain>
            <position_i_gain>1650</position_i_gain>
            <position_i_min>-15</position_i_min>
            <position_i_max>15</position_i_max>
            <position_cmd_min>-87</position_cmd_min>
            <position_cmd_max>87</position_cmd_max>
            <joint_name>%s</joint_name>
            <initial_position>%s</initial_position>
            <position_p_gain>9500</position_p_gain>
            <position_d_gain>47.5</position_d_gain>
            <position_i_gain>5225</position_i_gain>
            <position_i_min>-47.5</position_i_min>
            <position_i_max>47.5</position_i_max>
            <position_cmd_min>-87</position_cmd_min>
            <position_cmd_max>87</position_cmd_max>
            <joint_name>%s</joint_name>
            <initial_position>%s</initial_position>
            <position_p_gain>6500</position_p_gain>
            <position_d_gain>32.5</position_d_gain>
            <position_i_gain>3575</position_i_gain>
            <position_i_min>-32.5</position_i_min>
            <position_i_max>32.5</position_i_max>
            <position_cmd_min>-87</position_cmd_min>
            <position_cmd_max>87</position_cmd_max>
            <joint_name>%s</joint_name>
            <initial_position>%s</initial_position>
            <position_p_gain>6000</position_p_gain>
            <position_d_gain>30</position_d_gain>
            <position_i_gain>3300</position_i_gain>
            <position_i_min>-30</position_i_min>
            <position_i_max>30</position_i_max>
            <position_cmd_min>-87</position_cmd_min>
            <position_cmd_max>87</position_cmd_max>
            <joint_name>%s</joint_name>
            <initial_position>%s</initial_position>
            <position_p_gain>2750</position_p_gain>
            <position_d_gain>2.75</position_d_gain>
            <position_i_gain>1515</position_i_gain>
            <position_i_min>-6.88</position_i_min>
            <position_i_max>6.88</position_i_max>
            <position_cmd_min>-12</position_cmd_min>
            <position_cmd_max>12</position_cmd_max>
            <joint_name>%s</joint_name>
            <initial_position>%s</initial_position>
            <position_p_gain>2500</position_p_gain>
            <position_d_gain>2.5</position_d_gain>
            <position_i_gain>1375</position_i_gain>
            <position_i_min>-6.25</position_i_min>
            <position_i_max>6.25</position_i_max>
            <position_cmd_min>-12</position_cmd_min>
            <position_cmd_max>12</position_cmd_max>
            <joint_name>%s</joint_name>
            <initial_position>%s</initial_position>
            <position_p_gain>2000</position_p_gain>
            <position_d_gain>2</position_d_gain>
            <position_i_gain>1100</position_i_gain>
            <position_i_min>-5</position_i_min>
            <position_i_max>5</position_i_max>
            <position_cmd_min>-12</position_cmd_min>
            <position_cmd_max>12</position_cmd_max>
            </sdf>
            """ % \
            (self.get_joint_names()[0],
             str(self.get_initial_joint_positions()[0]),
             self.get_joint_names()[1],
             str(self.get_initial_joint_positions()[1]),
             self.get_joint_names()[2],
             str(self.get_initial_joint_positions()[2]),
             self.get_joint_names()[3],
             str(self.get_initial_joint_positions()[3]),
             self.get_joint_names()[4],
             str(self.get_initial_joint_positions()[4]),
             self.get_joint_names()[5],
             str(self.get_initial_joint_positions()[5]),
             self.get_joint_names()[6],
             str(self.get_initial_joint_positions()[6]))

    def __get_joint_trajectory_controller_config_gripper_only(self) -> str:
        # TODO: refactor into something more sensible
        return \
            """
            <sdf version="1.7">
            <topic>gripper_trajectory</topic>
            <joint_name>%s</joint_name>
            <initial_position>%s</initial_position>
            <position_p_gain>250</position_p_gain>
            <position_d_gain>0.2</position_d_gain>
            <position_i_gain>50</position_i_gain>
            <position_i_min>-10</position_i_min>
            <position_i_max>10</position_i_max>
            <position_cmd_min>-20</position_cmd_min>
            <position_cmd_max>20</position_cmd_max>
            <joint_name>%s</joint_name>
            <initial_position>%s</initial_position>
            <position_p_gain>250</position_p_gain>
            <position_d_gain>0.2</position_d_gain>
            <position_i_gain>50</position_i_gain>
            <position_i_min>-10</position_i_min>
            <position_i_max>10</position_i_max>
            <position_cmd_min>-20</position_cmd_min>
            <position_cmd_max>20</position_cmd_max>
            </sdf>
            """ % \
            (self.get_joint_names()[7],
             str(self.get_initial_joint_positions()[7]),
             self.get_joint_names()[8],
             str(self.get_initial_joint_positions()[8]))

    @classmethod
    def disable_collision(self,
                          model_file: str,
                          arm_collision: bool,
                          hand_collision: bool) -> str:
    """

    Disables collision for a model by removing the collision geometry elements from the sdf files.

    Parameters
    ----------
    model_file : string
        Contains the path to the sdf model of the panda robot
    arm_collision : bool
        If true the arm collision elements will be used for collision check
    hand_collision : bool
        If true the hand collision elements will be used for collision check

    Returns
    -------

    """
        new_model_file = path.join(path.dirname(model_file),
                                   'model_without_arm_collision.sdf')

        # Remove collision geometry
        with open(model_file, "r") as original_sdf_file:
            with open(new_model_file, "w") as new_sdf_file:
                while True:
                    # Read a new line and make sure it is not the end of the file
                    line = original_sdf_file.readline()
                    if not line.rstrip():
                        break

                    # Once `<collision>` for lower links is encountered, skip that and all lines until `</collision>` is reached
                    if not arm_collision:
                        if '<collision name="panda_link' in line:
                            line = original_sdf_file.readline()
                            while not '</collision>' in line:
                                line = original_sdf_file.readline()
                            continue

                    # Same as for arm, but check for hand and both fingers
                    if not hand_collision:
                        if '<collision name="panda_hand_collision">' in line \
                            or '<collision name="panda_leftfinger_collision">' in line \
                                or '<collision name="panda_rightfinger_collision">' in line:
                            line = original_sdf_file.readline()
                            while not '</collision>' in line:
                                line = original_sdf_file.readline()
                            continue

                    # Write all other lines into the new file
                    new_sdf_file.write(line)

        # Return path to the new file
        return new_model_file
