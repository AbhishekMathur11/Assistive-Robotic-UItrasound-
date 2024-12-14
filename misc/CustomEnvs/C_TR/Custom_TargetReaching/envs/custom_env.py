import gym
from gym import error, spaces, utils
from gym.utils import seeding
import pybullet as p
import numpy as np
import os, time

class WorldCreation:
    def __init__(self, pid, robot_type='rrbot', time_step=0.02, target_size=0.2, np_random=None):
        self.id = pid
        self.robot_type = robot_type
        self.time_step = time_step
        self.target_size = target_size
        self.np_random = np_random
        self.directory = os.path.join(os.path.dirname(os.path.realpath(f"/content/envs")), 'assets')
        self.utils = Utils(self.id, self.np_random)

    def create_new_world(self):
        p.resetSimulation(physicsClientId=self.id)

        # Configure camera position
        # p.resetDebugVisualizerCamera(cameraDistance=1.75, cameraYaw=-25, cameraPitch=-45, cameraTargetPosition=[-0.2, 0, 0.4], physicsClientId=self.id)

        p.configureDebugVisualizer(p.COV_ENABLE_MOUSE_PICKING, 0, physicsClientId=self.id)
        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0, physicsClientId=self.id)

        # Load all models off screen and then move them into place
        p.loadURDF(os.path.join(self.directory, 'plane', 'plane.urdf'), physicsClientId=self.id)

        # Disable rendering during creation
        p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0, physicsClientId=self.id)

        p.setTimeStep(self.time_step, physicsClientId=self.id)
        # Disable real time simulation so that the simulation only advances when we call stepSimulation
        p.setRealTimeSimulation(0, physicsClientId=self.id)

        # Load the base for robot
        if self.robot_type == 'ur5':
            self.load_box(position=[0,0,0.25], dimensions=(0.1,0.1,0.5), mass=0, color=(0.5,0.5,0.5,1))

        robot, all_joints, controllable_joints, tool_index, joint_lower_limits, joint_upper_limits  = self.load_robot()
        target, target_pos, target_orient = self.load_target(robot, tool_index)

        return robot, all_joints, controllable_joints, tool_index, joint_lower_limits, joint_upper_limits, target, target_pos, target_orient

    def load_box(self, position, orientation=(0, 0, 0, 1), mass=1., dimensions=(1., 1., 1.), color=None):
        """
        Load a box in the world (only available in the simulator).
        Args:
            position (float[3]): position of the box in the Cartesian world space (in meters)
            orientation (float[4]): orientation of the box using quaternion [x,y,z,w].
            mass (float): mass of the box (in kg). If mass = 0, the box won't move even if there is a collision.
            dimensions (float[3]): dimensions of the box (in meter)
            color (int[4], None): color of the box for red, green, blue, and alpha, each in range [0,1]
        Returns:
            int, Body: unique id of the box in the world, or the box body
        """
        dimensions = np.asarray(dimensions)
        collision_shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=dimensions / 2., physicsClientId=self.id)
        visual_shape = p.createVisualShape(p.GEOM_BOX, halfExtents=dimensions / 2., rgbaColor=color, physicsClientId=self.id)

        box = p.createMultiBody(baseMass=mass, baseCollisionShapeIndex=collision_shape, baseVisualShapeIndex=visual_shape,
                                basePosition=position, baseOrientation=orientation, physicsClientId=self.id)
        return box

    def load_robot(self):

        if self.robot_type == 'ur5':
            # Load the UR5 robot
            robot = p.loadURDF(os.path.join(self.directory, 'ur', 'ur5-afc.urdf'), useFixedBase=True, basePosition=[0, 0, 0.5], flags=p.URDF_USE_SELF_COLLISION, physicsClientId=self.id)
            control_joint_indices = [0, 1, 2]
            all_joint_indices = [0, 1, 2, 3, 4, 5]
            tool_index = 6
            joint_lower_limits, joint_upper_limits = self.get_joint_limits(robot, control_joint_indices)
            # joint_lower_limits = np.deg2rad([-360, -160, -160, -90, -90, -90])
            # joint_upper_limits = np.deg2rad([360, 20, 20, -90, -90, -90])            
            joint_lower_limits = np.deg2rad([-360, -360, -360, -90, -90, -90])
            joint_upper_limits = np.deg2rad([360, 360, 360, -90, -90, -90])  
            initial_joint_positions = [0.0, -1.57, 1.57, -1.57, -1.57, -1.57]
            # initial_joint_positions = np.random.uniform(low=joint_lower_limits, high=joint_upper_limits)

        if self.robot_type == 'rrbot':
            # Load the UR5 robot
            robot = p.loadURDF(os.path.join(self.directory, 'rrbot', 'rrbot.urdf'), useFixedBase=True, basePosition=[0, 0, 0], flags=p.URDF_USE_SELF_COLLISION, physicsClientId=self.id)
            control_joint_indices = [1, 2]
            all_joint_indices = [1, 2]
            tool_index = 3
            # joint_lower_limits, joint_upper_limits = self.get_joint_limits(robot, control_joint_indices)
            joint_lower_limits = np.deg2rad([-340, -340])
            joint_upper_limits = np.deg2rad([340, 340])
            # initial_joint_positions = [0.0, 0.0]
            initial_joint_positions = np.random.uniform(low=joint_lower_limits, high=joint_upper_limits)

        # Let the environment settle
        for _ in range(100):
            p.stepSimulation(physicsClientId=self.id)

        # Enforce the joint limits
        for  i, j_idx in enumerate(control_joint_indices):
            p.changeDynamics(robot, linkIndex=j_idx, jointLowerLimit=joint_lower_limits[i], jointUpperLimit=joint_upper_limits[i], physicsClientId=self.id)

        # Reset the joint states (change to random initial position)
        self.reset_joint_states(robot, joint_ids=all_joint_indices,  positions=initial_joint_positions)

        return robot, all_joint_indices, control_joint_indices, tool_index, joint_lower_limits, joint_upper_limits

    def get_joint_limits(self, body_id, joint_ids):
        lower_limits = []
        upper_limits = []
        
        for j in joint_ids:
            joint_info = p.getJointInfo(body_id, j, physicsClientId=self.id)
            joint_name = joint_info[1]
            lower_limit = joint_info[8]
            upper_limit = joint_info[9]
            if lower_limit == 0 and upper_limit == -1:
                lower_limit = -1e10
                upper_limit = 1e10
            lower_limits.append(lower_limit)
            upper_limits.append(upper_limit)

        lower_limits = np.array(lower_limits)
        upper_limits = np.array(upper_limits)
        
        return lower_limits, upper_limits

    def reset_joint_states(self, body_id, joint_ids, positions):
        """
        Reset the joint states. It is best only to do this at the start, while not running the simulation:
        `reset_joint_state` overrides all physics simulation.
        Args:
            body_id (int): unique body id.
            joint_ids (int, list[int]): joint indices where each joint index is between [0..num_joints(body_id)]
            positions (float, list[float], np.array[float]): the joint position(s) (angle in radians [rad] or
              position [m])
        """
        # reset the joint states
        for i, joint_id in enumerate(joint_ids):
            position = positions[i]
            p.resetJointState(body_id, jointIndex=joint_id, targetValue=position, physicsClientId=self.id)

    def load_target(self, robot_id, tool_id):

        if self.robot_type == 'ur5':
            sphere_radius = self.target_size/2.0
            # self.target_position = np.array([0.5, 0.1, 0.75])
            # target_high, target_low = np.array([1.0, 1.0, 1.0]), np.array([-1.0, -1.0, 0.5]) # full
            # target_high, target_low = np.array([1.0, 1.0, 1.0]), np.array([0.0, 0.0, 0.5]) # q1
            target_high, target_low = np.array([0.5, 0.5, 0.5]), np.array([0.2, -0.5, 0.5]) # q1
            target_orientation = np.array([1, 0, 0, 0])
            target_orientation = np.asarray(p.getEulerFromQuaternion(target_orientation))
            # orient_low, orient_high = np.array([0., 0., 0.]), np.array([0., 0., 0.])
            # target_orientation = np.asarray([0, 0, 1])
            robot_reach = 0.75 # 0.875 (ideal reach)
        elif self.robot_type == 'rrbot':
            sphere_radius = self.target_size/2.0
            target_high, target_low = np.array([2.0, 0.2, 4.0]), np.array([-2.0, 0.2, 0.0]) 
            target_orientation = np.array([1, 0, 0, 0])
            robot_reach = 1.5 # 2.0 (ideal reach)

        while True:
            target_position = self.np_random.uniform(low=target_low, high=target_high)
            if np.linalg.norm(target_position-self.robot_base_position()) < robot_reach:
                break
        # target_position = np.array([0.47717288, 0.1092235,  0.93179151])
        # target_orientation = self.np_random.uniform(low=orient_low, high=orient_high)

        visual_shape = p.createVisualShape(p.GEOM_SPHERE, radius=sphere_radius, rgbaColor=(1, 0, 0, 0.5), physicsClientId=self.id)
        target = p.createMultiBody(baseVisualShapeIndex=visual_shape, baseMass=0., basePosition=list(target_position), physicsClientId=self.id)
        
        return target, target_position, target_orientation

    def robot_base_position(self):
        if self.robot_type == 'rrbot':
            return np.array([0, 0, 2])
        elif self.robot_type == 'ur5':
            return np.array([0, 0, 0.5])
class Utils:
    def __init__(self, pid, np_random):
        self.id = pid
        self.np_random = np_random

    def get_joint_positions(self, body_id, joint_ids):
        """
        Get the position of the given joint(s).
        Args:
            body_id (int): unique body id.
            joint_ids (int, list[int]): joint id, or list of joint ids.
        Returns:
            if 1 joint:
                float: joint position [rad]
            if multiple joints:
                np.array[float[N]]: joint positions [rad]
        """
        if isinstance(joint_ids, int):
            return p.getJointState(body_id, joint_ids, physicsClientId=self.id)[0]
        return np.asarray([state[0] for state in p.getJointStates(body_id, joint_ids, physicsClientId=self.id)])

    def get_jacobian(self, robot_id, tool_id, q):
        r"""
        Return the full geometric Jacobian matrix :math:`J(q) = [J_{lin}(q)^T, J_{ang}(q)^T]^T`, such that:
        .. math:: v = [\dot{p}^T, \omega^T]^T = J(q) \dot{q}
        where :math:`\dot{p}` is the Cartesian linear velocity of the link, and :math:`\omega` is its angular velocity.
        Args:
            body_id (int): unique body id.
            link_id (int): link id.
            local_position (np.array[float[3]]): the point on the specified link to compute the Jacobian (in link
              local coordinates around its center of mass). If None, it will use the CoM position (in the link frame).
            q (np.array[float[N]]): joint positions of size N, where N is the number of DoFs.
            dq (np.array[float[N]]): joint velocities of size N, where N is the number of DoFs.
            des_ddq (np.array[float[N]]): desired joint accelerations of size N.
        Returns:
            np.array[float[6,N]], np.array[float[6,6+N]]: full geometric (linear and angular) Jacobian matrix. The
                number of columns depends if the base is fixed or floating.
        """
        # Note that q, dq, ddq have to be lists in PyBullet (it doesn't work with numpy arrays)
        if isinstance(q, np.ndarray):
            q = q.ravel().tolist()

        local_position = p.getLinkState(robot_id, tool_id, physicsClientId=self.id)[2]
        dq = [0]*len(q)
        # calculate full jacobian
        lin_jac, ang_jac = p.calculateJacobian(robot_id, tool_id, localPosition=local_position,
                                                      objPositions=list(q), objVelocities=dq, objAccelerations=dq, physicsClientId=self.id)
        return np.vstack((lin_jac, ang_jac))

    # def get_my_analytical_jacobian(self, robot_id, tool_id, q):
        
    #     if isinstance(q, np.ndarray):
    #         q = q.ravel().tolist()

    #     tool_orientation = np.asarray(p.getLinkState(robot_id, tool_id)[1])
    #     o = np.asarray(p.getEulerFromQuaternion(tool_orientation))
    #     (r,pi,y) = o
    #     B = np.array([[1, 0, np.sin(pi)],
    #                   [0, np.cos(r), -np.cos(pi)*np.sin(r)],
    #                   [0, np.sin(r), np.cos(pi)*np.cos(r)]])
    #     Bi = np.linalg.inv(B)
    #     # analytical jacobian
    #     temp = np.eye(6)
    #     temp[3:6,3:6] = Bi
    #     J = self.get_jacobian(robot_id, tool_id, q)
    #     Ja = np.matmul(temp, J)
    #     return Ja

    def get_jacobian_derivative_rpy_to_angular_velocity(self, rpy_angle):
        r"""
        Return the Jacobian that maps RPY angle rates to angular velocities, i.e. :math:`\omega = T(\phi) \dot{\phi}`.
        Warnings: :math:`T` is singular when the pitch angle :math:`\theta_p = \pm \frac{\pi}{2}`
        Args:
            rpy_angle (np.array[float[3]]): RPY Euler angles [rad]
        Returns:
            np.array[float[3,3]]: Jacobian matrix that maps RPY angle rates to angular velocities.
        """
        roll, pitch, yaw = rpy_angle
        T = np.array([[1., 0., np.sin(pitch)],
                      [0., np.cos(roll), -np.cos(pitch) * np.sin(roll)],
                      [0., np.sin(roll), np.cos(pitch) * np.cos(roll)]])
        return T

    def get_analytical_jacobian(self, jacobian, rpy_angle):
        r"""
        Return the analytical Jacobian :math:`J_{a}(q) = [J_{lin}(q), J_{\phi}(q)]^T`, which respects:
        .. math:: \dot{x} = [\dot{p}, \dot{\phi}]^T = J_{a}(q) \dot{q}
        where :math:`\dot{p}` is the Cartesian linear velocity of the link, and :math:`\phi` are the Euler angles
        representing the orientation of the link. In general, the derivative of the Euler angles is not equal to
        the angular velocity, i.e. :math:`\dot{\phi} \neq \omega`.
        The analytical and geometric Jacobian are related by the following expression:
        .. math::
            J_{a}(q) = \left[\begin{array}{cc}
                I_{3 \times 3} & 0_{3 \times 3} \\
                0_{3 \times 3} & T^{-1}(\phi)
                \end{array} \right] J(q)
        where :math:`T` is the matrix that respects: :math:`\omega = T(\phi) \dot{\phi}`.
        Warnings:
            - We assume that the Euler angles used are roll, pitch, yaw (RPY)
            - We currently compute the analytical Jacobian from the geometric Jacobian. If we assume that we use RPY
                Euler angles then T is singular when the pitch angle :math:`\theta_p = \pm \frac{\pi}{2}.
        Args:
            jacobian (np.array[float[6,N]], np.array[float[6,6+N]]): full geometric Jacobian.
            rpy_angle (np.array[float[3]]): RPY Euler angles
        Returns:
            np.array[float[6,N]], np.array[float[6,6+N]]: the full analytical Jacobian. The number of columns
                depends if the base is fixed or floating.
        """
        T = self.get_jacobian_derivative_rpy_to_angular_velocity(rpy_angle)
        Tinv = np.linalg.inv(T)
        Ja = np.vstack((np.hstack((np.identity(3), np.zeros((3, 3)))),
                        np.hstack((np.zeros((3, 3)), Tinv)))).dot(jacobian)
        return Ja

    def enable_force_torque_sensor(self, body_id, joint_ids):
        
        if isinstance(joint_ids, int):
            p.enableJointForceTorqueSensor(body_id, joint_ids, 1)
        else:
            for joint_id in joint_ids:
                p.enableJointForceTorqueSensor(body_id, joint_id, 1)

    def force_torque_sensing(self, body_id, joint_ids):
        """
        Return the joint reaction forces at the given joint. Note that the torque sensor must be enabled, otherwise
        it will always return [0,0,0,0,0,0].
        Args:
            body_id (int): unique body id.
            joint_ids (int, int[N]): joint id, or list of joint ids
        Returns:
            if 1 joint:
                np.array[float[6]]: joint reaction force (fx,fy,fz,mx,my,mz) [N,Nm]
            if multiple joints:
                np.array[float[N,6]]: joint reaction forces [N, Nm]
        """
        if isinstance(joint_ids, int):
            return np.asarray(p.getJointState(body_id, joint_ids, physicsClientId=self.id)[2])
        return np.asarray([state[2] for state in p.getJointStates(body_id, joint_ids, physicsClientId=self.id)])

    def quaternion_error(self, quat_des, quat_cur):
        r"""
        Compute the orientation (vector) error between the current and desired quaternion; that is, it is the difference
        between :math:`q_curr` and :math:`q_des`, which is given by: :math:`\Delta q = q_{curr}^{-1} q_{des}`.
        Only the vector part is returned which can be used in PD control.
        Args:
            quat_des (np.array[float[4]]): desired quaternion [x,y,z,w]
            quat_cur (np.array[float[4]]): current quaternion [x,y,z,w]
        Returns:
            np.array[float[3]]: vector error between the current and desired quaternion
        """
        diff = quat_cur[-1] * quat_des[:3] - quat_des[-1] * quat_cur[:3] - self.skew_matrix(quat_des[:3]).dot(quat_cur[:3])
        return diff

    def skew_matrix(self, vector):
        r"""
        Return the skew-symmetric matrix of the given vector, which allows to represents the cross product between the
        given vector and another vector, as the multiplication of the returned skew-symmetric matrix with the other
        vector.
        The skew-symmetric matrix from a 3D vector :math:`v=[x,y,z]` is given by:
        .. math::
            S(v) = \left[ \begin{array}{ccc} 0 & -z & y \\ z & 0 & -x \\ -y & x & 0 \\ \end{array} \right]
        It can be shown [2] that: :math:`\dot{R}(t) = \omega(t) \times R(t) = S(\omega(t)) R(t)`, where :math:`R(t)` is
        a rotation matrix that varies as time :math:`t` goes, :math:`\omega(t)` is the angular velocity vector of frame
        :math:`R(t) with respect to the reference frame at time :math:`t`, and :math:`S(.)` is the skew operation that
        returns the skew-symmetric matrix from the given vector.
        Args:
            vector (np.array[float[3]]): 3D vector
        Returns:
            np.array[float[3,3]]: skew-symmetric matrix
        References:
            - [1] Wikipedia: https://en.wikipedia.org/wiki/Skew-symmetric_matrix#Cross_product
            - [2] "Robotics: Modelling, Planning and Control" (sec 3.1.1), by Siciliano et al., 2010
        """
        x, y, z = np.array(vector).flatten()
        return np.array([[0., -z, y],
                        [z, 0., -x],
                        [-y, x, 0.]])

    def get_damped_least_squares_inverse(self, jacobian, damping_factor=0.01):
        r"""
        Return the damped least-squares (DLS) inverse, given by:
        .. math:: \hat{J} = J^T (JJ^T + k^2 I)^{-1}
        which can then be used to get joint velocities :math:`\dot{q}` from the cartesian velocities :math:`v`, using
        :math:`\dot{q} = \hat{J} v`.
        Args:
            jacobian (np.array[float[D,N]]): Jacobian matrix
            damping_factor (float): damping factor
        Returns:
            np.array[float[N,D]]: DLS inverse matrix
        """
        J, k = jacobian, damping_factor
        return J.T.dot(np.linalg.inv(J.dot(J.T) + k**2 * np.identity(J.shape[0])))

    def map_action_values(self, output_lower_limit, output_upper_limit, action_value):
        input_upper_limit, input_lower_limit = self.action_space.high[0], self.action_space.low[0]
        output = output_lower_limit + ((output_upper_limit - output_lower_limit) / (input_upper_limit - input_lower_limit)) * (action_value - input_lower_limit)
        return output

    def euclidian_distance(self, a, b):
        return np.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2 + (a[2] - b[2])**2)    

    def tool_position(self, body_id, tool_id):
        return np.asarray(p.getLinkState(body_id, tool_id, physicsClientId=self.id)[0])        

    def tool_orientation(self, body_id, tool_id):
        return np.asarray(p.getLinkState(body_id, tool_id, physicsClientId=self.id)[1])

class CustomTargetReachingEnv(gym.Env):
    metadata = {'render.modes':['human']}
    def __init__(self, robot_type = 'rrbot', action_robot_len=2, obs_robot_len=16, reward_type=2, orientation_ctrl=False, action_type='joint_space', max_speed=np.pi, target_size=0.2, time_step = 0.02):

        """
        Initialize the environment.
        Args:
            robot_type (str, rrbot) : type of robot 
            action_robot_len (int, None): length of action space
            obs_robot_len (int, None): length of observation space
            reward_type (int, 2): type of rewards for target reaching
            action_type (str, joint_space): action space
            max_speed ()
        """

        # Start the bullet physics server
        self.gui = False
        self.id = p.connect(p.DIRECT)
        
        self.robot_type = robot_type
        self.max_speed = max_speed #rad/sec
        # self.max_speed = 1.0 #rad/sec
        self.time_step = time_step
        self.reward_ver = reward_type
        self.action_type = action_type
        
        self.action_robot_len = action_robot_len
        self.obs_robot_len = obs_robot_len
        self.orientation_ctrl = orientation_ctrl
        if self.action_type == 'joint_space':
            action_high = np.array([self.max_speed]*self.action_robot_len)
            action_low = -action_high
            self.action_space = spaces.Box(low=np.float32(action_low), high=np.float32(action_high), dtype=np.float32)
        elif self.action_type == 'task_space':
            if self.robot_type == 'rrbot':
                if self.orientation_ctrl:
                    action_high = np.array([2.0, 0.0, 2.0, 0, np.pi, 0])
                else:
                    action_high = np.array([2.0, 0.0, 2.0, 0, 0, 0])                
                action_low = -action_high
            elif self.robot_type == 'ur5':
                if self.orientation_ctrl:
                    action_high = np.array([0.5, 0.5, 0.5, np.pi, np.pi, np.pi])
                else:
                    action_high = np.array([0.5, 0.5, 0.5, 0, 0, 0])
                action_low = -action_high                
            self.action_space = spaces.Box(low=np.float32(action_low), high=np.float32(action_high), dtype=np.float32)
        self.observation_space = spaces.Box(low=np.float32(np.array([-1.0]*self.obs_robot_len)), high=np.float32(np.array([1.0]*self.obs_robot_len)), dtype=np.float32)

        self.target_position = np.array([0.0, 0.0, 0.0]) #[x,y,z]
        self.obs_state = None
        self.target_size = target_size

        #attributes
        self.directory = os.path.join(os.path.dirname(os.path.realpath(f"/content/envs")), 'assets')
        self.setup_timing()
        self.seed(1001) #initialize a seed

        self.world_creation = WorldCreation(self.id, robot_type=self.robot_type, time_step=self.time_step, target_size=self.target_size, np_random=self.np_random)
        self.utils = Utils(self.id, self.np_random)

        self.width = 1920
        self.height = 1080
        # self.width = 3840
        # self.height = 2160
        # self.width = 400
        # self.height = 300
    
    def seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]

    def step(self, action):
        # print(action)
        action = np.clip(action, a_min=self.action_space.low, a_max=self.action_space.high)
        # action = np.array([0, 0])
        action *= 0.15
        # print('clipped-action:', action) 
        
        # update state
        # q = self.get_joint_positions(self.robot, self.controllable_joints)
        # q = q + action * self.dt

        # q = np.clip(q, a_min=self.joint_lower_limits, a_max=self.joint_upper_limits)

        # p.setJointMotorControlArray(self.robot, self.controllable_joints, 
        #                             controlMode=p.POSITION_CONTROL, targetPositions=q)

        self.iteration +=1
        if self.last_sim_time is None:
            self.last_sim_time = time.time()

        if self.action_type == 'joint_space':
            p.setJointMotorControlArray(self.robot, self.controllable_joints,
                                        controlMode=p.VELOCITY_CONTROL,
                                        targetVelocities=action,
                                        physicsClientId=self.id)
        elif self.action_type == 'task_space':
            # get current joint states
            q = self.utils.get_joint_positions(self.robot, self.all_joints)
            # print('joint_positions:', np.rad2deg(q))
            # get jacobian
            J = self.utils.get_jacobian(self.robot, self.tool, q)

            if self.orientation_ctrl:
                tool_orient = self.utils.tool_orientation(self.robot, self.tool)
                rpy_tool = p.getEulerFromQuaternion(tool_orient, physicsClientId=self.id)
                # print(self.utils.tool_position(self.robot, self.tool))
                pos_error = (self.target_position - self.utils.tool_position(self.robot, self.tool))
                # print('target_orient:', self.target_orientation)
                # print('rpy_tool:', rpy_tool)
                # # d
                orient_err = (self.target_orientation - np.asarray(rpy_tool))
                # print('orient_err:', orient_err)
                # # orient_err = np.array([0,0,0])
                action = np.hstack((pos_error, orient_err))
                # print('my_action:',action)
                J = self.utils.get_analytical_jacobian(jacobian=J, rpy_angle=rpy_tool)
                # J = self.utils.get_my_analytical_jacobian(self.robot, self.tool, q)

            # print('Jacobian:', J)
            # Pseudo-inverse: \hat{J} = J^T (JJ^T + k^2 I)^{-1}
            Jp = self.utils.get_damped_least_squares_inverse(J, damping_factor=0.01)
            # print('Jacobian_inv:', Jp)

            # get joint velocity
            joint_velocities = np.matmul(Jp, action)
            # print('joint_velocities:', joint_velocities)

            # small tweak to control less joints in UR5
            # joint_velocities = joint_velocities[self.controllable_joints] if self.robot_type == 'ur5' else joint_velocities

            p.setJointMotorControlArray(self.robot, self.all_joints,
                                        controlMode=p.VELOCITY_CONTROL,
                                        targetVelocities=joint_velocities,
                                        physicsClientId=self.id)

            # Joint position control
            # q = q + joint_velocities*self.time_step

            # p.setJointMotorControlArray(self.robot, self.all_joints,
            #                             controlMode=p.POSITION_CONTROL,
            #                             targetPositions=q,
            #                             physicsClientId=self.id)

            action = joint_velocities # variable assignment just to include it in reward similar to joint space learning

        # Update robot position
        p.stepSimulation(physicsClientId=self.id)
        if self.gui:
            # Slow down time so that the simulation matches real time
            self.slow_time()

        #get an observation of state
        obs_state = self._get_obs()

        # get reward        
        distance_to_target = self.utils.euclidian_distance(self.target_position, self.utils.tool_position(self.robot, self.tool))
        # print('distance-to-target %.2f:' % (distance_to_target))
        orientation_error = self.utils.euclidian_distance(self.target_orientation, self.utils.tool_orientation(self.robot, self.tool))
        
        reward_dist = -distance_to_target
        reward_orient = -orientation_error
        reward_ctrl = -np.square(action).sum()
        
        reached = False
        terminated = False
        if distance_to_target < self.target_size:
            # print('Reached Goal !!')
            reached = True

        # Reward: 0
        if self.reward_ver == 0:
            reward = reward_dist
        # Reward: 1
        if self.reward_ver == 1:
            reward = reward_dist + reward_ctrl
        # # Reward: 2 or 3
        if self.reward_ver == 2:
            if not reached:
                reward = - 1.0 + reward_ctrl
                # reward = - 1.0
            else:
                reward = 100
        if self.reward_ver == 3:
            if not reached:
                reward = reward_dist + reward_ctrl
                # reward = - 1.0
            else:
                reward = 100 
        if self.reward_ver == 4:
            if not reached:
                reward = reward_dist + reward_ctrl + reward_orient
            else:
                reward = 100 
        # if (self.iteration*self.dt) > self.max_time:
        #     print('Time Limit Reached !!')
        #     terminated = True

        # done = reached or terminated
        done = False
        
        info = {'Position Error': distance_to_target, 'Orientation_error:': orientation_error, 'task_success': int(reached)}
        # print(info)

        return obs_state, reward, done, info

    def _get_obs(self):
        # get current end-effector position and velocity in the task/operational space
        x = np.asarray(p.getLinkState(self.robot, self.tool)[0])
        dx = np.asarray(p.getLinkState(self.robot, self.tool, computeLinkVelocity=1)[6])
        o = np.asarray(p.getLinkState(self.robot, self.tool)[1])
        do = np.asarray(p.getLinkState(self.robot, self.tool, computeLinkVelocity=1)[7])

        # get end-effector velocity in task space
        tool_pose = np.concatenate((x, o))
        tool_velocity = np.concatenate((dx, do))

        return np.concatenate((self.target_position, tool_pose, tool_velocity)).ravel()

    def reset(self):
        # print('Reset Env!!')

        self.setup_timing()
        self.robot, self.all_joints, self.controllable_joints, self.tool, self.joint_lower_limits, self.joint_upper_limits, self.target, self.target_position, self.target_orientation = self.world_creation.create_new_world()

        # self.create_world()
        # self.robot, self.controllable_joints, self.tool, self.joint_lower_limits, self.joint_upper_limits  = self.load_robot()
        # self.target, self.targte_position, self.target_orientation = self.load_target()
        
        # Enable rendering
        p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1, physicsClientId=self.id)
        # p.setGravity(0, 0, -1, physicsClientId=self.id)

        return self._get_obs()

    def setup_timing(self):
        self.total_time = 0
        self.last_sim_time = None
        self.iteration = 0    

    def slow_time(self):
        # Slow down time so that the simulation matches real time
        t = time.time() - self.last_sim_time
        if t < self.time_step:
            time.sleep(self.time_step - t)
        self.last_sim_time = time.time()

    def render(self, mode='human'):
        if not self.gui:
            self.gui = True
            p.disconnect(self.id)
            # self.id = p.connect(p.GUI, options='--background_color_red=0.8 --background_color_green=0.9 --background_color_blue=1.0 --width=%d --height=%d' % (self.width, self.height))
            self.id = p.connect(p.GUI, options='--background_color_red=0.8 --background_color_green=0.9 --background_color_blue=1.0')
            # self.id = p.connect(p.GUI)            
            self.world_creation = WorldCreation(self.id, robot_type=self.robot_type, time_step=self.time_step, target_size=self.target_size, np_random=self.np_random)
            # self.util = Util(self.id, self.np_random)
            # # print('Physics server ID:', self.id)

