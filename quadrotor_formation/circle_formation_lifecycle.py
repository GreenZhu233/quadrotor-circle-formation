import rclpy
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn, State
from rcl_interfaces.msg import SetParametersResult
from rclpy.parameter import Parameter
from nav_msgs.msg import Odometry
import geometry_msgs.msg as geometry_msgs
from geometry_msgs.msg import Wrench
from message_filters import Subscriber, ApproximateTimeSynchronizer

import numpy as np
import quaternion as qt
from functools import cmp_to_key

class PIDRegulator:
    def __init__(self, p, i, d, sat:float):
        self.p = p
        self.i = i
        self.d = d
        self.sat = sat

        self.integral = 0
        self.prev_err = 0
        self.prev_t = -1.0

    def regulate(self, err, t:float):
        derr_dt = 0.0
        dt = t - self.prev_t
        if self.prev_t > 0.0 and dt > 0.0:
            derr_dt = (err - self.prev_err)/dt
            self.integral += 0.5*(err + self.prev_err)*dt

        u = self.p*err + self.d*derr_dt + self.i*self.integral

        self.prev_err = err
        self.prev_t = t

        if (np.linalg.norm(u) > self.sat):
            u = self.sat*u/np.linalg.norm(u)
            self.integral = 0.0

        return u

class Quadrotor:
    def __init__(self, id:int, namespace:str, mass:float):
        self.id = id
        self.namespace = namespace
        self.mass = mass

        # State variables
        self.x = 0.0        # x position
        self.y = 0.0        # y position
        self.z = 0.0        # z position
        self.vx = 0.0       # x velocity
        self.vy = 0.0       # y velocity
        self.vz = 0.0       # z velocity
        self.wx = 0.0       # x angular velocity
        self.wy = 0.0       # y angular velocity
        self.wz = 0.0       # z angular velocity
        self.time = 0.0
        self.ori = qt.from_float_array([1.0, 0.0, 0.0, 0.0])
        self.theta = 0.0

        # PID regulators
        self.pid_horizontal = None
        self.pid_vertical = None

    def odom_callback(self, msg: Odometry):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.z = msg.pose.pose.position.z
        self.vx = msg.twist.twist.linear.x
        self.vy = msg.twist.twist.linear.y
        self.vz = msg.twist.twist.linear.z
        self.wx = msg.twist.twist.angular.x
        self.wy = msg.twist.twist.angular.y
        self.wz = msg.twist.twist.angular.z
        self.time = msg.header.stamp.sec + 1e-9*msg.header.stamp.nanosec
        self.ori = qt.from_float_array([msg.pose.pose.orientation.w,
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
        ])

    def create_pid_regulators(self, vel_p, vel_i, vel_d, vel_sat, height_p, height_i, height_d, height_sat):
        self.pid_horizontal = PIDRegulator(vel_p, vel_i, vel_d, vel_sat)
        self.pid_vertical = PIDRegulator(height_p, height_i, height_d, height_sat)

class QuadSteeringNode(LifecycleNode):
    def __init__(self, name):
        super().__init__(name)
        self.name = name
        self.sorted = False
        self.pid_config = dict()
        self.center_changed = False
        self.get_logger().info(f'{name} initialized')
        self.declare_parameter("num_of_quadrotors", 5)
        self.declare_parameter("center_x", 0.0)
        self.declare_parameter("center_y", 0.0)
        self.declare_parameter("center_z", 4.0)
        self.declare_parameter("radius", 5.0)

        self.declare_parameter("vel_p", 15.0)
        self.declare_parameter("vel_i", 0.1)
        self.declare_parameter("vel_d", 0.5)
        self.declare_parameter("vel_sat", 30.0)
        self.declare_parameter("height_p", 1.0)
        self.declare_parameter("height_i", 0.1)
        self.declare_parameter("height_d", 2.5)
        self.declare_parameter("height_sat", 20.0)
        self.declare_parameter("rot_p", [0.4, 0.4, 0.8])
        self.declare_parameter("rot_d", [0.1, 0.1, 0.2])
        self.declare_parameter("rot_sat", 0.5)

        self.declare_parameter("Lambda", 1.2)
        self.declare_parameter("Gamma", 0.01)
        self.declare_parameter("C1", 0.4)
        self.declare_parameter("C2", 1.0)

    def on_configure(self, state: State):
        self.get_logger().info(f'{self.name} configured')
        self.center = [0.0, 0.0, 0.0]
        self.num = self.get_parameter("num_of_quadrotors").get_parameter_value().integer_value
        self.center[0] = self.get_parameter("center_x").get_parameter_value().double_value
        self.center[1] = self.get_parameter("center_y").get_parameter_value().double_value
        self.center[2] = self.get_parameter("center_z").get_parameter_value().double_value
        self.goal_center = self.center.copy()
        self.radius = self.get_parameter("radius").get_parameter_value().double_value
        self.Lambda = self.get_parameter("Lambda").get_parameter_value().double_value
        self.Gamma = self.get_parameter("Gamma").get_parameter_value().double_value
        self.C1 = self.get_parameter("C1").get_parameter_value().double_value
        self.C2 = self.get_parameter("C2").get_parameter_value().double_value
        self.quadlist = [Quadrotor(i, f"/quad_{i}", 1.316) for i in range(self.num)]

        vel_p = self.get_parameter("vel_p").get_parameter_value().double_value
        vel_i = self.get_parameter("vel_i").get_parameter_value().double_value
        vel_d = self.get_parameter("vel_d").get_parameter_value().double_value
        vel_sat = self.get_parameter("vel_sat").get_parameter_value().double_value
        height_p = self.get_parameter("height_p").get_parameter_value().double_value
        height_i = self.get_parameter("height_i").get_parameter_value().double_value
        height_d = self.get_parameter("height_d").get_parameter_value().double_value
        height_sat = self.get_parameter("height_sat").get_parameter_value().double_value
        self.rot_p = np.array(self.get_parameter("rot_p").get_parameter_value().double_array_value)
        self.rot_d = np.array(self.get_parameter("rot_d").get_parameter_value().double_array_value)
        self.rot_sat = self.get_parameter("rot_sat").get_parameter_value().double_value
        self.pid_config = {'vel_p': vel_p, 'vel_i': vel_i, 'vel_d': vel_d, 'vel_sat': vel_sat,
                           'height_p': height_p, 'height_i': height_i, 'height_d': height_d, 'height_sat': height_sat}
        for quad in self.quadlist:
            quad.create_pid_regulators(vel_p, vel_i, vel_d, vel_sat, height_p, height_i, height_d, height_sat)

        self.pub = []
        self.sub: list[Subscriber] = []
        for i, quad in enumerate(self.quadlist):
            self.pub.append(self.create_publisher(Wrench, f"{quad.namespace}/gazebo_ros_force", 10))
            self.sub.append(Subscriber(self, Odometry, f"{quad.namespace}/odom"))

        self.synchronizer = ApproximateTimeSynchronizer(self.sub, 10, 0.1)
        self.add_on_set_parameters_callback(self.on_parameter_event)
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State):
        self.get_logger().info(f'{self.name} activated')
        self.synchronizer.registerCallback(self.circle_formation_control)
        return super().on_activate(state)

    def on_deactivate(self, state: State):
        self.get_logger().info(f'{self.name} deactivated')
        super().on_deactivate(state)
        self.synchronizer.callbacks.clear()
        self.sorted = False
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: State):
        self.get_logger().info(f'{self.name} cleaned up')
        for pub in self.pub:
            self.destroy_publisher(pub)
        for sub in self.sub:
            self.destroy_subscription(sub)
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State):
        self.get_logger().info(f'{self.name} shutting down')
        for pub in self.pub:
            self.destroy_publisher(pub)
        for sub in self.sub:
            self.destroy_subscription(sub)
        return TransitionCallbackReturn.SUCCESS

# --------------------------------------------------------------------------------------------

    def counterclockwise_sort(self, center):
        for quad in self.quadlist:
            quad.theta = np.arctan2(quad.y - center[1], quad.x - center[0])
        def cmp(quad1: Quadrotor, quad2: Quadrotor):
            delta_theta = quad1.theta - quad2.theta
            if delta_theta < -0.1:
                return -1
            elif delta_theta > 0.1:
                return 1
            else:
                r1 = np.linalg.norm([quad1.x - center[0], quad1.y - center[1]])
                r2 = np.linalg.norm([quad2.x - center[0], quad2.y - center[1]])
                if r1 < r2:
                    return -1
                else:
                    return 1
        self.quadlist.sort(key = cmp_to_key(cmp))

    def circle_formation_control(self, *args):
        for quad in self.quadlist:
            quad.odom_callback(args[quad.id])
        if(not self.sorted):
            self.counterclockwise_sort(self.center)
            self.sorted = True
        if(self.center_changed):
            flag = 0
            for i in range(3):
                if abs(self.goal_center[i] - self.center[i]) > 0.05:
                    self.center[i] += 0.05 * np.sign(self.goal_center[i] - self.center[i])
                else:
                    self.center[i] = self.goal_center[i]
                    flag += 1
            if flag == 3:
                self.center_changed = False
        p_bar = [np.empty(2, float) for _ in range(self.num)]       # current position vector of each quadrotor
        d = [2 * np.pi / self.num] * self.num       # desired angle between each quadrotor
        for i, quad in enumerate(self.quadlist):
            p_bar[i][0] = quad.x - self.center[0]
            p_bar[i][1] = quad.y - self.center[1]
            quad.theta = np.arctan2(p_bar[i][1], p_bar[i][0])
        for i, quad in enumerate(self.quadlist):
            i_plus = (i + 1) % self.num
            i_minus = i - 1
            alpha = self.quadlist[i_plus].theta - quad.theta
            if alpha < -0.1:
                alpha += 2 * np.pi
            alpha_minus = quad.theta - self.quadlist[i_minus].theta
            if alpha_minus < -0.1:
                alpha_minus += 2 * np.pi
            l = self.radius ** 2 - np.linalg.norm(p_bar[i]) ** 2
            u_p = self.Lambda * np.array([self.Gamma*l, -1, 1, self.Gamma*l]).reshape(2, 2) @ p_bar[i]
            u_alpha = (d[i_minus] * alpha - d[i] * alpha_minus) / (d[i] + d[i_minus])
            f = self.C1 + self.C2 * u_alpha / (2 * np.pi)
            vel = u_p * f

            # PID control
            err_v = vel - np.array([quad.vx, quad.vy])
            force_x, force_y = quad.pid_horizontal.regulate(err_v, quad.time)
            force_z = quad.pid_vertical.regulate(self.center[2] - quad.z, quad.time) + quad.mass * 9.8
            force = np.array([force_x, force_y, force_z])

            axis = np.cross(np.array([0,0,1]), force)
            theta = np.arccos(force_z / np.linalg.norm(force))
            w = np.cos(theta / 2)
            if np.linalg.norm(axis) > 1e-6:
                axis = axis / np.linalg.norm(axis)
            else:
                axis = np.array([0, 0, 1])
            x, y, z = axis * np.sin(theta / 2)
            error_quat = qt.from_float_array([w, x, y, z]) * quad.ori.inverse()
            torque = self.rot_p * np.array([error_quat.x, error_quat.y, error_quat.z])- self.rot_d * np.array([quad.wx, quad.wy, quad.wz])
            if np.linalg.norm(torque) > self.rot_sat:
                torque = self.rot_sat * torque / np.linalg.norm(torque)

            # send control command
            cmd = Wrench()
            cmd.force = geometry_msgs.Vector3(x=force_x, y=force_y, z=force_z)
            cmd.torque = geometry_msgs.Vector3(x=torque[0], y=torque[1], z=torque[2])
            self.pub[quad.id].publish(cmd)

    def on_parameter_event(self, data: list[Parameter]):
        pid_changed = False
        for parameter in data:
            match parameter.name:
                case "Gamma":
                    self.Gamma = parameter.value
                case "Lambda":
                    self.Lambda = parameter.value
                case "C1":
                    self.C1 = parameter.value
                case "C2":
                    self.C2 = parameter.value
                case "radius":
                    self.radius = parameter.value
                case "center_x":
                    self.goal_center[0] = parameter.value
                    self.center_changed = True
                case "center_y":
                    self.goal_center[1] = parameter.value
                    self.center_changed = True
                case "center_z":
                    self.goal_center[2] = parameter.value
                    self.center_changed = True
                case "rot_p":
                    self.rot_p = np.array(parameter.value)
                case "rot_d":
                    self.rot_d = np.array(parameter.value)
                case "rot_sat":
                    self.rot_sat = parameter.value
                case s if s in self.pid_config.keys():
                    self.pid_config[s] = parameter.value
                    pid_changed = True
                case _:
                    self.get_logger().warn(f"Unknown parameter: {parameter.name}")
                    return SetParametersResult(successful=False)
        if pid_changed:
            for quad in self.quadlist:
                quad.create_pid_regulators(self.pid_config['vel_p'], self.pid_config['vel_i'], self.pid_config['vel_d'], self.pid_config['vel_sat'],
                                           self.pid_config['height_p'], self.pid_config['height_i'], self.pid_config['height_d'], self.pid_config['height_sat'])
        self.get_logger().info("Parameters dynamically changed...")
        return SetParametersResult(successful=True)

def main(args=None):
    try:
        rclpy.init(args=args)
        node = QuadSteeringNode('quad_steering_node')
        rclpy.spin(node)
    except Exception as e:
        print('Caught exception: ' + str(e))
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()