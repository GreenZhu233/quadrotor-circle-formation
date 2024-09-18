import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Wrench
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
import threading
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import ReliabilityPolicy, QoSProfile
import numpy as np
import quaternion as qt
from functools import cmp_to_key

sim_time = 0.0  # 仿真时间
gravity = 9.8   # 重力加速度
frame = 1
callback_complete = []

# 目标圆
center = [0.0,0.0,4.0]
radius = 5.0

class Quadrotor:
    def __init__(self, id:int, namespace:str, mass:float, inertia:list[float]):
        self.id = id
        self.namespace = namespace
        self.mass = mass
        self.inertia = inertia  # 主轴坐标系下的惯量
        self.theta = 0.0

        self.integral_error_vx = 0
        self.integral_error_vy = 0
        self.previous_error_vx = 0
        self.previous_error_vy = 0
        self.previous_sim_time = 0

    # 订阅Odometry信息的回调函数
    def odom_callback(self, msg:Odometry):
        if callback_complete[self.id] == True:
            return
        # 获取当前位置
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.z = msg.pose.pose.position.z
        # 获取当前姿态
        self.ori = qt.from_float_array([msg.pose.pose.orientation.w,
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            ])
        # 获取当前速度
        self.vx = msg.twist.twist.linear.x
        self.vy = msg.twist.twist.linear.y
        self.vz = msg.twist.twist.linear.z
        # 获取当前角速度
        self.wx = msg.twist.twist.angular.x
        self.wy = msg.twist.twist.angular.y
        self.wz = msg.twist.twist.angular.z
        callback_complete[self.id] = True

    # 返回实现无人机期望速度的力
    def velocity_control(self, desired_vx, desired_vy, desired_height):
        vx = self.vx
        vy = self.vy
        vz = self.vz
        z = self.z
        # z方向
        kp_z = 1
        kd_z = 3
        force_z = self.mass * (-kd_z * vz + kp_z * (desired_height - z) + gravity)
        # xy方向
        dt = sim_time - self.previous_sim_time
        kp_xy = 3
        ki_xy = 0.1
        kd_xy = 0.5
        error_vx = desired_vx - vx
        error_vy = desired_vy - vy
        self.integral_error_vx += error_vx * dt
        self.integral_error_vy += error_vy * dt
        derivative_error_vx = (error_vx - self.previous_error_vx) / dt
        derivative_error_vy = (error_vy - self.previous_error_vy) / dt
        ax = kp_xy * error_vx + ki_xy * self.integral_error_vx / frame + kd_xy * derivative_error_vx
        ay = kp_xy * error_vy + ki_xy * self.integral_error_vy / frame + kd_xy * derivative_error_vy
        self.previous_error_vx = error_vx
        self.previous_error_vy = error_vy
        self.previous_sim_time = sim_time
        force_x = self.mass * ax
        force_y = self.mass * ay
        return np.array([force_x, force_y, force_z])

    # 返回实现无人机期望姿态的力矩
    def orientation_control(self, desired_quat = np.quaternion(1, 0, 0, 0)):
        ori = self.ori
        wx, wy, wz = self.wx, self.wy, self.wz
        kp = np.array(self.inertia) * 2
        kd = np.array(self.inertia) * 10
        error_quat = desired_quat * ori.inverse()
        torque = kp * np.array([error_quat.x, error_quat.y, error_quat.z])- kd * np.array([wx, wy, wz])
        return torque

    # 将前两个函数结合，实现根据期望速度自动调整无人机的姿态
    def vel_ori_control(self, desired_vx, desired_vy, desired_height):
        force = self.velocity_control(desired_vx, desired_vy, desired_height)
        axis = np.cross(np.array([0,0,1]), force)
        theta = np.arctan2(np.linalg.norm(axis), np.dot(np.array([0,0,1]), force))
        w = np.cos(theta / 2)
        axis = axis / np.linalg.norm(axis)
        x, y, z = axis * np.sin(theta / 2)
        torque = self.orientation_control(np.quaternion(w, x, y, z))
        return force, torque

# LIMIT-CYCLE-BASED DECOUPLED CONTROL
def circle_formation_control(radius, p_bar, p_hat, p_hat_minus, d, d_minus):
    Lambda, Gamma, C1, C2 = 1.2, 0.01, 0.4, 1.0
    p_bar = np.array(p_bar)
    p_hat = np.array(p_hat)
    p_hat_minus = np.array(p_hat_minus)
    p_bar_plus = p_bar + p_hat
    p_bar_minus = p_bar - p_hat_minus
    alpha = np.arctan2(np.cross(p_bar, p_bar_plus), np.dot(p_bar, p_bar_plus))
    if alpha < -0.1:
        alpha += 2*np.pi
    alpha_minus = np.arctan2(np.cross(p_bar_minus, p_bar), np.dot(p_bar_minus, p_bar))
    if alpha_minus < -0.1:
        alpha_minus += 2*np.pi
    l = radius**2 - np.linalg.norm(p_bar)**2
    u_p = Lambda * np.array([Gamma*l, -1, 1, Gamma*l]).reshape(2, 2) @ p_bar
    u_alpha = (d_minus * alpha - d * alpha_minus) / (d + d_minus)
    f = C1 + C2 * u_alpha / (2 * np.pi)
    u = u_p * f
    return u[0], u[1]

class SteeringNode(Node):
    def __init__(self, name):
        super().__init__(name)
        self.get_logger().info(f"Steering node {name} is started")
        self.declare_parameter("num_of_quadrotors", 5)
        self.declare_parameter("center_x", 0.0)
        self.declare_parameter("center_y", 0.0)
        self.declare_parameter("center_z", 4.0)
        num_of_quadrotors = self.get_parameter("num_of_quadrotors").get_parameter_value().integer_value
        global center
        center[0] = self.get_parameter("center_x").get_parameter_value().double_value
        center[1] = self.get_parameter("center_y").get_parameter_value().double_value
        center[2] = self.get_parameter("center_z").get_parameter_value().double_value
        self.quadlist = [Quadrotor(i, f"/quad_{i}", 1.316, [0.0128, 0.0128, 0.0218]) for i in range(num_of_quadrotors)]

        self.pub = []
        self.sub = []
        self.callback_group = ReentrantCallbackGroup()
        for quad in self.quadlist:
            # 创建Wrench发布者
            self.pub.append(self.create_publisher(Wrench, f"{quad.namespace}/gazebo_ros_force", 10))
            # 创建Odometry信息订阅者
            self.sub.append(self.create_subscription(Odometry,
                                                     f"{quad.namespace}/odom",
                                                     quad.odom_callback,
                                                     1,
                                                     callback_group=self.callback_group))
        # 创建 /clock 订阅者
        self.sub.append(
            self.create_subscription(Clock,
                                     "/clock",
                                     self.clock_callback,
                                     QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT),
                                    callback_group=self.callback_group))
        global callback_complete
        callback_complete = [False] * (num_of_quadrotors + 1)

    def clock_callback(self, msg:Clock):
        global sim_time
        sim_time = msg.clock.sec + msg.clock.nanosec * 1e-9
        callback_complete[-1] = True

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
        self.get_logger().info("排序后："+str([quad.id for quad in self.quadlist]))

    def send_wrench(self, quadrotor:Quadrotor, force = [0.0, 0.0, 0.0], torque=[0.0, 0.0, 0.0]):
        msg = Wrench()
        msg.force.x = force[0]
        msg.force.y = force[1]
        msg.force.z = force[2]
        msg.torque.x = torque[0]
        msg.torque.y = torque[1]
        msg.torque.z = torque[2]
        self.pub[quadrotor.id].publish(msg)
        # self.get_logger().info(f"Send force {force} and torque {torque} to quadrotor {quadrotor.id}")


def spin_all(executor: MultiThreadedExecutor, max_time):
    while rclpy.ok() and not all(callback_complete):
        executor.spin_once(max_time)
    callback_complete[:] = [False] * len(callback_complete)

def main(args=None):
    rclpy.init(args=args)
    node = SteeringNode("quad_steering_node")
    rclpy.spin_once(node)
    start_time = sim_time
    rate = node.create_rate(30)

    # 实时更新无人机的位置和速度信息
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    # 等待3秒
    while rclpy.ok():
        spin_all(executor, 1/30)
        if sim_time - start_time > 3.0:
            break
        rate.sleep()

    # 2秒内起飞到目标高度
    az = center[2] - node.quadlist[0].z     # z方向加速度
    while rclpy.ok():
        spin_all(executor, 1/30)
        if sim_time - start_time > 4.0:
            break
        for quad in node.quadlist:
            force = [0.0, 0.0, quad.mass * (gravity + az)]
            torque = quad.orientation_control()
            node.send_wrench(quad, force, torque)
        rate.sleep()
    while rclpy.ok():
        spin_all(executor, 1/30)
        if sim_time - start_time > 5.0:
            break
        for quad in node.quadlist:
            force = [0.0, 0.0, quad.mass * (gravity - az)]
            torque = quad.orientation_control()
            node.send_wrench(quad, force, torque)
        rate.sleep()

    # 等待1秒
    while rclpy.ok():
        spin_all(executor, 1/30)
        if sim_time - start_time > 6.0:
            break
        for quad in node.quadlist:
            force, torque = quad.vel_ori_control(0.0, 0.0, center[2])
            node.send_wrench(quad, force, torque)
        rate.sleep()

    # 无人机环形编队
    num = len(node.quadlist)    # 无人机数量
    p_bar = [[0, 0]] * num   # 各无人机相对圆环中心的位置x,y坐标
    p_hat = [[0, 0]] * num   # 相邻无人机的相对位置
    d = [2 * np.pi / num] * num  # 无人机的目标角距离
    node.counterclockwise_sort(center)
    previous_sim_time = sim_time
    while rclpy.ok():
        spin_all(executor, 1/30)
        if sim_time - previous_sim_time < 0.01:
            rate.sleep()
            continue
        else:
            previous_sim_time = sim_time
        for i, quad in enumerate(node.quadlist):
            p_bar[i] = quad.x - center[0], quad.y - center[1]
        for i in range(-1, num-1):
            p_hat[i] = p_bar[i+1][0] - p_bar[i][0], p_bar[i+1][1] - p_bar[i][1]
        for i, quad in enumerate(node.quadlist):
            desired_vx, desired_vy = circle_formation_control(radius, p_bar[i], p_hat[i], p_hat[i-1], d[i], d[i-1])
            force, torque = quad.vel_ori_control(desired_vx, desired_vy, center[2])
            node.send_wrench(quad, force, torque)
        global frame
        frame += 1
        rate.sleep()
    rclpy.shutdown()