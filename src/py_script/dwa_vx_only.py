import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
import threading
import time
import numpy as np
import math

from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition, VehicleAttitude
from sensor_msgs.msg import LaserScan

from scipy.spatial import cKDTree

def enu_to_ned(pos_enu):
    return [pos_enu[1], pos_enu[0], -pos_enu[2]]

def quaternion_to_dcm(q):
    '''
    This DCM transforms vectors from the body frame (FRD: Forward-Right-Down) to the world frame (NED).
    '''
    w, x, y, z = q
    dcm = [
        [1 - 2*y**2 - 2*z**2, 2*x*y - 2*z*w, 2*x*z + 2*y*w],
        [2*x*y + 2*z*w, 1 - 2*x**2 - 2*z**2, 2*y*z - 2*x*w],
        [2*x*z - 2*y*w, 2*y*z + 2*x*w, 1 - 2*x**2 - 2*y**2]
    ]
    return dcm

class DroneAvoidance(Node):
    def __init__(self):
        super().__init__('drone_avoidance')

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.scan_sub = self.create_subscription(LaserScan, '/lidar_scan', self.scan_cb, qos)
        self.localpos_sub = self.create_subscription(
            VehicleLocalPosition, 
            '/fmu/out/vehicle_local_position_v1',
            self.vehicle_local_position_callback, 
            qos
        )
        self.att_sub = self.create_subscription(
            VehicleAttitude, 
            '/fmu/out/vehicle_attitude',
            self.vehicle_attitude_callback, 
            qos
        )

        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos
        )
        self.trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos
        )

        # self.goto_setpoint_publisher = self.create_publisher(GotoSetpoint, '/fmu/in/goto_setpoint', qos)
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos
        )

        self.scan = None
        self.local_position = None
        self.attitude = None
        #self.current_setpoint = None
        self.current_setpoint = {'position': [0.0, 0.0, -3.0], 'yaw': float('nan')}
        self.mode = 'position'
        self.dwa_active = False
        self.offboard_setpoint_counter = 0

        # Background thread for publishing heartbeat and setpoint
        self.heartbeat_thread = threading.Thread(target=self.heartbeat_loop)
        self.heartbeat_thread.daemon = True
        self.heartbeat_thread.start()
        self.get_logger().info("DroneAvoidance node initialized")

    def scan_cb(self, msg):
        self.scan = msg
        #self.get_logger().info("Received LaserScan data")

    def vehicle_local_position_callback(self, msg):
        self.local_position = msg
        #self.get_logger().info(f"Received VehicleLocalPosition: x={msg.x:.2f}, y={msg.y:.2f}, z={msg.z:.2f}")

    def vehicle_attitude_callback(self, msg):
        self.attitude = msg
        #self.get_logger().info(f"Received VehicleAttitude: q={msg.q}")

    def heartbeat_loop(self):
        while rclpy.ok():
            self.publish_offboard_control_heartbeat_signal(control_mode=self.mode)
            self.publish_current_setpoint()
            if self.offboard_setpoint_counter< 20:
               self.offboard_setpoint_counter = self.offboard_setpoint_counter + 1
            self.get_logger().debug(f"Published heartbeat, counter={self.offboard_setpoint_counter}")
            time.sleep(0.1)  # 10 Hz

    def publish_offboard_control_heartbeat_signal(self, control_mode='position'):
        """
        Publish the offboard control mode.

        Args:
            control_mode (str): The control mode to use. Options are 'position', 'velocity', 'acceleration',
                                'attitude', or 'body_rate'. Default is 'position'.
        """
        # Create an instance of OffboardControlMode message
        msg = OffboardControlMode()
        # Reset all control modes to False by default
        msg.position = False
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        # Set the appropriate control mode based on the argument
        if control_mode == 'position':
            msg.position = True
        elif control_mode == 'velocity':
            msg.velocity = True
        elif control_mode == 'acceleration':
            msg.acceleration = True
        elif control_mode == 'attitude':
            msg.attitude = True
        elif control_mode == 'body_rate':
            msg.body_rate = True
        else:
            # Handle invalid control mode
            self.get_logger().warn(f"Invalid control mode '{control_mode}'. Defaulting to 'position'.")
            msg.position = True  # Default to position if an invalid mode is passed
        # Set the current timestamp (in microseconds)
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)

        # Publish the message
        self.offboard_control_mode_publisher.publish(msg)

    def publish_current_setpoint(self):
        if self.current_setpoint is None:
            return

        msg = TrajectorySetpoint()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.acceleration = [float('nan')] * 3
        msg.jerk = [float('nan')] * 3

        if self.mode == 'position':
            msg.position = self.current_setpoint['position']
            msg.yaw = self.current_setpoint.get('yaw', float('nan'))
            msg.velocity = [float('nan')] * 3
            msg.yawspeed = float('nan')
            self.get_logger().info(f"Publishing px4 position setpoint: {msg.position}, yaw={msg.yaw:.2f}")
        elif self.mode == 'velocity':
            msg.velocity = self.current_setpoint['velocity']
            msg.yawspeed = self.current_setpoint.get('yawspeed', float('nan'))
            msg.position = [float('nan')] * 3
            msg.yaw = float('nan')
            self.get_logger().info(f"Publishing px4 velocity setpoint: {msg.velocity}, yawspeed={msg.yawspeed:.2f}")

        self.trajectory_setpoint_publisher.publish(msg)

    def update_setpoint(self, position=None, velocity=None, yawspeed=None):
        if position is not None:
            self.mode = 'position'
            if self.local_position is not None:
                cur_pos_ned = [self.local_position.x, self.local_position.y, self.local_position.z]
                yaw = math.atan2(position[1] - cur_pos_ned[1], position[0] - cur_pos_ned[0])
            else:
                yaw = float('nan')
            self.current_setpoint = {'position': position, 'yaw': yaw}
        elif velocity is not None:
            self.mode = 'velocity'
            self.current_setpoint = {'velocity': velocity, 'yawspeed': yawspeed or float('nan')}

    def engage_offboard_mode(self):
        """Switch to offboard mode."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        self.get_logger().info("Switching to offboard mode")

    def publish_vehicle_command(self, command, **params) -> None:
        """Publish a vehicle command."""
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = params.get("param1", 0.0)
        msg.param2 = params.get("param2", 0.0)
        msg.param3 = params.get("param3", 0.0)
        msg.param4 = params.get("param4", 0.0)
        msg.param5 = params.get("param5", 0.0)
        msg.param6 = params.get("param6", 0.0)
        msg.param7 = params.get("param7", 0.0)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher.publish(msg)

    def is_obstacle_ahead(self, safe_dist):
        if self.scan is None:
            return False
        angle_min = self.scan.angle_min
        increment = self.scan.angle_increment
        ranges = self.scan.ranges
        front_angle = 0.60  # about 60 degrees in radians
        min_dist = float('inf')
        for i, r in enumerate(ranges):
            theta = angle_min + i * increment
            if abs(theta) < front_angle and r < min_dist:
                min_dist = r
        self.get_logger().info(f"Minimum front distance: {min_dist:.2f}m")
        return min_dist < safe_dist
    
    
    def compute_dwa(self, target_ned):
        if self.local_position is None or self.attitude is None or self.scan is None:
            self.get_logger().warn("Missing data for DWA computation")
            return 0.0, 0.0

        cur_pos_ned = np.array([self.local_position.x, self.local_position.y, self.local_position.z])
        rel_ned = np.array(target_ned) - cur_pos_ned
        dcm = np.array(quaternion_to_dcm(self.attitude.q))
        rel_frd = dcm.T @ rel_ned
        rel_flu = np.array([rel_frd[0], -rel_frd[1], -rel_frd[2]])

        goal_x = rel_flu[0]
        goal_y = rel_flu[1]
        goal_heading = math.atan2(goal_y, goal_x)
        self.get_logger().info(f"Goal in FLU: x={goal_x:.2f}, y={goal_y:.2f}, heading={goal_heading:.2f} rad")

        # Obstacle points in FLU
        obs = []
        angle_min = self.scan.angle_min
        increment = self.scan.angle_increment
        ranges = self.scan.ranges
        for i in range(len(ranges)):
            r = ranges[i]
            if r < float('inf'):
                theta = angle_min + i * increment
                ox = r * math.cos(theta)
                oy = r * math.sin(theta)
                obs.append((ox, oy))
        self.get_logger().info(f"Number of obstacles detected: {len(obs)}")

        obs_points = np.array(obs)
        obs_tree = cKDTree(obs_points) if len(obs_points) > 0 else None

        # DWA parameters (优化后)
        max_v, min_v = 2.0, 0.0
        max_w = math.pi / 2
        min_w = -max_w
        v_samples, w_samples = 10, 20
        dt = 0.2
        predict_time = 1.0
        steps = int(predict_time / dt)
        robot_radius = 0.5
        alpha, beta, gamma = 0.5, 0.2, 0.3

        best_score = -float('inf')
        best_v = best_w = 0.0

        for v in np.linspace(min_v, max_v, v_samples):
            for w in np.linspace(min_w, max_w, w_samples):
                # Simulate trajectory
                x = y = th = 0.0
                traj = []
                for _ in range(steps):
                    x += v * math.cos(th) * dt
                    y += v * math.sin(th) * dt
                    th += w * dt
                    traj.append((x, y))

                # Check collision using KDTree
                collided = False
                min_dist = float('inf')
                for (tx, ty) in traj:
                    if obs_tree is not None:
                        dist, _ = obs_tree.query([tx, ty])
                        if dist <= robot_radius:
                            collided = True
                            break
                        if dist < min_dist:
                            min_dist = dist
                    else:
                        min_dist = max_v * predict_time

                if collided:
                    continue

                # Scoring
                heading_diff = math.pi - abs(th - goal_heading)
                heading_score = heading_diff / math.pi
                vel_score = v / max_v
                clear_score = min_dist / (max_v * predict_time)
                score = alpha * heading_score + beta * vel_score + gamma * clear_score

                if score > best_score:
                    best_score, best_v, best_w = score, v, w
         # Recovery if no valid trajectory found (all collided): Rotate away from nearest obstacle
        if best_score == -float('inf') or best_v == 0.0:
            self.get_logger().warning("No valid trajectory or best_v == 0.0; recovering by rotating away from nearest obstacle")
            if obs_tree is not None:
                _, nearest_idx = obs_tree.query([0, 0])
                ox, oy = obs_points[nearest_idx]
                nearest_theta = math.atan2(oy, ox) 
                
                #旋转方向 = 障碍物方向的符号
                best_w = math.copysign(max_w / 2, nearest_theta)
                
                # 限制旋转角度（避免过度旋转）
                if abs(nearest_theta) < 0.1:  # 障碍物在正前方
                    best_w = math.copysign(max_w / 2, 1)  # 默认左转
            else:
                best_w = max_w / 2  # 无障碍物时默认左转

        self.get_logger().info(f"Best DWA: vx={best_v:.2f}, w={best_w:.2f}, score={best_score:.2f}")
        return best_v, best_w

def main(args=None):
    rclpy.init(args=args)
    node = DroneAvoidance()

    # Background thread for spinning the node
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,))
    spin_thread.daemon = True
    spin_thread.start()

    
    while node.offboard_setpoint_counter <10:
      time.sleep(0.1)
    node.engage_offboard_mode()

    target_enu = [10.0, -10.0, 5.0]
    target_ned = enu_to_ned(target_enu)
    safe_dist = 3.5

    while rclpy.ok():
        if node.local_position is None or node.attitude is None or node.scan is None:
            time.sleep(0.1)
            continue

        cur_pos_ned = [node.local_position.x, node.local_position.y, node.local_position.z]

        # Check if reached target
        if np.linalg.norm(np.array(target_ned) - np.array(cur_pos_ned)) < 1.0:
            node.get_logger().info("Target reached")
            break

        # Rule to judge entering/exiting DWA mode
        obstacle_ahead = node.is_obstacle_ahead(safe_dist)
        if not node.dwa_active and obstacle_ahead:
            node.dwa_active = True
            node.get_logger().info("Entering DWA mode")
        elif node.dwa_active and not node.is_obstacle_ahead(safe_dist * 1.5):
            node.dwa_active = False
            node.get_logger().info("Exiting DWA mode")

        if not node.dwa_active:
            # Global planning: position mode
            setpoint_pos = target_ned
            node.update_setpoint(position=setpoint_pos)
        else:
            # Local avoidance: DWA in velocity mode
            time_start = time.time()
            v, w_flu = node.compute_dwa(target_ned)
            time_elapsed = time.time() - time_start
            print(f"dwa takes {time_elapsed:.4f} s")
            w_frd = -w_flu
            body_vel_frd = np.array([v, 0.0, 0.0])
            dcm = np.array(quaternion_to_dcm(node.attitude.q))
            ned_vel = np.dot(dcm, body_vel_frd)
            setpoint_vel = ned_vel.tolist()
            node.update_setpoint(velocity=setpoint_vel, yawspeed=w_frd)

        time.sleep(0.1)

    rclpy.shutdown()
    spin_thread.join()

if __name__ == '__main__':
    main()