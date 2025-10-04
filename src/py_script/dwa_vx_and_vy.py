'''
writer: liu Yongxue
email: 805110687@qq.com
'''

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
        front_angle = 0.5236  # 30 degrees in radians
        min_dist = float('inf')
        for i, r in enumerate(ranges):
            theta = angle_min + i * increment
            if abs(theta) < front_angle and r < min_dist:
                min_dist = r
        self.get_logger().info(f"Minimum front distance: {min_dist:.2f}m")
        return min_dist < safe_dist
    
  


    def compute_dwa(self, target_ned):
        """
        Compute the best velocity commands using Dynamic Window Approach (DWA) for 2D obstacle avoidance.
        This function operates in the FLU (Forward-Left-Up) radar coordinate frame.
        It samples possible velocities (vx, vy, w), simulates trajectories, checks for collisions using a KDTree,
        scores them based on heading alignment, speed, and clearance, and selects the best.
        
        Args:
            target_ned (list or array): Target position in NED frame [x, y, z].
        
        Returns:
            tuple: (best_vx, best_vy, best_w) - Linear velocities in x/y (FLU) and angular velocity w.
        """
        # Check for required data; warn and return zero velocities if missing
        if self.local_position is None or self.attitude is None or self.scan is None:
            self.get_logger().warn("Missing data for DWA computation")
            return 0.0, 0.0, 0.0  # vx, vy, w

        # Compute relative target position in NED frame
        cur_pos_ned = np.array([self.local_position.x, self.local_position.y, self.local_position.z])
        rel_ned = np.array(target_ned) - cur_pos_ned
        
        # Convert quaternion to DCM (body FRD to NED)
        dcm = np.array(quaternion_to_dcm(self.attitude.q))
        
        # Transform relative position to FRD (body frame), then to FLU (radar frame: x forward, y left, z up)
        rel_frd = dcm.T @ rel_ned  # Note: dcm.T transforms from NED to FRD
        rel_flu = np.array([rel_frd[0], -rel_frd[1], -rel_frd[2]])  # FLU: negate y and z for left-up convention
        
        # Compute goal heading in FLU frame for scoring
        goal_x = rel_flu[0]
        goal_y = rel_flu[1]
        goal_heading = math.atan2(goal_y, goal_x)
        self.get_logger().info(f"Goal in FLU: x={goal_x:.2f}, y={goal_y:.2f}, heading={goal_heading:.2f} rad")

        # Extract and filter obstacle points from LaserScan in FLU frame
        obs = []
        angle_min = self.scan.angle_min
        increment = self.scan.angle_increment
        ranges = self.scan.ranges
        for i in range(len(ranges)):
            r = ranges[i]
            if not math.isinf(r) and not math.isnan(r) and r > 0:  # Filter invalid or zero ranges
                theta = angle_min + i * increment
                ox = r * math.cos(theta)  # x forward
                oy = r * math.sin(theta)  # y left (positive to left in FLU)
                obs.append((ox, oy))
        self.get_logger().info(f"Number of obstacles detected: {len(obs)}")

        # Build KDTree for fast nearest neighbor queries; handle empty case
        obs_points = np.array(obs) if obs else np.empty((0, 2))
        obs_tree = cKDTree(obs_points) if len(obs_points) > 0 else None

        # DWA parameters: Tunable for drone dynamics and environment
        max_speed = 2.0  # Max linear speed (m/s)
        min_speed = 0.0  # Min linear speed; set negative to allow reversing
        max_w = math.pi / 2  # Max angular speed (rad/s)
        min_w = -max_w
        v_samples = 10  # Number of samples for vx and vy
        w_samples = 20  # Number of samples for w
        dt = 0.2  # Time step for simulation (s)
        predict_time = 1.0  # Prediction horizon (s)
        steps = int(predict_time / dt)  # Number of simulation steps
        robot_radius = 0.5  # Safety buffer around drone (m)
        alpha, beta, gamma = 0.2, 0.2, 0.6  # Weights: heading (alignment to goal), velocity (prefer speed), clearance (avoid obstacles)

        best_score = -float('inf')
        best_vx = best_vy = best_w = 0.0

        # Sample velocities: vx, vy in circular space (<= max_speed), w in [min_w, max_w]
        for vx in np.linspace(-max_speed, max_speed, v_samples):
            for vy in np.linspace(-max_speed, max_speed, v_samples):
                speed = math.sqrt(vx**2 + vy**2)
                if speed > max_speed + 1e-6:  # Skip if exceeds max_speed (circular bound)
                    continue
                if speed < min_speed:  # Optional: Enforce min speed if desired
                    continue

                for w in np.linspace(min_w, max_w, w_samples):
                    # Simulate trajectory in FLU frame using forward kinematics (body velocities)
                    # Start at (0,0,0); integrate with rotation for curved paths
                    x = y = th = 0.0
                    traj = []
                    for _ in range(steps):
                        # Compute position increments in current orientation
                        dx = vx * math.cos(th) - vy * math.sin(th)
                        dy = vx * math.sin(th) + vy * math.cos(th)
                        x += dx * dt
                        y += dy * dt
                        th += w * dt
                        traj.append((x, y))

                    # Collision check using KDTree
                    collided = False
                    min_dist = float('inf')
                    for (tx, ty) in traj:
                        if obs_tree is not None:
                            dist, _ = obs_tree.query([tx, ty])  # Nearest obstacle distance
                            if dist <= robot_radius:
                                collided = True
                                break
                            if dist < min_dist:
                                min_dist = dist
                    if collided:
                        continue

                    # If no obstacles, set max possible clearance
                    if obs_tree is None:
                        min_dist = max_speed * predict_time

                    # Scoring components
                    # Heading: Normalize difference between final orientation and goal heading [0,1]
                    heading_diff = math.pi - abs(th - goal_heading)
                    heading_score = heading_diff / math.pi

                    # Velocity: Encourage higher speeds [0,1]
                    vel_score = speed / max_speed if max_speed > 0 else 0.0

                    # Clearance: Distance to nearest obstacle along trajectory [0,1]
                    clear_score = min_dist / (max_speed * predict_time + 1e-6)

                    # Weighted total score
                    score = alpha * heading_score + beta * vel_score + gamma * clear_score

                    # Update best if improved
                    if score > best_score:
                        best_score = score
                        best_vx, best_vy, best_w = vx, vy, w

        # Recovery if no valid trajectory found (all collided): Rotate away from nearest obstacle
        if best_score == -float('inf') or (best_vx == 0.0 and best_vy ==0.0):
            self.get_logger().warning("No valid trajectory or 'best_vx == 0.0 and best_vy ==0'; recovering by rotating away from nearest obstacle")
            if obs_tree is not None:
                _, nearest_idx = obs_tree.query([0, 0])  # Nearest to current position
                nearest_theta = math.atan2(obs_points[nearest_idx][1], obs_points[nearest_idx][0])
                best_w = math.copysign(max_w / 2, nearest_theta)  # Half max angular speed, opposite direction
        

        # Log the selected commands for debugging
        self.get_logger().info(f"Best DWA: vx={best_vx:.2f}, vy={best_vy:.2f}, w={best_w:.2f}, score={best_score:.2f}")

        return best_vx, best_vy, best_w


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

    target_enu = [10.0, -15.0, 5.0]
    target_ned = enu_to_ned(target_enu)
    safe_dist = 3.0

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
            vx, vy, w_flu = node.compute_dwa(target_ned)
            time_elapsed = time.time() - time_start
            print(f"dwa takes {time_elapsed:.4f} s")
            w_frd = -w_flu
            body_vel_frd = np.array([vx, -vy, 0.0])
            dcm = np.array(quaternion_to_dcm(node.attitude.q))
            ned_vel = np.dot(dcm, body_vel_frd)
            setpoint_vel = ned_vel.tolist()
            node.update_setpoint(velocity=setpoint_vel, yawspeed=w_frd)

        time.sleep(0.1)

    rclpy.shutdown()
    spin_thread.join()

if __name__ == '__main__':
    main()
