#!/usr/bin/env python
############################################################################
#
#   Copyright (C) 2022 PX4 Development Team. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name PX4 nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
############################################################################

__author__ = "Jaeyoung Lim, Vinicius Abrao"
__contact__ = "jalim@ethz.ch, vinicius.abrao@hotmail.com"

import rclpy
import numpy as np
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import VehicleStatus
from px4_msgs.msg import VehicleCommand
from px4_msgs.msg import VehicleControlMode
from px4_msgs.msg import VehicleAttitude
from px4_msgs.msg import VehicleLocalPosition

from csv import DictWriter
import pymap3d as pm

class OffboardControl(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=1
        )


        # Define Subscribers
        self.attitude_sub = self.create_subscription(
            VehicleAttitude,
            '/fmu/out/vehicle_attitude',
            self.vehicle_attitude_callback,
            qos_profile)
        self.local_position_sub = self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position',
            self.vehicle_local_position_callback,
            qos_profile)

        self.status_sub = self.create_subscription(
            VehicleStatus,
            '/fmu/out/vehicle_status',
            self.vehicle_status_callback,
            qos_profile)
        
        # Define Publishers
        self.offboard_control_mode_publisher_ = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.trajectory_setpoint_publisher_ = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.vehicle_command_publisher_ = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', qos_profile)
        timer_period = 0.02  # seconds
        self.timer = self.create_timer(timer_period, self.cmdloop_callback)

        self.data_filename = 'data.csv'



        # Initial

        self.nav_state = VehicleStatus.NAVIGATION_STATE_MAX
        self.dt = timer_period
        self.theta = 0.0
        self.radius = 10.0
        self.omega = 0.5
        self.offboard_setpoint_counter_ = 0
        self.vehicle_attitude = np.array([1.0, 0.0, 0.0, 0.0])
        self.vehicle_local_position = np.array([0.0, 0.0, 0.0])
        self.vehicle_local_velocity = np.array([0.0, 0.0, 0.0])
        self.ref_lat = 0
        self.ref_lon = 0
        self.ref_alt = 0

        # Net Parameters
        # [lat, lon] in degrees
        self.net_corners = np.array([[29.628399, -82.360596],  # top right
                                     [29.628260, -82.360704],  # top left
                                     [29.627893, -82.360081],  # bottom left
                                     [29.628035, -82.359972]]) # bottom right
        self.local_corners_set = False

 
    def vehicle_status_callback(self, msg):
        # TODO: handle NED->ENU transformation
        print("NAV_STATUS: ", msg.nav_state)
        print("  - offboard status: ", VehicleStatus.NAVIGATION_STATE_OFFBOARD)
        self.nav_state = msg.nav_state

    def cmdloop_callback(self):
        # self.get_logger().info(" IN COMMAND LOOP!!!!")
        

        if self.offboard_setpoint_counter_ == 50:
            # Change to Offboard mode after 50 setpoints (1s)
            self.engage_offBoard_mode()

            # Get local-frame net corner values
            if not self.local_corners_set:
                self.get_local_net_corners_enu()
              
            # Arm the vehicle
            self.arm()
           

            
        if self.offboard_setpoint_counter_ < 550:
            # offboard_control_mode needs to be paired with trajectory_setpoint
            print("start counter")
            self.publish_offboard_control_mode_position()
            if self.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
                self.publish_trajectory_setpoint_position(0.0, 0.0, -5.0, -3.14)
            self.offboard_setpoint_counter_ += 1

        if self.offboard_setpoint_counter_ >= 550 and self.offboard_setpoint_counter_ < 1650:
            # offboard_control_mode needs to be paired with trajectory_setpoint
            print("start counter")
            self.publish_offboard_control_mode_velocity()
            if self.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
                self.publish_trajectory_setpoint_circle()
            self.offboard_setpoint_counter_ += 1
        
        if self.offboard_setpoint_counter_ >= 1650 and self.offboard_setpoint_counter_ < 2250:
            # offboard_control_mode needs to be paired with trajectory_setpoint
            print("start counter")
            self.publish_offboard_control_mode_position()
            if self.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
                self.publish_trajectory_setpoint_position(0.0, 0.0, -5.0, 0.0)
            self.offboard_setpoint_counter_ += 1

        if self.offboard_setpoint_counter_ == 2250:
            # Land and cancel timer after (38s)
            self.land()
            self.timer.cancel()

        self.record_state(self.data_filename)
    

    def arm(self):
        self.get_logger().info("Arm command sent")
        msg = VehicleCommand()
        msg.param1 = 1.0
        msg.command = VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
        self.publish_vehicle_command(msg)

    def disarm(self):
        print('Disarm command sent')
        msg = VehicleCommand()
        msg.command = VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
        self.publish_vehicle_command(msg)
    
    def land(self):
        print('Land command sent')
        msg = VehicleCommand()
        msg.command = VehicleCommand.VEHICLE_CMD_NAV_LAND
        self.publish_vehicle_command(msg)
        
    def publish_offboard_control_mode_velocity(self):
        msg = OffboardControlMode()
        msg.position = False
        msg.velocity = True
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(Clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher_.publish(msg)

    def publish_offboard_control_mode_position(self):
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(Clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher_.publish(msg)

    def engage_offBoard_mode(self):
        self.get_logger().info('Offboard mode command sent')
        msg = VehicleCommand()
        msg.param1 = 1.0
        msg.param2 = 6.0
        msg.command = VehicleCommand.VEHICLE_CMD_DO_SET_MODE
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.publish_vehicle_command(msg)
        
    def publish_trajectory_setpoint_position(self,x,y,z,yaw):
        msg = TrajectorySetpoint()
        
        # msg.position = [0.0, 0.0, -5.0]
        # msg.yaw = -3.14
        
        msg.position = [x,y,z]
        msg.yaw = yaw

        msg.timestamp = int(Clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher_.publish(msg)

        
    def publish_trajectory_setpoint_circle(self):
        msg = TrajectorySetpoint()
              
        # msg.position[0] = self.radius * np.cos(self.theta)
        # msg.position[1] = self.radius * np.sin(self.theta)
        # msg.position[2] = -5.0
        
        msg.timestamp = int(Clock().now().nanoseconds / 1000)
        msg.velocity[0] = -self.radius * self.omega * np.sin(self.theta)
        msg.velocity[1] =  self.radius * self.omega * np.cos(self.theta)
        msg.velocity[2] = 0.0
        msg.position[0] = float('nan')
        msg.position[1] = float('nan')
        msg.position[2] = float('nan')
        msg.acceleration[0] = float('nan')
        msg.acceleration[1] = float('nan')
        msg.acceleration[2] = float('nan')
        msg.yaw = float('nan')
        msg.yawspeed = float('nan')

        # msg.velocity[0] = float('nan')
        # msg.velocity[1] = float('nan')
        # msg.velocity[2] = float('nan')
        # msg.position[0] = float('nan')
        # msg.position[1] = float('nan')
        # msg.position[2] = float('nan')
        # msg.acceleration[0] = -self.radius * self.omega * self.omega * np.cos(self.theta)
        # msg.acceleration[1] = -self.radius * self.omega * self.omega * np.sin(self.theta)
        # msg.acceleration[2] = 0.0
        # msg.yaw = float('nan')
        # msg.yawspeed = float('nan')


        msg.timestamp = int(Clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher_.publish(msg)
        
        self.theta = self.theta + self.omega * self.dt

    def publish_vehicle_command(self, msg):
        msg.timestamp = int(Clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher_.publish(msg)

    def vehicle_attitude_callback(self, msg):
        # TODO: handle NED->ENU transformation 
        self.vehicle_attitude[0] = msg.q[0]
        self.vehicle_attitude[1] = msg.q[1]
        self.vehicle_attitude[2] = -msg.q[2]
        self.vehicle_attitude[3] = -msg.q[3]

    def vehicle_local_position_callback(self, msg):
        # TODO: handle NED->ENU transformation 
        self.vehicle_local_position[0] = msg.x
        self.vehicle_local_position[1] = -msg.y
        self.vehicle_local_position[2] = -msg.z
        self.vehicle_local_velocity[0] = msg.vx
        self.vehicle_local_velocity[1] = -msg.vy
        self.vehicle_local_velocity[2] = -msg.vz
            
        if msg.xy_global:
            self.ref_lat = msg.ref_lat
            self.ref_lon = msg.ref_lon
            
        if msg.z_global:
            self.ref_alt = msg.ref_alt


    def record_state(self, filename):
        rows = [self.vehicle_local_position[0], self.vehicle_local_position[1], self.vehicle_local_position[2],\
                self.vehicle_local_velocity[0], self.vehicle_local_velocity[1], self.vehicle_local_velocity[2],\
                self.vehicle_attitude[0], self.vehicle_attitude[1], self.vehicle_attitude[2], self.vehicle_attitude[3]]

        # self.get_logger().info('rows: {}'.format(rows))


        field_names = ['time','pos_x','pos_y','pos_z','vel_x','vel_y','vel_z','q0','q1','q2','q3']

        time = self.offboard_setpoint_counter_ * self.dt
        dict = {
            'time': time,
            'pos_x': self.vehicle_local_position[0],
            'pos_y': self.vehicle_local_position[1],
            'pos_z': self.vehicle_local_position[2],
            'vel_x': self.vehicle_local_velocity[0],
            'vel_y': self.vehicle_local_velocity[1],
            'vel_z': self.vehicle_local_velocity[2],
            'q0': self.vehicle_attitude[0],
            'q1': self.vehicle_attitude[1],
            'q2': self.vehicle_attitude[2],
            'q3': self.vehicle_attitude[3]
        }



        with open(filename, 'a') as f_object:

            dictwriter_object = DictWriter(f_object, fieldnames=field_names)

            dictwriter_object.writerow(dict)

            f_object.close()


    def get_local_net_corners_enu(self):
        num_corners = self.net_corners.shape[0]
        self.local_corner_enu = np.zeros((num_corners,2))

        for i in range(num_corners):

            enu = pm.geodetic2enu(self.net_corners[i,0], self.net_corners[i,1], self.ref_alt, self.ref_lat, self.ref_lon, self.ref_alt)
            self.local_corner_enu[i,0] = enu[0]
            self.local_corner_enu[i,1] = enu[1]
            self.get_logger().info('enu: {}'.format(enu))

        self.local_corners_set = True


        self.get_logger().info('local_corner_enu: {}'.format(self.local_corner_enu))

        # Testing distances
        d1 = np.linalg.norm(self.local_corner_enu[0,:] - self.local_corner_enu[1,:])
        d2 = np.linalg.norm(self.local_corner_enu[1,:] - self.local_corner_enu[2,:])
        d3 = np.linalg.norm(self.local_corner_enu[2,:] - self.local_corner_enu[3,:])
        d4 = np.linalg.norm(self.local_corner_enu[3,:] - self.local_corner_enu[1,:])

        self.get_logger().info('d1: {} m'.format({d1}))
        self.get_logger().info('d2: {} m'.format({d2}))
        self.get_logger().info('d3: {} m'.format({d3}))
        self.get_logger().info('d4: {} m'.format({d4}))

        self.build_netline_params()


    def build_netline_params(self):
        num_sides = 4
        num_params = 3 # [a,b,c]
        self.netline_params = np.zeros((num_sides, num_params))
        

        for i in range(num_sides):
            
            x1 = np.local_corner_enu[i,0]
            y1 = np.local_corner_enu[i,1]
        

            if i != (num_sides-1):
                x2 = np.local_corner_enu[i+1,0]
                y2 = np.local_corner_enu[i+1,1]

            else:
                x2 = np.local_corner_enu[0,0]
                y2 = np.local_corner_enu[0,1]
             
            a = y1 - y2
            b = x2 - x1
            c = y2*x1 - x2*y1

            self.netline_params[i,0] = a
            self.netline_params[i,1] = b
            self.netline_params[i,2] = c




    def dist_to_net(self):

        







def main(args=None):
    rclpy.init(args=args)

    offboard_control = OffboardControl()

    rclpy.spin(offboard_control)

    offboard_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()