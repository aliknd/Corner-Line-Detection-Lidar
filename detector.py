#! /usr/bin/env python

from numpy.lib.shape_base import split
import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import LaserScan
import matplotlib.pyplot as plt
import math
import re

from matplotlib import pyplot as plt

class FeatureExtracter(Node):
    def __init__(self):
        super().__init__('feature_extracter')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10)
        self.plot = 0
        self.publisher_ = self.create_publisher(LaserScan, '/feature_scan', 10)
        self.front_publisher = self.create_publisher(LaserScan, '/front_scan', 10)
  
    def polar_to_cartesian_coordinate(self, ranges, angle_min, angle_max):
        angle_step = (angle_max - angle_min) / len(ranges)
        angle = 0
        points = []
        for range in ranges:
            x = range * np.cos(angle)
            y = range * np.sin(angle)
            angle += angle_step
            points.append([x,y])


        
        return points

    def getAngle(self, a, b, c):
        ang = math.degrees(math.atan2(c[1]-b[1], c[0]-b[0]) - math.atan2(a[1]-b[1], a[0]-b[0]))
        return ang + 360 if ang < 0 else ang

    def split(arr, size):
        arrs = []
        while len(arr) > size:
            pice = arr[:size]
            arrs.append(pice)
            arr   = arr[size:]
        arrs.append(arr)
        return arrs
    
    def scan_callback(self,msg):

        # self.polar_to_cartesian_coordinate(msg.ranges, msg.angle_min, msg.angle_max)

        #################################
        ranges = []
    
        for r in msg.ranges:
            if r > 2.5 or r < 1:   # Feature extraction code here
                ranges.append(0.0)
            else:
                ranges.append(r)
        ################################## 



        scan = LaserScan()
        scan.header.stamp = msg.header.stamp
        scan.header.frame_id = msg.header.frame_id
        scan.angle_min = msg.angle_min
        scan.angle_max = msg.angle_max
        scan.angle_increment = msg.angle_increment
        scan.time_increment = msg.time_increment
        scan.range_min = msg.range_min
        scan.range_max = msg.range_max 
        scan.ranges = ranges
        
        self.publisher_.publish(scan)

        front_ranges = []
        for key, range in enumerate(msg.ranges):
            if key > 60 and key < 300:   # Feature extraction code here
                front_ranges.append(0.0)
            else:
                front_ranges.append(range)

        front_scan = LaserScan()
        front_scan.header.stamp = msg.header.stamp
        front_scan.header.frame_id = msg.header.frame_id
        front_scan.angle_min = msg.angle_min
        front_scan.angle_max = msg.angle_max
        front_scan.angle_increment = msg.angle_increment
        front_scan.time_increment = msg.time_increment
        front_scan.range_min = msg.range_min
        front_scan.range_max = msg.range_max 
        front_scan.ranges = front_ranges

        self.front_publisher.publish(front_scan)
    
        
        if self.plot == 0:
            self.plot = 1
            points = self.polar_to_cartesian_coordinate(msg.ranges, msg.angle_min, msg.angle_max)
            points2 = self.polar_to_cartesian_coordinate(front_scan.ranges, front_scan.angle_min, front_scan.angle_max)
            points_np = np.array(points)
            points2_np = np.array(points2)
            fig, (ax1, ax2) = plt.subplots(2, constrained_layout=True)
            fig.suptitle('Cartesian coordinates')
            ax1.scatter(points_np[:,0], points_np[:,1])
            ax1.set_title('Original point cloud')
            ax2.scatter(points2_np[:,0], points2_np[:,1])
            ax2.set_title('Front point cloud')
            plt.show()

            fig2, (ax3, ax4) = plt.subplots(2, constrained_layout=True)
            fig2.suptitle('Polar coordinates')
            angles = np.arange(360)
            ax3.scatter(angles, msg.ranges)
            ax3.set_title('Original point cloud')
            ax4.scatter(angles, front_scan.ranges)
            ax4.set_title('Front point cloud')
            plt.show()

        points = self.polar_to_cartesian_coordinate(msg.ranges, msg.angle_min, msg.angle_max)
        
        # Corner Detection
        
        array_t = []
        for key, range in enumerate(points, start=1):
            if key < 352:
                a = points[key]
                b = points[key+3]
                c = points[key+8]
                calculated_angle = self.getAngle(a, b, c)
                calculated_angle = calculated_angle - 180
                if calculated_angle > 75:
                    array_t.append(a)
                    array_t.append(b)
                    array_t.append(c)
                    points_np1 = np.array(array_t)
                    fig, ax = plt.subplots()
                    fig.suptitle('Corner Detection')
                    ax.scatter(points_np1[:,0], points_np1[:,1])
                    ax.set_title('Original points')
                    plt.show()

        # Line detection

        eight_split = np.array_split(points, 45)
        pd1_array_m = []
        pd1_array_b = []
        pd2_array_m = []
        pd2_array_b = []
        for key, range in enumerate(eight_split):
            if key % 2 == 0:
                xs1 = range[:,0]
                ys1 = range[:,1]
                #print(key, xs1)
                #print("-----------------------1")
                pf1 = np.polyfit(np.array(xs1), np.array(ys1), 1)
                pd1 = np.poly1d(pf1)
                pd1_array_m.append(pd1[1])
                pd1_array_b.append(pd1[0])
                # print(pd1)
                # print(pd1[1])


            else:
                xs2 = range[:,0]
                ys2 = range[:,1]
                #print(key, xs2)
                #print("-----------------------2")
                pf2 = np.polyfit(np.array(xs2), np.array(ys2), 1)
                pd2 = np.poly1d(pf2)
                pd2_array_m.append(pd2[1])
                pd2_array_b.append(pd2[0])
                # print(pd2)
                # print(pd2[1])

        # print("-----------------------3")
        # print(pd1_array_m)
        # print("-----------------------4")
        # print(pd2_array_m)

        count = 0
        while(count < 22):
            if 0 < abs(pd1_array_m[count]) < 1 and 2 < abs(pd2_array_b[count]) < 3:
                print("in the same line")
            else:
                print("not in the same line")
            count = count+1
        print("---------------end line")
        
def main(args=None):
    rclpy.init(args=args)
    feature_extracter = FeatureExtracter()
    rclpy.spin(feature_extracter) 
    feature_extracter.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()