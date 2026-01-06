#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import cv2
import time
import os
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import String

class AdvancedFireBot:
    def __init__(self):
        rospy.init_node('final_project_bot', anonymous=True)
        
        # === 訂閱與發布 ===
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.image_callback)
        self.scan_sub = rospy.Subscriber("/scan", LaserScan, self.scan_callback)
        self.cmd_sub = rospy.Subscriber("/control_center", String, self.command_callback)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        
        self.bridge = CvBridge()
        self.rate = rospy.Rate(10)

        # === 系統狀態 ===
        # 狀態機: PATROL -> EXTINGUISHING -> WAITING -> ESCAPING -> PATROL
        self.state = "PATROL" 
        self.obstacle_detected = False
        
        # 計時器
        self.extinguish_start_time = 0
        self.extinguish_duration = 5.0 # 滅火動作 5 秒
        
        self.escape_start_time = 0     # 新增：撤離計時器
        self.escape_duration = 3.0     # 新增：撤離旋轉 3 秒

        # 資料存檔路徑 (存於程式所在位置)
        self.log_path = os.path.dirname(os.path.abspath(__file__))

        # 紅色閾值
        self.lower_red1 = np.array([0, 100, 100])
        self.upper_red1 = np.array([10, 255, 255])
        self.lower_red2 = np.array([160, 100, 100])
        self.upper_red2 = np.array([180, 255, 255])

        rospy.loginfo("System Started! Ready for Action.")
        rospy.loginfo("Waiting for command... (Send 'RESUME' to /control_center to reset)")

    def command_callback(self, msg):
        command = msg.data.strip().upper()
        if command == "RESUME":
            if self.state == "WAITING":
                # === 關鍵修改：收到指令後，先進入逃跑狀態 ===
                rospy.loginfo("Command Received: Turning away from hazard...")
                self.state = "ESCAPING"
                self.escape_start_time = time.time()
            else:
                rospy.loginfo("Ignoring RESUME (Not in Waiting state)")
        elif command == "STOP":
            self.state = "WAITING"
            rospy.loginfo("Emergency Stop triggered by Operator.")

    def scan_callback(self, msg):
        ranges = np.array(msg.ranges)
        ranges[ranges == 0.0] = float('inf')
        # 檢查前方 60 度
        front = np.concatenate((ranges[0:30], ranges[-30:]))
        if np.min(front) < 0.3:
            self.obstacle_detected = True
        else:
            self.obstacle_detected = False

    def image_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            return

        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.lower_red1, self.upper_red1) + \
               cv2.inRange(hsv, self.lower_red2, self.upper_red2)
        red_pixels = cv2.countNonZero(mask)

        # 顯示狀態
        # 根據狀態顯示不同顏色的文字
        color = (0, 255, 0) if self.state == "PATROL" else (0, 0, 255)
        cv2.putText(cv_image, f"Mode: {self.state}", (10, 30), 
                    cv2.FONT_HERSHEY_SIMPLEX, 1, color, 2)
        
        cv2.imshow("Operator View", cv_image)
        cv2.waitKey(3)

        # 狀態轉移：只有在巡邏模式下才偵測火災
        if self.state == "PATROL" and red_pixels > 25000:
            rospy.logwarn("FIRE DETECTED! Starting Extinguishing Protocol...")
            self.state = "EXTINGUISHING"
            self.extinguish_start_time = time.time()
            self.save_evidence(cv_image)

    def save_evidence(self, image):
        timestamp = time.strftime("%Y%m%d-%H%M%S")
        img_name = f"{self.log_path}/evidence_{timestamp}.jpg"
        cv2.imwrite(img_name, image)
        
        log_name = f"{self.log_path}/mission_log.txt"
        with open(log_name, "a") as f:
            f.write(f"[{timestamp}] EVENT: Fire Extinguished. Evidence: {img_name}\n")

    def perform_extinguishing_action(self, move_cmd):
        elapsed = time.time() - self.extinguish_start_time
        if elapsed > self.extinguish_duration:
            rospy.loginfo("Fire Extinguished. Waiting for operator confirmation...")
            self.state = "WAITING"
            move_cmd.linear.x = 0.0
            move_cmd.angular.z = 0.0
        else:
            # 左右搖擺
            move_cmd.linear.x = 0.0
            if int(elapsed * 4) % 2 == 0:
                move_cmd.angular.z = 0.5
            else:
                move_cmd.angular.z = -0.5

    def run(self):
        move_cmd = Twist()
        
        while not rospy.is_shutdown():
            if self.state == "PATROL":
                if self.obstacle_detected:
                    move_cmd.linear.x = 0.0
                    move_cmd.angular.z = 0.5
                else:
                    move_cmd.linear.x = 0.2
                    move_cmd.angular.z = 0.05
                    
            elif self.state == "EXTINGUISHING":
                self.perform_extinguishing_action(move_cmd)
                
            elif self.state == "WAITING":
                move_cmd.linear.x = 0.0
                move_cmd.angular.z = 0.0
            
            # === 新增：撤離模式 ===
            elif self.state == "ESCAPING":
                elapsed = time.time() - self.escape_start_time
                if elapsed > self.escape_duration:
                    # 時間到，轉回巡邏模式
                    self.state = "PATROL"
                    rospy.loginfo("Safe distance reached. Resuming Patrol.")
                else:
                    # 動作：原地快速右轉 (背對火源)
                    move_cmd.linear.x = 0.0
                    move_cmd.angular.z = -0.8 
            
            self.cmd_vel_pub.publish(move_cmd)
            self.rate.sleep()

if __name__ == '__main__':
    try:
        bot = AdvancedFireBot()
        bot.run()
    except rospy.ROSInterruptException:
        pass
    finally:
        cv2.destroyAllWindows()