#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import actionlib
import yaml
import time
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import String
from geometry_msgs.msg import Twist

class TaskManager:
    def __init__(self):
        rospy.init_node('task_manager_node')
        
        # Dosya yolları
        self.mission_file = rospy.get_param('~mission_file', '/home/ozdemir/catkin_ws/src/final_odev/config/mission.yaml')
        self.report_file = '/home/ozdemir/catkin_ws/src/final_odev/temizlik_raporu.txt'

        # Hareket İstemcileri
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        
        rospy.loginfo("Navigasyon servisi bekleniyor...")
        self.client.wait_for_server()
        rospy.loginfo("Navigasyon servisi hazir!")

        # QR Dinleyicisi
        self.last_qr_data = None
        rospy.Subscriber("/qr_code_read", String, self.qr_callback)

        # Raporlama Defteri
        self.report = {} 

    def qr_callback(self, msg):
        self.last_qr_data = msg.data

    def load_mission(self):
        try:
            with open(self.mission_file, 'r') as file:
                return yaml.safe_load(file)
        except Exception as e:
            rospy.logerr(f"Dosya okunamadi: {e}")
            return None

    # --- ROBOTU HAREKET ETTIREN YARDIMCI FONKSIYON ---
    def move_robot(self, lin_x, ang_z, duration):
        """
        Robotu belirli bir hızda ve sürede manuel hareket ettirir.
        lin_x: İleri/Geri hızı (Geri için eksi ver)
        ang_z: Dönüş hızı (Sağ için eksi, Sol için artı)
        duration: Ne kadar süre hareket edeceği
        """
        msg = Twist()
        msg.linear.x = lin_x
        msg.angular.z = ang_z
        self.cmd_vel_pub.publish(msg)
        rospy.sleep(duration)
        
        # Durdur
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.cmd_vel_pub.publish(msg)
        rospy.sleep(1.0) # Kameranın odaklanması için bekle

    # --- QR KONTROL VE KURTARMA SENARYOSU (SENIN ISTEDIGIN KISIM) ---
    def smart_recovery_and_check(self, target_qr):
        """
        1. Önce normal bakar.
        2. Bulamazsa GERİ gelir bakar.
        3. Bulamazsa SAĞA döner bakar.
        4. Bulamazsa SOLA döner bakar.
        """
        
        # Adım 0: Olduğun yerde bir bak (3 saniye)
        rospy.loginfo("QR araniyor (Sabit)...")
        if self.check_qr_with_timeout(target_qr, 3.0): return True

        # Adım 1: Biraz GERİ gel (-0.15 m/s hızla 1.5 saniye)
        rospy.logwarn("QR yok! 1. Hamle: Biraz GERI geliniyor...")
        self.move_robot(-0.15, 0.0, 1.5)
        if self.check_qr_with_timeout(target_qr, 2.0): return True

        # Adım 2: Hafif SAĞA bak (Dönüş hızı -0.4 ile 1.5 saniye)
        rospy.logwarn("Hala yok! 2. Hamle: Hafif SAGA bakiliyor...")
        self.move_robot(0.0, -0.4, 1.5)
        if self.check_qr_with_timeout(target_qr, 2.0): return True

        # Adım 3: Hafif SOLA bak (Önce ortaya gelmesi lazım, o yüzden daha uzun süre sola dönüyoruz)
        rospy.logwarn("Hala yok! 3. Hamle: Hafif SOLA bakiliyor...")
        self.move_robot(0.0, 0.4, 3.0) # Sağa dönüşü telafi edip sola geçmek için süre uzun
        if self.check_qr_with_timeout(target_qr, 2.0): return True

        # Hepsini denedik, olmadı
        return False

    def check_qr_with_timeout(self, target, timeout):
        start = time.time()
        while time.time() - start < timeout:
            if self.last_qr_data == target:
                return True
            time.sleep(0.1)
        return False

    def go_to_goal(self, x, y, z, w):
        try:
            x, y, z, w = float(x), float(y), float(z), float(w)
        except ValueError:
            rospy.logerr("Koordinat hatasi! Sayi degil.")
            return False

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.orientation.z = z
        goal.target_pose.pose.orientation.w = w

        self.client.send_goal(goal)
        wait = self.client.wait_for_result(rospy.Duration(90))
        
        if not wait:
            self.client.cancel_goal()
            return False
        return self.client.get_state() == actionlib.GoalStatus.SUCCEEDED

    def perform_cleaning_tour(self, points, room_name):
        rospy.loginfo(f"--- {room_name.upper()} TEMIZLENIYOR ---")
        for i, point in enumerate(points):
            rospy.loginfo(f"Temizlik Noktasi {i+1} gidiliyor...")
            self.go_to_goal(point['x'], point['y'], point['z'], point['w'])
            rospy.sleep(1)

    def save_final_report(self):
        rospy.loginfo("\n=== GOREV SONU RAPORU ===")
        with open(self.report_file, "w") as f:
            f.write("KTUN ROBOTIK FINAL - TEMIZLIK RAPORU\n")
            f.write("------------------------------------\n")
            for room, status in self.report.items():
                line = f"ODA: {room.upper()} \t -> {status}\n"
                f.write(line)
                rospy.loginfo(line.strip())
            f.write("------------------------------------\n")

    def run(self):
        mission_data = self.load_mission()
        if not mission_data: return

        rooms = ["salon", "mutfak", "koridor", "yatak_odasi"]

        for room_name in rooms:
            if room_name not in mission_data: continue

            room_data = mission_data[room_name]
            entry = room_data['entry']
            qr_target = room_data['qr_text']
            
            rospy.loginfo(f"### GOREV: {room_name.upper()} ###")

            # 1. Odaya Git
            if not self.go_to_goal(entry['x'], entry['y'], entry['z'], entry['w']):
                rospy.logerr(f"{room_name} ulaşılamadı!")
                self.report[room_name] = "FAIL (Gidilemedi)"
                continue

            # 2. QR Kontrol (AKILLI SENARYO BURADA)
            rospy.loginfo(f"{room_name} QR kontrol ediliyor...")
            self.last_qr_data = None # Hafızayı temizle
            
            # --- YENİ FONKSİYONU ÇAĞIRIYORUZ ---
            qr_found = self.smart_recovery_and_check(qr_target)
            # -----------------------------------

            if qr_found:
                rospy.loginfo(f"QR ONAYLANDI: {room_name}")
                if 'cleaning_points' in room_data:
                    self.perform_cleaning_tour(room_data['cleaning_points'], room_name)
                    self.report[room_name] = "SUCCESS"
                else:
                    self.report[room_name] = "SUCCESS (Nokta Yok)"
            else:
                rospy.logwarn(f"HATA: {room_name} QR bulunamadi (Tüm denemeler başarisiz).")
                self.report[room_name] = "SKIPPED (QR Yok)"
            
            rospy.sleep(1)

        self.save_final_report()

if __name__ == '__main__':
    try:
        manager = TaskManager()
        manager.run()
    except rospy.ROSInterruptException:
        pass
