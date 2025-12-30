#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String
from pyzbar.pyzbar import decode

class QRReader:
    def __init__(self):
        rospy.init_node('qr_reader_node', anonymous=True)
        
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.image_callback)
        self.qr_pub = rospy.Publisher("/qr_code_read", String, queue_size=10)
        
        self.bridge = CvBridge()
        
        # --- HAFIZA KISMI ---
        # En son okuduğumuz QR'ı burada tutacağız
        self.last_qr_data = "" 
        
        rospy.loginfo("QR Okuyucu Baslatildi")

    def image_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            decoded_objects = decode(cv_image)
            
            for obj in decoded_objects:
                qr_data = obj.data.decode("utf-8")
                
                # --- AKILLI KONTROL ---
                # Eğer okunan QR, bir öncekiyle AYNI DEĞİLSE ekrana yaz.
                # Aynıysa sessizce sadece sisteme bildir (Publish et).
                if qr_data != self.last_qr_data:
                    rospy.loginfo(f"QR OKUNDU: {qr_data}")
                    self.last_qr_data = qr_data # Hafızayı güncelle
                
                # Arka planda Task Manager duysun diye HER ZAMAN yayınla
                # (Ama ekrana print basma)
                self.qr_pub.publish(qr_data)

        except Exception as e:
            # Hata mesajlarını da spam yapmasın diye logdebug kullanabiliriz ama şimdilik kalsın
            pass

if __name__ == '__main__':
    try:
        qr = QRReader()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
