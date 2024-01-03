import rclpy
import torch
import numpy as np
import cv2
import os

from ultralytics import YOLO
from supervision import Detections, BoxAnnotator
from rclpy.node import Node
from std_msgs.msg import String, Int8MultiArray, Int8, Bool
from rclpy import qos
from cv_bridge import CvBridge

class bucket_model(Node):
    def __init__(self):
        super().__init__("bucket_model_node")
        self.sent_bucket_detect = self.create_publisher(
            Int8MultiArray, "bucket/detect", qos_profile=qos.qos_profile_system_default
        )
        self.sent_center_detect = self.create_publisher(
            Bool, "bucket/detect_center", qos_profile=qos.qos_profile_system_default
        )
        self.sent_timer = self.create_timer(0.05, self.timer_callback)

        self.sub_state = self.create_subscription(
            Int8,
            "state/main_ros",
            self.sub_state_mainros_callback,
            qos_profile=qos.qos_profile_sensor_data,
        )
        self.sub_state

        self.color_array = [0,0,0,0,0,0,0,0,0]
        self.round = 0
        self.check = False
        self.center = False
        
        self.cap = cv2.VideoCapture("/dev/video2")
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        self.device = "cuda" if torch.cuda.is_available() else "cpu"
        self.path = os.path.join(
            os.path.expanduser("~"),
            "ros_ws",
            "install",
            "jinnode",
            "share",
            "jinnode",
            "weights",
        )
        self.frame = np.zeros((480, 640, 3), dtype=np.uint8)
        self.model = self.load_model("best.pt")
        self.CLASS_NAMES_DICT = self.model.model.names
        self.box_annotator = BoxAnnotator(thickness=3, text_thickness=1, text_scale=0.5)

        self.mainros_state = -1

    def load_model(self, file):
        model = YOLO(f"{self.path}/{file}")
        model.fuse()
        return model

    def predict(self, frame):
        results = self.model(frame)
        return results
        
    def plot_bboxes(self, results, frame):
        for result in results[0]:
            if result.boxes.conf.cpu().numpy() >= 0.4:
                detections = Detections(
                    xyxy=result.boxes.xyxy.cpu().numpy(),
                    confidence=result.boxes.conf.cpu().numpy(),
                    class_id=result.boxes.cls.cpu().numpy().astype(int),
                )
                self.labels = [
                    f"{self.CLASS_NAMES_DICT[class_id]}"
                    for confidence, class_id in zip(
                        detections.confidence, detections.class_id
                    )
                ]
                frame = self.box_annotator.annotate(
                    scene=frame, detections=detections, labels=self.labels
                ) 
                    
                for xyxys,classid in zip (detections.xyxy, detections.class_id):
                    cv2.circle(frame,(round((xyxys[2] + xyxys[0]) / 2),
                        round((xyxys[3] + xyxys[1]) / 2),),5,(255, 0, 0),
                        -1,
                    )
                    if detections.confidence[0] > 0.75 and not self.check:
                        if (round((xyxys[2] + xyxys[0]) / 2))>=205 and (round((xyxys[2] + xyxys[0]) / 2)) <= 520: # 1st x
                            if round((xyxys[3] + xyxys[1]) / 2)>(0) and round((xyxys[3] + xyxys[1]) / 2)<=(235):
                                self.color_array[0] = int(classid)+1
                            if round((xyxys[3] + xyxys[1]) / 2)>(235) and round((xyxys[3] + xyxys[1]) / 2)<=(410):
                                self.color_array[1] = int(classid)+1
                            if round((xyxys[3] + xyxys[1]) / 2)>(410) and round((xyxys[3] + xyxys[1]) / 2)<=(605): 
                                self.color_array[2] = int(classid)+1
                        if (round((xyxys[2] + xyxys[0]) / 2))>520 and (round((xyxys[2] + xyxys[0]) / 2)) <= 815: # 2nd x
                            if round((xyxys[3] + xyxys[1]) / 2)>(0) and round((xyxys[3] + xyxys[1]) / 2)<=(235):
                                self.color_array[3] = int(classid)+1
                            if round((xyxys[3] + xyxys[1]) / 2)>(235) and round((xyxys[3] + xyxys[1]) / 2)<=(410):
                                self.color_array[4] = int(classid)+1
                            if round((xyxys[3] + xyxys[1]) / 2)>(410) and round((xyxys[3] + xyxys[1]) / 2)<=(605): 
                                self.color_array[5] = int(classid)+1
                        if (round((xyxys[2] + xyxys[0]) / 2))>815 and (round((xyxys[2] + xyxys[0]) / 2)) <= 1085: # 3rd x
                            if round((xyxys[3] + xyxys[1]) / 2)>(0) and round((xyxys[3] + xyxys[1]) / 2)<=(235):
                                self.color_array[6] = int(classid)+1
                            if round((xyxys[3] + xyxys[1]) / 2)>(235) and round((xyxys[3] + xyxys[1]) / 2)<=(410):
                                self.color_array[7] = int(classid)+1
                            if round((xyxys[3] + xyxys[1]) / 2)>(410) and round((xyxys[3] + xyxys[1]) / 2)<=(605): 
                                self.color_array[8] = int(classid)+1
                    
                    self.round +=1
                    if self.mainros_state == 2 or self.mainros_state == 4 or self.mainros_state == 6:
                        if (round((xyxys[2] + xyxys[0]) / 2))>=350 and (round((xyxys[2] + xyxys[0]) / 2)) <= 950:
                            if round((xyxys[3] + xyxys[1]) / 2) > 0 and round((xyxys[3] + xyxys[1]) / 2) <= 410:
                                self.center = True
                            else: pass

            if self.round == 9:
                self.check = True

        return frame
        
    def timer_callback(self):
        msg_detect = Int8MultiArray() 
        msg_center = Bool()       
        ref, frame = self.cap.read()
        if self.mainros_state == 0:
            if not self.check:
                results = self.predict(frame)
                self.frame = self.plot_bboxes(results, frame)
                cv2.line(frame,(250,0),(250,605),(0, 0, 0), 4) # x limit
                cv2.line(frame,(520,0),(520,605),(0, 0, 0), 4) # x limit
                cv2.line(frame,(815,0),(815,605),(0, 0, 0), 4) # x limit
                cv2.line(frame,(1085,0),(1085,605),(0, 0, 0), 4) # x limit
                cv2.line(frame,(250,235),(1085,235),(0, 0, 0), 4) # y upper limit
                cv2.line(frame,(250,410),(1085,410),(0, 0, 0), 4) # y midden limit
                cv2.line(frame,(250,605),(1085,605),(0, 0, 0), 4) # y lower limit            
                cv2.imshow("frame", self.frame)
                
            if len(self.color_array) != 0:
                msg_detect.data = self.color_array
                self.sent_bucket_detect.publish(msg_detect)
                # cv2.destroyAllWindows()
                # exit()
                            
        if not self.center and self.mainros_state > 0:
            results = self.predict(frame)
            self.frame = self.plot_bboxes(results, frame)
            cv2.line(frame,(350,0),(350,410),(0, 0, 0), 4) # x limit
            cv2.line(frame,(950,0),(950,410),(0, 0, 0), 4) # x limit
            cv2.line(frame,(350,410),(950,410),(0, 0, 0), 4) # y upper limit
            cv2.imshow("frame", self.frame)
            if self.center:
                msg_center.data = self.center
                self.sent_center_detect.publish(msg_center)
                self.center = False
            # cv2.destroyAllWindows()
            # exit()
        

        if not ref:
            cv2.destroyAllWindows()
            exit()

        if cv2.waitKey(1) & 0xFF == ord("q"):
            cv2.destroyAllWindows()
            exit()

    def sub_state_mainros_callback(self, msg_in):
        self.mainros_state = msg_in.data

def main():
    rclpy.init()
    sub = bucket_model()
    rclpy.spin(sub)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
