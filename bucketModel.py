import torch
import numpy as np
import cv2
import os
from ultralytics import YOLO

from supervision import Detections, BoxAnnotator


class ObjectDetection:
    def __init__(self, capture_index):
        self.captur_index = capture_index

        self.device = "cuda" if torch.cuda.is_available() else "cpu"
        print("Using Device: ", self.device)

        self.cap = cv2.VideoCapture("/dev/video2")
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        self.device = "cuda" if torch.cuda.is_available() else "cpu"
        self.path = os.path.join(
            os.path.expanduser("~"), "ros_ws/program/weights/"
        )
        self.model = self.load_model("best.pt")
        self.CLASS_NAMES_DICT = self.model.model.names
        self.box_annotator = BoxAnnotator(thickness=3, text_thickness=1, text_scale=0.5)
        self.color_array = [0,0,0,0,0,0,0,0,0]
        self.frame = np.zeros((480, 640, 3), dtype=np.uint8)
        self.check = False

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
                    f"{self.CLASS_NAMES_DICT[class_id]} {confidence:0.2f}"
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
                    if detections.confidence[0] > 0.75:
                        if (round((xyxys[2] + xyxys[0]) / 2))>=205 and (round((xyxys[2] + xyxys[0]) / 2)) <= 520: # 1st x
                            if round((xyxys[3] + xyxys[1]) / 2)>(0) and round((xyxys[3] + xyxys[1]) / 2)<=(235):
                                self.color_array[0] = classid+1
                            # else:
                            #     self.color_array[0] = 0
                            if round((xyxys[3] + xyxys[1]) / 2)>(235) and round((xyxys[3] + xyxys[1]) / 2)<=(410):
                                self.color_array[1] = classid+1
                            # else : 
                            #     self.color_array[1] = 0
                            if round((xyxys[3] + xyxys[1]) / 2)>(410) and round((xyxys[3] + xyxys[1]) / 2)<=(605): 
                                self.color_array[2] = classid+1
                            # else :
                            #     self.color_array[2] = 0
                        if (round((xyxys[2] + xyxys[0]) / 2))>520 and (round((xyxys[2] + xyxys[0]) / 2)) <= 815: # 2nd x
                            if round((xyxys[3] + xyxys[1]) / 2)>(0) and round((xyxys[3] + xyxys[1]) / 2)<=(235):
                                self.color_array[3] = classid+1
                            # else:
                            #     self.color_array[3] = 0
                            if round((xyxys[3] + xyxys[1]) / 2)>(235) and round((xyxys[3] + xyxys[1]) / 2)<=(410):
                                self.color_array[4] = classid+1
                            # else : 
                            #     self.color_array[4] = 0
                            if round((xyxys[3] + xyxys[1]) / 2)>(410) and round((xyxys[3] + xyxys[1]) / 2)<=(605): 
                                self.color_array[5] = classid+1
                            # else :
                            #     self.color_array[5] = 0
                        if (round((xyxys[2] + xyxys[0]) / 2))>815 and (round((xyxys[2] + xyxys[0]) / 2)) <= 1085: # 3rd x
                            if round((xyxys[3] + xyxys[1]) / 2)>(0) and round((xyxys[3] + xyxys[1]) / 2)<=(235):
                                self.color_array[6] = classid+1
                            # else:
                            #     self.color_array[6] = 0
                            if round((xyxys[3] + xyxys[1]) / 2)>(235) and round((xyxys[3] + xyxys[1]) / 2)<=(410):
                                self.color_array[7] = classid+1
                            # else : 
                            #     self.color_array[7] = 0
                            if round((xyxys[3] + xyxys[1]) / 2)>(410) and round((xyxys[3] + xyxys[1]) / 2)<=(605): 
                                self.color_array[8] = classid+1
                            # else :
                            #     self.color_array[8] = 0
                    
                    # for i in self.color_array:
                    #     if self.color_array[i] == "B":
                    #         return 0
                    #     if self.color_array[i] == "R":
                    #         return 1
                    #     if self.color_array[i] == "B":
                    #         return 2 
                    print(self.color_array)

        return frame

    def __call__(self):

        while self.cap.isOpened():
            
            ref, frame = self.cap.read()
            results = self.predict(frame)
            frame = self.plot_bboxes(results, frame)
            cv2.line(frame,(250,0),(250,605),(0, 0, 0), 4) # x limit
            cv2.line(frame,(520,0),(520,605),(0, 0, 0), 4) # x limit
            cv2.line(frame,(815,0),(815,605),(0, 0, 0), 4) # x limit
            cv2.line(frame,(1085,0),(1085,605),(0, 0, 0), 4) # x limit
            cv2.line(frame,(250,235),(1085,235),(0, 0, 0), 4) # y upper limit
            cv2.line(frame,(250,410),(1085,410),(0, 0, 0), 4) # y midden limit
            cv2.line(frame,(250,605),(1085,605),(0, 0, 0), 4) # y lower limit   

            cv2.imshow("frame", frame)

            if not ref:
                break

            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

        self.cap.release()
        cv2.destroyAllWindows()


detector = ObjectDetection(capture_index="/dev/video2")
detector()
