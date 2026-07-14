import cv2
from ultralytics import YOLO

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from vision_msgs.msg import TargetCoord

class YOLO_Node(Node):
    def __init__(self):
        super().__init__('yolo_node')

        offboard_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.vis_pub = self.create_publisher(TargetCoord, 'vision/custom_target', offboard_qos)
        self.model = YOLO("/home/proxima/Chapt/chapt_ws/yolov8n.pt")
        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH,640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT,480)
        self.hit_point = None

    def vision_pb(self,x,y,con,id):
        msg = TargetCoord()
        msg.p_x = x
        msg.p_y = y
        msg.conf = con
        msg.class_id = id
        self.vis_pub.publish(msg)
        
    def to_point(self):
    
        if not self.cap.isOpened():
            print("打开失败")
            return
    
        while True:
            ret, frame = self.cap.read()
            results = self.model(frame,conf=0.5)
            for res in results:
                boxes = res.boxes
                if boxes is not None:
                    for box in boxes:
                        x1,y1,x2,y2 = map(int,box.xyxy[0])
                        conf = float(box.conf[0])
                        cls_id = int(box.cls[0])
                        cls_name = self.model.names[cls_id]
                        self.vision_pb((x1+x2)/2,(y1+y2)/2,conf,cls_id)
                        print((x1+x2)/2,(y1+y2)/2,conf,cls_id)
                        cv2.rectangle(frame,(x1,y1),(x2,y2),(0.255,0),2)
                        text = f"{cls_name}{conf:.2f}"
                        cv2.putText(frame,text,(x1,y1 - 10),cv2.FONT_HERSHEY_SIMPLEX,0.5,(0,255,0),2)
        
            cv2.imshow('YOLOv8n', frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        self.cap.release()
        cv2.destroyAllWindows()

def main(args=None):
    rclpy.init(args=args)
    yolo_node = YOLO_Node()
    yolo_node.to_point()
    rclpy.spin(yolo_node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()