#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os, json, time
import cv2
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String, Int32
from ultralytics import YOLO
import torch

# YOLOv8 50 epoch 학습 모델 Path
# MODEL_PATH = os.path.join(
#     os.path.dirname(os.path.abspath(__file__)),
#     "train_e50_lr0", "weights", "best_e50_lr0.pt"
# )

# YOLOv8 500 epoch 학습 모델 Path
MODEL_PATH = os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    "train_e500_lr0", "weights", "best_e500.pt"
)

CLASSNAMES = ["left", "right"]
CONF_TH    = 0.5
IOU_TH     = 0.45
SHOW       = True        # OpenCV preview on subscriber side
WINDOW_SEC = 3.0         # Voting window seconds
ONE_SHOT   = True        # Publish one result then exit

class YoloNode:
    def __init__(self):
        # Model / bridge / pubs-subs
        self.model  = YOLO(MODEL_PATH)
        self.bridge = CvBridge()

        # Keep topic names fixed (no remapping/params)
        self.pub_json   = rospy.Publisher("/yolo_topic",  String, queue_size=10)
        self.pub_viz    = rospy.Publisher("/yolo/image",  Image,  queue_size=1)
        # Latch so late subscribers can still get the last result
        self.pub_result = rospy.Publisher("/yolo_result", Int32,  queue_size=1, latch=True)

        self.sub_img = rospy.Subscriber("/camera/image_raw", Image, self.on_image,
                                        queue_size=1, buff_size=2**24)

        device = "cuda" if torch.cuda.is_available() else "cpu"
        rospy.loginfo(f"[yolo] model={os.path.basename(MODEL_PATH)} on {device} classes={CLASSNAMES}")

        # Voting window state
        self.window_start = None
        self.left_count   = 0
        self.right_count  = 0
        self.decided      = False

    def _cls_name(self, cls_id: int):
        if 0 <= cls_id < len(CLASSNAMES):
            return CLASSNAMES[cls_id]
        names = getattr(self.model, "names", {})
        return names.get(cls_id, f"cls_{cls_id}")

    def _update_vote(self, detections):
        # Count only the most confident left/right detection in this frame
        best = None  # (label, conf)
        for det in detections:
            cls_name = det["cls"]
            conf     = det["conf"]
            label = None
            if str(cls_name) in ("left", "0"):
                label = "left"
            elif str(cls_name) in ("right", "1"):
                label = "right"
            if label is None:
                continue
            if (best is None) or (conf > best[1]):
                best = (label, conf)

        if best is None:
            return
        if best[0] == "left":
            self.left_count += 1       # change to += best[1] to use conf weighting
        else:
            self.right_count += 1

    def _maybe_decide(self):
        if self.window_start is None:
            return
        elapsed = time.time() - self.window_start
        if elapsed < WINDOW_SEC:
            return

        # Majority
        if self.left_count > self.right_count:
            result = 0
        elif self.right_count > self.left_count:
            result = 1
        else:
            result = -1

        rospy.loginfo(f"[yolo] window {WINDOW_SEC:.1f}s => {result} "
                      f"(left={self.left_count}, right={self.right_count})")
        self.pub_result.publish(Int32(data=result))
        self.decided = True

        if ONE_SHOT:
            rospy.signal_shutdown("yolo_result published (one-shot)")
        else:
            # Reset the window
            self.window_start = time.time()
            self.left_count = 0
            self.right_count = 0

    def on_image(self, msg: Image):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        if self.window_start is None:
            self.window_start = time.time()

        # Inference
        t0 = time.time()
        results = self.model(frame, conf=CONF_TH, iou=IOU_TH, verbose=False)
        t1 = time.time()

        detections = []
        vis = frame.copy()

        for r in results:
            for box in r.boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                conf = float(box.conf[0])
                cls_id = int(box.cls[0]) if box.cls is not None else -1
                cls_name = self._cls_name(cls_id)

                detections.append({
                    "cls": cls_name,
                    "conf": round(conf, 4),
                    "bbox": [x1, y1, x2, y2]
                })

                # Visualization colors
                if str(cls_name) in ["left", "0"]:
                    color = (255, 0, 0)
                elif str(cls_name) in ["right", "1"]:
                    color = (0, 0, 255)
                else:
                    color = (128, 128, 128)

                cv2.rectangle(vis, (x1, y1), (x2, y2), color, 2)
                label = f"{cls_name}: {conf:.2f}"
                (w, h), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.7, 2)
                cv2.rectangle(vis, (x1, y1 - h - 10), (x1 + w, y1), color, -1)
                cv2.putText(vis, label, (x1, y1 - 5),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255), 2)

        # JSON topic (optional)
        out = {
            "stamp": rospy.Time.now().to_sec(),
            "detections": detections,
            "infer_ms": int((t1 - t0) * 1000)
        }
        self.pub_json.publish(String(data=json.dumps(out)))

        # Visualization topic
        self.pub_viz.publish(self.bridge.cv2_to_imgmsg(vis, encoding="bgr8"))

        # Voting
        self._update_vote(detections)
        if not self.decided:
            self._maybe_decide()

        # Local preview
        if SHOW:
            cv2.imshow("YOLO Detection (subscriber)", vis)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                rospy.signal_shutdown("User requested quit")

    def close(self):
        cv2.destroyAllWindows()

def main():
    rospy.init_node("vision_sub_node", anonymous=True)
    node = YoloNode()
    rospy.loginfo("vision_sub_node started (hardcoded config)")
    try:
        rospy.spin()
    finally:
        node.close()
        rospy.loginfo("vision_sub_node stopped")

if __name__ == "__main__":
    main()
