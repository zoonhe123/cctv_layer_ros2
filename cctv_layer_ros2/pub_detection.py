import rclpy
from rclpy.node import Node
from gazebo_msgs.msg import ModelStates
import re
import math
import time
import numpy as np
from cctv_layer_msgs.msg import DetectionArray
from cctv_layer_msgs.msg import Detection
from geometry_msgs.msg import Pose, Vector3

class PubDetection(Node):
    def __init__(self):
        super().__init__('pub_detection')

        self.subscriber = self.create_subscription(
            ModelStates,
            '/model_states',
            self.model_states_callback,
            10
        )

        self.publisher = self.create_publisher(
            DetectionArray,
            '/detections',
            10
        )

        # 이전 위치 저장용 딕셔너리: { 'walking_human_1': (x, y, t) }
        self.prev_positions = {}
        self.time_prev = None

    def model_states_callback(self, msg: ModelStates):

        # /model_states 의 평균 hz 는 27.7~8 정도 (0.033 ~ 0.038s)
        #====================== dt 계산 ======================== 
        current_time = self.get_clock().now().nanoseconds
        detections = []

        if self.time_prev == None:
            self.time_prev = current_time

            for i, name in enumerate(msg.name):
                if name.startswith('walking_human'):
                    x = msg.pose[i].position.x
                    y = msg.pose[i].position.y
                    self.prev_positions[name] = (x, y)
            return

        dt = (current_time - self.time_prev) / 1e9
        self.time_prev = current_time

        for i, name in enumerate(msg.name):
            if name.startswith('walking_human'):

                prev_x, prev_y = self.prev_positions[name]
                x = msg.pose[i].position.x
                y = msg.pose[i].position.y

                vx = (x - prev_x) / dt
                vy = (y - prev_y) / dt

                velocity = np.sqrt(vx **2 + vy **2)
                print('{} vel:{}'.format(name,velocity))

                self.prev_positions[name] = (x, y)

                # 여기부터 /detection msg pub 코드
                match = re.search(r'\d+', name)
                tracking_id = match.group() if match else '0'

                # Detection 메시지 구성
                detection_msg = Detection()
                detection_msg.class_id = 0
                detection_msg.class_name = 'human'
                detection_msg.score = 1.0
                detection_msg.id = tracking_id

                pose_msg = Pose()
                pose_msg.position.x = x
                pose_msg.position.y = y
                detection_msg.pose = pose_msg

                vel_msg = Vector3()
                vel_msg.x = vx
                vel_msg.y = vy
                detection_msg.vel = vel_msg

                detections.append(detection_msg)

        # DetectionArray 구성 및 퍼블리시
        detection_array_msg = DetectionArray()
        detection_array_msg.detections = detections
        self.publisher.publish(detection_array_msg)

 
        


def main(args=None):
    rclpy.init(args=args)
    pub_detection = PubDetection()
    rclpy.spin(pub_detection)
    pub_detection.destroy_node()
    rclpy.shutdown()
