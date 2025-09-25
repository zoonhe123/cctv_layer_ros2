import rclpy
from rclpy.node import Node
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.srv import SetEntityState
from gazebo_msgs.msg import EntityState
from geometry_msgs.msg import Pose, Twist

import re

class MoveObstacle(Node):
    def __init__(self):
        super().__init__('move_obstacle')

        # 모델 위치 저장용 딕셔너리
        self.model_poses = {}

        # /model_states 구독
        self.subscriber = self.create_subscription(
            ModelStates,
            '/model_states',
            self.model_states_callback,
            10
        )

        # 서비스 클라이언트
        self.cli = self.create_client(SetEntityState, '/set_entity_state')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /set_entity_state service...')



    def model_states_callback(self, msg):
        for i, name in enumerate(msg.name):


            if name.startswith('walking_human'):

                match = re.search(r'\d+', name)
                if match:
                    idx = match.group()  # 예: '1'
                    obstacle_name = f'unit_obstacle_{idx}'

                req = SetEntityState.Request()
                req.state.name = obstacle_name
                req.state.pose.position.x = msg.pose[i].position.x
                req.state.pose.position.y = msg.pose[i].position.y

                future = self.cli.call_async(req)



def main(args=None):
    rclpy.init(args=args)
    move_obstacle = MoveObstacle()
    rclpy.spin(move_obstacle)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

