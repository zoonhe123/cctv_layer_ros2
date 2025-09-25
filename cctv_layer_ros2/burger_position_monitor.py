import rclpy
from rclpy.node import Node
from gazebo_msgs.msg import ModelStates # ModelStates 메시지 타입 임포트

class BurgerPositionMonitor(Node):
    def __init__(self):
        super().__init__('burger_position_monitor')
        self.get_logger().info('BurgerPositionMonitor node has been started. Monitoring burger position.')

        # /model_states 토픽 구독 설정
        self.subscription = self.create_subscription(
            ModelStates,
            '/model_states',
            self.model_states_callback,
            10  # QoS 큐 사이즈
        )
        self.subscription  # Prevent unused variable warning

    def model_states_callback(self, msg: ModelStates):
        """
        /model_states 메시지가 수신될 때마다 호출되는 콜백 함수.
        메시지에서 'burger' 모델의 위치를 찾아 출력합니다.
        """
        found_burger = False
        for i, model_name in enumerate(msg.name):
            if model_name == 'burger':
                position = msg.pose[i].position
                self.get_logger().info(
                    f"Burger Position: "
                    f"x={position.x:.3f}, "
                    f"y={position.y:.3f}, "

                )
                found_burger = True
                break # burger를 찾았으면 더 이상 다른 모델을 검색할 필요 없음

        if not found_burger:
            # burger 모델이 현재 메시지에 포함되지 않은 경우 (선택 사항)
            # self.get_logger().debug("Burger model not found in current ModelStates message.")
            pass


def main(args=None):
    rclpy.init(args=args) # ROS 2 통신 초기화

    burger_position_monitor = BurgerPositionMonitor() # 노드 인스턴스 생성

    try:
        rclpy.spin(burger_position_monitor) # 노드를 계속 실행하며 콜백 대기
    except KeyboardInterrupt:
        burger_position_monitor.get_logger().info('BurgerPositionMonitor node stopped by user.')
    finally:
        burger_position_monitor.destroy_node() # 노드 소멸
        rclpy.shutdown() # ROS 2 통신 종료

if __name__ == '__main__':
    main()