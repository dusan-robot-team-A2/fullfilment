import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from fullfilment_interfaces.action import Move2D
from geometry_msgs.msg import Vector3

class Nav2GoalClient(Node):
    def __init__(self):
        super().__init__('nav2goal_client')
        self._client = ActionClient(self, Move2D, 'nav2_goal')

    def send_goal(self, position, orientation):
        # Move2D Goal 메시지 준비
        goal_msg = Move2D.Goal()
        # position을 Vector3로 설정
        goal_msg.position = Vector3(x=position[0], y=position[1], z=position[2])  # x, y, z 좌표
        # orientation을 Vector3로 설정
        goal_msg.orientation = Vector3(x=orientation[0], y=orientation[1], z=orientation[2])  # 방향 단위 벡터 (a, b, 0)

        # 목표 전송
        self.get_logger().info(f"Sending goal: Position: {goal_msg.position}, Orientation: {goal_msg.orientation}")
        self._client.send_goal(goal_msg, feedback_callback=self.feedback_callback)

    def feedback_callback(self, feedback):
        # 피드백을 출력
        self.get_logger().info(f"Feedback: {feedback}")

def main(args=None):
    rclpy.init(args=args)
    client = Nav2GoalClient()

    # 목표 위치와 방향 설정
    position = [1.0, 2.0, 0.0]  # 예시: 목표 위치 (x, y, z)
    orientation = [0.707, 0.707, 0.0]  # 예시: 2D 방향 (단위 벡터 a, b, 0)

    # 목표 전송
    client.send_goal(position, orientation)

    rclpy.spin(client)

if __name__ == '__main__':
    main()
