import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from fullfilment_interfaces.action import Move2D, Rotate2D
from geometry_msgs.msg import Vector3

class Nav2GoalClient(Node):
    def __init__(self):
        super().__init__('nav2goal_client')
        self._client = ActionClient(self, Move2D, 'nav2_goal')
        self._rotate_client = ActionClient(self, Rotate2D, 'feedback_position')  # Rotate2D 액션 클라이언트 추가

    def send_goal(self, position, orientation):
        # Move2D Goal 메시지 준비
        goal_msg = Move2D.Goal()
        # position을 Vector3로 설정
        goal_msg.position = Vector3(x=position[0], y=position[1], z=position[2])  # x, y, z 좌표
        # orientation을 Vector3로 설정
        goal_msg.orientation = Vector3(x=orientation[0], y=orientation[1], z=orientation[2])  # 방향 단위 벡터 (a, b, 0)

        # 목표 전송
        self.get_logger().info(f"Sending goal: Position: {goal_msg.position}, Orientation: {goal_msg.orientation}")
        send_goal_future = self._client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)

        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal was rejected.')
            return

        self.get_logger().info('Goal accepted, waiting for result...')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def feedback_callback(self, feedback):
        # 피드백 처리
        self.get_logger().info(f"Feedback: {feedback.feedback}")

    def result_callback(self, future):
        # 결과 처리
        result = future.result().result
        if result.success:
            self.get_logger().info(f"Success: {result.message}")
            self.send_rotate_goal(Vector3(x=0.707, y=0.707, z=0.0))  # 예시로 회전 목표 추가
        else:
            self.get_logger().info(f"Failed: {result.message}")

    def send_rotate_goal(self, orientation):
        goal_msg = Rotate2D.Goal()
        goal_msg.orientation = orientation
        self._rotate_client.send_goal_async(goal_msg, feedback_callback=self.rotate_feedback_callback)

    def rotate_feedback_callback(self, feedback):
        self.get_logger().info(f"Rotation Feedback: {feedback}")

def main(args=None):
    rclpy.init(args=args)
    client = Nav2GoalClient()

    # 목표 위치와 방향 설정
    position = [1.0, 2.0, 0.0]  # 예시: 목표 위치 (x, y, z)
    orientation = [0.707, 0.707, 0.0]  # 예시: 2D 방향 (단위 벡터 a, b, 0)

    # 목표 전송
    client.send_goal(position, orientation)

    rclpy.spin(client)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
