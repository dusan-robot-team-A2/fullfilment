import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import Vector3
from fullfilment_interfaces.action import Rotate2D

class Rotate2DActionClient(Node):
    def __init__(self):
        super().__init__('rotate2d_action_client')
        # 'feedback_position' 액션 서버와의 연결을 위한 액션 클라이언트 생성
        self._action_client = ActionClient(self, Rotate2D, 'feedback_position')

    def send_goal(self, orientation):
        # 목표 생성 (Goal 메시지 설정)
        goal_msg = Rotate2D.Goal()
        goal_msg.orientation = orientation

        # 목표를 서버로 전송
        self._action_client.wait_for_server()  # 서버 대기
        self.get_logger().info('Sending goal request...')
        send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)

        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal was rejected.')
            return

        self.get_logger().info('Goal accepted, waiting for result...')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def feedback_callback(self, feedback_msg):
        # 피드백 처리
        self.get_logger().info(f'Feedback: {feedback_msg.feedback}')

    def result_callback(self, future):
        # 결과 처리
        result = future.result().result
        if result.success:
            self.get_logger().info(f'Success: {result.message}')
        else:
            self.get_logger().info(f'Failed: {result.message}')

def main(args=None):
    rclpy.init(args=args)
    action_client = Rotate2DActionClient()
    orientation = Vector3(x=0.707, y=0.707, z=0.0)  # 예시 방향 (단위 벡터)
    action_client.send_goal(orientation)
    rclpy.spin(action_client)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
