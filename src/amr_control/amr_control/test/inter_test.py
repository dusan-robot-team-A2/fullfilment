import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Vector3
from fullfilment_interfaces.action import Move2D, Rotate2D

class ActionClientNode(Node):
    def __init__(self):
        super().__init__('action_client_node')
        
        # Move2D 액션 서버 연결
        self._move2d_action_client = ActionClient(self, Move2D, 'nav2_goal')
        
        # Rotate2D 액션 서버 연결
        self._rotate2d_action_client = ActionClient(self, Rotate2D, 'feedback_position')

    def send_move2d_goal(self, position, orientation):
        # Move2D Goal 메시지 준비
        goal_msg = Move2D.Goal()
        goal_msg.position = Vector3(x=position[0], y=position[1], z=position[2])
        goal_msg.orientation = Vector3(x=orientation[0], y=orientation[1], z=orientation[2])

        self.get_logger().info(f"Sending Move2D goal: Position: {goal_msg.position}, Orientation: {goal_msg.orientation}")
        
        # 서버가 준비되었는지 확인하고 액션을 전송
        self._move2d_action_client.wait_for_server()
        send_goal_future = self._move2d_action_client.send_goal_async(goal_msg, feedback_callback=self.move2d_feedback_callback)
        send_goal_future.add_done_callback(self.move2d_goal_response_callback)

    def move2d_goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Move2D goal was rejected.')
            return

        self.get_logger().info('Move2D goal accepted, waiting for result...')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.move2d_result_callback)

    def move2d_result_callback(self, future):
        result = future.result().result
        if result.success:
            self.get_logger().info(f'Move2D Success: {result.message}')
            # Move2D가 성공적으로 끝난 후 Rotate2D 발행
            self.send_rotate2d_goal(Vector3(x=0.0, y=1.0, z=0.0))  # 예시 방향
        else:
            self.get_logger().info(f'Move2D Failed: {result.message}')

    def move2d_feedback_callback(self, feedback_msg):
        # Move2D 피드백 처리
        self.get_logger().info(f'Move2D Feedback: {feedback_msg.feedback}')

    def send_rotate2d_goal(self, orientation):
        # Rotate2D Goal 메시지 준비
        goal_msg = Rotate2D.Goal()
        goal_msg.orientation = orientation

        self.get_logger().info(f"Sending Rotate2D goal: Orientation: {goal_msg.orientation}")
        
        # 서버가 준비되었는지 확인하고 액션을 전송
        self._rotate2d_action_client.wait_for_server()
        send_goal_future = self._rotate2d_action_client.send_goal_async(goal_msg, feedback_callback=self.rotate2d_feedback_callback)
        send_goal_future.add_done_callback(self.rotate2d_goal_response_callback)

    def rotate2d_goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Rotate2D goal was rejected.')
            return

        self.get_logger().info('Rotate2D goal accepted, waiting for result...')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.rotate2d_result_callback)

    def rotate2d_result_callback(self, future):
        result = future.result().result
        if result.success:
            self.get_logger().info(f'Rotate2D Success: {result.message}')
        else:
            self.get_logger().info(f'Rotate2D Failed: {result.message}')

    def rotate2d_feedback_callback(self, feedback_msg):
        # Rotate2D 피드백 처리
        self.get_logger().info(f'Rotate2D Feedback: {feedback_msg.feedback}')

def main(args=None):
    rclpy.init(args=args)
    action_client_node = ActionClientNode()

    # 목표 위치와 방향 설정
    position = [0.05,0.73, 0.0]  # 예시: 목표 위치 (x, y, z)
    orientation = [1.0, 1.0, 0.0]  # 예시: 2D 방향 (단위 벡터 a, b, 0)

    # Move2D 목표 전송
    action_client_node.send_move2d_goal(position, orientation)

    rclpy.spin(action_client_node)
    action_client_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
