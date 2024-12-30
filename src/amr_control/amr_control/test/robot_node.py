# turtlebot_controller.py
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from geometry_msgs.msg import Twist
from fullfilment_interfaces.action import Move2D, Rotate2D
from fullfilment_interfaces.srv import MoveBasket
from robot import Robot

class TurtleBotController(Node):
    def __init__(self):
        super().__init__('turtlebot_controller')
        # create ROS communication
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10) # 로봇 이동명령 pub
        self.nav2goal_action = ActionServer(self, Move2D, 'nav2_goal', self.nav2goal_callback) # 로봇 목표 위치 및 초기방향
        self.pose_feedback_action = ActionServer(self, Rotate2D, 'feedback_position', self.feedback_pose) # 로봇 목표 방향
        self.basket_client = self.create_client(MoveBasket, 'move_basket')

        self.robot = Robot(self)

    def nav2goal_callback(self, goal_handle):   # msg 구성: 1. 터틀봇의 초기 방향, 2. goal 좌표와 터틀봇의 상대좌표  (글로벌 좌표계 기준)
        self.get_logger().info(f"Received goal request: 2D position: {goal_handle.request.position}, 2D orientation: {goal_handle.request.orientation}")
        self.start_rotation_and_move(goal_handle.request)
        goal_handle.succeed()
        result = Move2D.Result()
        result.success = True
        result.message = "Arrive at the target point"
        self.get_logger().info(result.message)
        self.destroy_node()
        return result
    
    def start_rotation_and_move(self, data):
        self.robot.rotation(self.robot.get_velocity(data), data)
        print("initpose를 맞쳤습니다.")
        self.robot.move(self.robot.get_velocity(data))

    def feedback_pose(self,goal_handle):
        data = goal_handle.request  # 피드백 데이터
        self.robot.rotation(self.robot.calculate_angle(data), data)
        message = "아루코 마커를 바라보고 있습니다."
        self.get_logger().info(message)

        self.handle_service_and_result(goal_handle)

        return None
    
    def handle_service_and_result(self, goal_handle):
        goal_handle.succeed()
        result = Rotate2D.Result()
        result.success = True
        result.message = "Updated robot pose"
        self.get_logger().info(result.message)
        print("아루코 바라보기 결과를 발행합니다.")
        print("바스켓을 집도록 합니다.")
        self.send_service_request()
    
    def send_service_request(self):
        if not self.check_server():
            return
        request = MoveBasket.Request()
        request.message = 'Move to Basket'

        future = self.basket_client.call_async(request)
        future.add_done_callback(self.service_response_callback)

    def service_response_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info(f'Service response: {response}')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')

    
    def check_server(self):
        if not self.basket_client.service_is_ready():
            self.get_logger().info('Basket service is not ready.')
            return False
        return True
    
def main(args=None):
    rclpy.init(args=args)
    node = TurtleBotController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
