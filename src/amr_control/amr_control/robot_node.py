# turtlebot_controller.py
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Twist
from fullfilment_interfaces.action import Move2D, Rotate2D
from fullfilment_interfaces.srv import MoveToBasket
from robot import Robot

class TurtleBotController(Node):
    def __init__(self):
        super().__init__('turtlebot_controller')
        # create ROS communication
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10) # 로봇 이동명령 pub
        self.nav2goal_action = ActionServer(self, Move2D, 'nav2_goal', self.nav2goal_callback) # 로봇 목표 위치 및 초기방향
        self.pose_feedback_action = ActionServer(self, Rotate2D, 'feedback_position', self.feedback_pose) # 로봇 목표 방향
        self.nav2_basket_srv = self.create_service(MoveToBasket, 'nav2_basket', self.nav2_basket)

        self.robot = Robot(self)

    def nav2goal_callback(self, goal_handle):   # msg 구성: 1. 터틀봇의 초기 방향, 2. goal 좌표와 터틀봇의 상대좌표  (글로벌 좌표계 기준)
        self.get_logger().info(f"Received goal request: 2D position: {goal_handle.request.position}, 2D orientation: {goal_handle.request.orientation}")
        self.start_rotation_and_move(goal_handle.request)
        goal_handle.succeed()
        result = Move2D.Result()
        result.success = True
        result.message = "Arrive at the target point"
        self.get_logger().info(result.message)
        return result
    
    def start_rotation_and_move(self, data):
        self.robot.rotation(self.robot.get_velocity(data), data)
        self.get_logger().info("initpose를 맞쳤습니다.")
        self.robot.move(self.robot.get_velocity(data))

    def feedback_pose(self,goal_handle):
        data = goal_handle.request  # 피드백 데이터
        self.robot.rotation(self.robot.calculate_angle(data), data)
        goal_handle.succeed()
        result = Rotate2D.Result()
        result.success = True
        result.message = "Updated robot pose"
        self.get_logger().info(result.message)
        self.get_logger().info("아루코 바라보기 결과를 발행합니다.")
        return result
    
    def nav2_basket(self, request, response):
        self.robot.back()
        response.success = True
        response.message = "바구니 픽업 장소에 도착했습니다."
        self.get_logger().info(response.message)
        self.get_logger().info("바구니 픽업장소 이동 결과를 발행합니다.")
        return response
    
def main(args=None):
    rclpy.init(args=args)
    node = TurtleBotController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
