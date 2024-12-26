import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from std_msgs.msg import String, Bool, Int16
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from fullfilment_interfaces.action import JobAction
from std_srvs.srv import SetBool
import serial

import random, time

class SimplePublisher(Node):
    def __init__(self):
        super().__init__('simple_publisher')  # 노드 이름
        self.robot_status_pub = self.create_publisher(Int16, 'robot_status', 10)
        self.conveyor_status_pub = self.create_publisher(Bool, 'conveyor_status', 10)

        self.timer = self.create_timer(3.0, self.timer_callback)  # 3초마다 콜백 실행
        self.get_logger().info('Publisher node has been started.')

        self.job_action_server = ActionServer(
            self,
            JobAction,
            'job_command',
            self.execute_callback
        )
        self.conveyor_service = self.create_service(SetBool, 'conveyor_control', self.conveyor_callback)

    def timer_callback(self):

        robot_status = Int16()
        robot_status.data = random.randint(0,2)
        self.robot_status_pub.publish(robot_status)
        self.get_logger().info(f'Robot Status Pub: "{robot_status.data}"') 

        conveyor_status = Bool()
        conveyor_status.data = bool(random.randint(0,1))
        self.conveyor_status_pub.publish(conveyor_status)
        self.get_logger().info(f'Conveyor Status Pub: "{conveyor_status.data}"')
    
    def conveyor_callback(self, request, response):
        status = request.data
        try:
            py_serial = serial.Serial(
                    port='/dev/ttyACM0',
                    baudrate=115200,
                )
            if status:
                self.get_logger().info(f'Activate Conveyor Belt')
                command = 10000
                py_serial.write(command.encode())
                time.sleep(0.1)
                response.message="Activate Conveyor Belt"
            else:
                self.get_logger().info(f'Deactivate Conveyor Belt')
                command = 1
                py_serial.write(command.encode())
                response.message="Deactivate Conveyor Belt"
            
            response.success = True
            return response   
        except:
            response.message="USB is not conect"
        
            response.success = True
            return response   

    async def execute_callback(self, goal_handle):
        self.get_logger().info(f"Goal Received: red_num={goal_handle.request.red_num}, "
                               f"blue_num={goal_handle.request.blue_num}, goal_num={goal_handle.request.goal_num}")

        # Simulate some processing
        red = goal_handle.request.red_num
        blue = goal_handle.request.blue_num
        goal = goal_handle.request.goal_num

        time.sleep(5)


        # Complete the goal
        goal_handle.succeed()
        result = JobAction.Result()
        result.message = f"Calculation complete: Result ="
        result.success = True
        self.get_logger().info(f"Goal Completed: {result.success}")
        return result


def main(args=None):
    rclpy.init(args=args)  # ROS 2 초기화
    node = SimplePublisher()  # 퍼블리셔 노드 생성

    try:
        rclpy.spin(node)  # 노드 실행
    except KeyboardInterrupt:
        pass  # Ctrl+C로 종료
    finally:
        node.destroy_node()  # 노드 소멸
        rclpy.shutdown()  # ROS 2 종료

if __name__ == '__main__':
    main()
