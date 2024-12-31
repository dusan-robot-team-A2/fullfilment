import rclpy
from rclpy.node import Node
from fullfilment_interfaces.srv import MoveToBasket

class BasketClient(Node):
    def __init__(self):
        super().__init__('basket_client')

        # 'nav2_basket' 서비스 클라이언트 생성
        self.client = self.create_client(MoveToBasket, 'nav2_basket')

        # 서비스가 준비될 때까지 기다림
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('서비스가 준비되지 않았습니다. 1초 후 다시 시도합니다...')

        # 서비스 요청을 생성
        self.request = MoveToBasket.Request()
        self.request.call = 1  # 요청의 call 값 설정 (예시로 1로 설정)

    def send_request(self):
        # 서비스 요청을 보내고 응답을 기다림
        future = self.client.call_async(self.request)
        future.add_done_callback(self.callback)

    def callback(self, future):
        # 응답을 처리하는 콜백 함수
        response = future.result()
        if response.success:
            self.get_logger().info(f"성공: {response.message}")
        else:
            self.get_logger().info(f"실패: {response.message}")

def main(args=None):
    rclpy.init(args=args)

    # 클라이언트 객체 생성
    basket_client = BasketClient()

    # 요청 보내기
    basket_client.send_request()

    # 응답을 기다림
    rclpy.spin(basket_client)

    # 종료 시 자원 정리
    basket_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
