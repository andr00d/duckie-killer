import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class ModePublisher(Node):
    def __init__(self):
        super().__init__('mode_publisher')
        self.publisher_ = self.create_publisher(String, 'mode', 10)
        
        # Ask user for the mode at the start
        self.mode_input = self.get_user_mode_input()
        
        # Create a timer to publish the mode at 10Hz
        self.timer = self.create_timer(0.1, self.timer_callback)  # 0.1 seconds = 10Hz
        
        self.get_logger().info('Mode Publisher Node has been started.')

    def get_user_mode_input(self):
        valid_modes = ["Home", "Guard", "Surveillance"]
        while True:
            mode_input = "Guard"#input("Enter mode (Home, Guard, Surveillance): ").strip()
            if mode_input in valid_modes:
                return mode_input
            else:
                self.get_logger().warn(f'Invalid mode: {mode_input}, please enter one of {valid_modes}')

    def timer_callback(self):
        msg = String()
        msg.data = self.mode_input
        self.publisher_.publish(msg)
        #self.get_logger().info(f'Published mode: {self.mode_input}')

def main(args=None):
    rclpy.init(args=args)
    node = ModePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

