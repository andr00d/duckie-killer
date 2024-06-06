import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class NavigatorNode(Node):
    def __init__(self):
        super().__init__('navigator')
        
        # Initialize publishers for different modes
        self.home_publisher = self.create_publisher(String, 'home_follower', 10)
        self.guard_publisher = self.create_publisher(String, 'guard_follower', 10)
        self.surveillance_publisher = self.create_publisher(String, 'surveillance_follower', 10)
        
        # Initialize subscriptions
        self.subscription_objects = self.create_subscription(
            String,
            'detected_objects',
            self.objects_callback,
            10)
        
        self.subscription_mode = self.create_subscription(
            String,
            'mode',
            self.mode_callback,
            10)
        
        self.current_mode = None
        self.get_logger().info('Navigator Node has been started.')
        
    def objects_callback(self, msg):
        if self.current_mode == "Home":
            self.home_publisher.publish(msg)
            #self.get_logger().info(f'Redirecting to home_follower: {msg.data}')
        elif self.current_mode == "Guard":
            self.guard_publisher.publish(msg)
            #self.get_logger().info(f'Redirecting to guard_follower: {msg.data}')
        elif self.current_mode == "Surveillance":
            self.surveillance_publisher.publish(msg)
            #self.get_logger().info(f'Redirecting to surveillance_follower: {msg.data}')
        else:
            self.get_logger().warn('Mode is not set or invalid. Message will not be redirected.')

    def mode_callback(self, msg):
        self.current_mode = msg.data
        #self.get_logger().info(f'Mode set to: {self.current_mode}')

def main(args=None):
    rclpy.init(args=args)
    node = NavigatorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

