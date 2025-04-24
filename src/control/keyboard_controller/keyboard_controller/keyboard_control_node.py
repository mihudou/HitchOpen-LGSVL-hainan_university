#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from lgsvl_msgs.msg import VehicleControlData
from pynput import keyboard
import threading

class KeyboardControlNode(Node):
    def __init__(self):
        super().__init__('keyboard_control_node')
        
        # Create publisher for vehicle control
        self.publisher = self.create_publisher(VehicleControlData, '/lgsvl/control', 10)
        
        # Initialize control values
        self.throttle = 0.0
        self.brake = 0.0
        self.steering = 0.0
        self.current_gear = VehicleControlData.GEAR_DRIVE
        
        # Control parameters
        self.throttle_step = 0.1
        self.brake_step = 0.1
        self.steering_step = 0.1
        self.max_steering = 1.0
        
        
        # Create timer for publishing control messages
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10Hz
        
        # Start keyboard listener in a separate thread
        self.keyboard_thread = threading.Thread(target=self.keyboard_listener)
        self.keyboard_thread.daemon = True
        self.keyboard_thread.start()
        
        # Print instructions
        self.get_logger().info("Keyboard Controller Started")
        self.get_logger().info("Controls:")
        self.get_logger().info("W: Accelerate")
        self.get_logger().info("S: Brake")
        self.get_logger().info("A/D: Steer Left/Right")
        self.get_logger().info("↑: Shift up gear")
        self.get_logger().info("↓: Shift down gear")
        self.get_logger().info("Q: Quit")

    def keyboard_listener(self):
        with keyboard.Listener(on_press=self.on_press, on_release=self.on_release) as listener:
            listener.join()

    def shift_up(self):
        if self.current_gear < 6:
            self.current_gear += 1
            self.get_logger().info(f'Shifted up to gear {self.current_gear}')
        else:
            self.get_logger().info(f'Already on max gear {self.current_gear}')

    def shift_down(self):
        if self.current_gear > 0:
            self.current_gear -= 1
            self.get_logger().info(f'Shifted down to gear {self.current_gear}')
        else:
            self.get_logger().info(f'Already on min gear {self.current_gear}')

    def on_press(self, key):
        try:
            if key.char == 'w':
                self.throttle = min(1.0, self.throttle + self.throttle_step)
                self.brake = 0.0
                self.get_logger().info(f'Throttle: {self.throttle}')
            elif key.char == 's':
                self.brake = min(1.0, self.brake + self.brake_step)
                self.throttle = 0.0
                self.get_logger().info(f'Brake: {self.brake}')
            elif key.char == 'a':
                self.steering = min(self.max_steering, self.steering - self.steering_step)
                self.get_logger().info(f'Steering: {self.steering}')
            elif key.char == 'd':
                self.steering = max(-self.max_steering, self.steering + self.steering_step)
                self.get_logger().info(f'Steering: {self.steering}')
            elif key.char == 'q':
                # Quit the node
                self.get_logger().info('Shutting down...')
                rclpy.shutdown()
        except AttributeError:
            # Handle special keys
            if key == keyboard.Key.up:
                self.shift_up()
            elif key == keyboard.Key.down:
                self.shift_down()
            pass

    def on_release(self, key):
        try:
            if key.char == 'w':
                self.throttle = 0.0
            elif key.char == 's':
                self.brake = 0.0
            elif key.char in ['a', 'd']:
                self.steering = 0.0
        except AttributeError:
            pass

    def timer_callback(self):
        msg = VehicleControlData()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        
        # Set control values
        msg.acceleration_pct = float(self.throttle)
        msg.braking_pct = float(self.brake)
        msg.target_wheel_angle = float(self.steering)
        msg.target_wheel_angular_rate = 0.0
        msg.target_gear = self.current_gear

        # self.get_logger().info(f'Publishing control message: {msg}')
        # Publish the control message
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardControlNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
