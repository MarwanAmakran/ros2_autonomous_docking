import sys
import threading
import time
import termios
import tty
import serial
import select
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

KEYMAP = {
    'z': "D 50 50 1",     # forwards
    'a': "D 50 -50 1",    # going left
    'e': "D -50 50 1",    # going right
    's': "D -50 -50 1",   # backwards
}

HELP = """\
Controls:
  z = forwards
  a = going left
  e = going right
  s = backwards
  q = quitting

Focus the terminal window and press keys (no Enter needed).
"""


class KeyboardPublisher(Node):
    def __init__(self):
        super().__init__('keyboard_publisher')

        self.publisher = self.create_publisher(String, 'cmd_key', 10)

        self.get_logger().info("Keyboard publisher started")
        self.get_logger().info("\n" + HELP)

        self._orig_term = termios.tcgetattr(sys.stdin.fileno())
        tty.setcbreak(sys.stdin.fileno())

        self.timer = self.create_timer(0.02, self._poll_keyboard)

    def _poll_keyboard(self):
        try:
            rlist, _, _ = select.select([sys.stdin], [], [], 0)
            if rlist:
                ch = sys.stdin.read(1)

                if ch == 'q':
                    self.get_logger().info("Quitting")
                    rclpy.shutdown()
                    return

                if ch in KEYMAP:
                    msg = String()
                    msg.data = KEYMAP[ch]
                    self.publisher.publish(msg)
                    self.get_logger().info(f"> {msg.data}")

                elif ch == 'h':
                    self.get_logger().info("\n" + HELP)

        except Exception as e:
            self.get_logger().warn(f"Keyboard error: {e}")

    def destroy_node(self):
        try:
            termios.tcsetattr(sys.stdin.fileno(), termios.TCSADRAIN, self._orig_term)
        except Exception:
            pass
        super().destroy_node()


def main():
    rclpy.init()
    node = KeyboardPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()


if __name__ == '__main__':
    main()
