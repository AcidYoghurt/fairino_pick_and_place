import subprocess
import time
import threading
import os
import signal
from rclpy.node import Node
import rclpy

class FairinoGuardThread(Node):
    def __init__(self):
        super().__init__('fairino_guard_thread_node')
        self.declare_parameter('ping_target', '192.168.58.3')
        self._ping_target = self.get_parameter('ping_target').value
        self._process = None

        self._monitor_thread = threading.Thread(target=self.monitor_ping, daemon=True)
        self._monitor_thread.start()
        self.get_logger().info("法奥守护线程启动中")

    def monitor_ping(self):
        while rclpy.ok():
            # 建议加上 -W 1 参数设置超时时间，防止ping不通时脚本卡住
            response = subprocess.run(['ping', '-c', '1', '-W', '1', self._ping_target], stdout=subprocess.PIPE)

            if response.returncode == 0:
                self.get_logger().info(f"{self._ping_target} 能ping通") # 日志太频繁可以注释掉
                self.start_launch()
            else:
                self.get_logger().warn(f"{self._ping_target} ping不通")
                self.stop_launch()

            time.sleep(2)

    # 启动launch文件
    def start_launch(self):
        if not self._process:
            self.get_logger().info(f"启动launch文件")
            self._process = subprocess.Popen(
                ['ros2', 'launch', 'fairino_bringup', 'fairino_bringup.launch.py'],
                preexec_fn=os.setsid
            )

    # 结束launch文件
    def stop_launch(self):
        if self._process:
            try:
                pgid = os.getpgid(self._process.pid)

                # 先 SIGTERM
                os.killpg(pgid, signal.SIGTERM)
                time.sleep(1)

                # 再 SIGKILL（无条件强杀）
                os.killpg(pgid, signal.SIGKILL)

            except Exception as e:
                self.get_logger().error(f"停止进程出错: {e}")
            finally:
                self._process = None



def main(args=None):
    rclpy.init(args=args)
    node = FairinoGuardThread()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # 退出脚本时也要清理
        if node._process:
            node.stop_launch()
    finally:
        rclpy.shutdown()