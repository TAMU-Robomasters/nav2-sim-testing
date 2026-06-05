import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Twist
from tf2_ros import TransformBroadcaster
from ctypes import Structure, c_uint8, c_float, sizeof
import serial
import math
import time

# ── Structs ───────────────────────────────────────────────────────────────────

class QueryToEmbedded(Structure):
    _pack_ = 1
    _fields_ = [
        ("magic", c_uint8),
        ("messageType", c_uint8),
    ]

class EmbeddedOdometryMessage(Structure):
    _pack_ = 1
    _fields_ = [
        ("magic", c_uint8),
        ("x", c_float),
        ("y", c_float),
        ("theta", c_float),
    ]

class VelocityToEmbedded(Structure):
    """Velocity command host -> MCU. Sent on /cmd_vel; no response expected.

    Velocities are in the EMBEDDED frame (x=right, y=forward), i.e. the same
    convention the MCU reports odometry in. The MCU's own yaw controller owns
    angular motion, so no omega is sent.
    """
    _pack_ = 1
    _fields_ = [
        ("magic", c_uint8),        # 'a'
        ("messageType", c_uint8),  # 'v'
        ("vx", c_float),           # embedded +x = right    (m/s)
        ("vy", c_float),           # embedded +y = forward  (m/s)
    ]

# ── Node ──────────────────────────────────────────────────────────────────────

class UartOdomNode(Node):
    def __init__(self):
        super().__init__('uart_odom_node')

        # Declare parameters with defaults
        self.declare_parameter('port', '/dev/ttyTHS1')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('update_rate', 100.0)
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('cmd_vel_timeout', 0.3)

        # Get parameter values
        port = self.get_parameter('port').value
        baudrate = self.get_parameter('baudrate').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        publish_tf = self.get_parameter('publish_tf').value
        update_rate = self.get_parameter('update_rate').value
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.cmd_vel_timeout = self.get_parameter('cmd_vel_timeout').value

        self.broadcaster = TransformBroadcaster(self)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.embedded_odom_pub = self.create_publisher(Odometry, '/embedded_odom', 10)

        # /cmd_vel -> UART velocity bridge. Lives in this node because the MCU
        # serial port (/dev/ttyTHS1) is opened exclusively here; a separate node
        # could not also open it. The single-threaded executor serializes the
        # odom query/read and these velocity writes on the one serial line.
        self._last_cmd_vel_time = 0.0  # monotonic seconds; 0 = none received yet
        self.cmd_vel_sub = self.create_subscription(
            Twist, cmd_vel_topic, self.cmd_vel_callback, 10
        )

        try:
            self.ser = serial.Serial(
                port,
                baudrate=baudrate,
                timeout=1.0,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE
            )
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()
            self.get_logger().info(f'Connected to {port} at {baudrate} baud')
        except Exception as e:
            self.get_logger().error(f'Failed to open serial port {port}: {e}')
            self.ser = None
            return

        self.expected_size = sizeof(EmbeddedOdometryMessage())
        self.publish_tf = publish_tf
        
        # Broadcast static TF from embedded_odom to odom (-90° yaw rotation)
        # embedded: x=right, y=forward, z=up
        # odom: x=forward, y=left, z=up
        # Rotation: -90° around z-axis (quaternion: qx=0, qy=0, qz=-0.7071, qw=0.7071)
        self._publish_static_embedded_to_odom_transform()

        timer_period = 1.0 / update_rate
        self.timer = self.create_timer(timer_period, self.query_and_read)

        # Safety watchdog: if no /cmd_vel arrives within cmd_vel_timeout, keep
        # commanding zero velocity so the robot stops when Nav2 dies or a goal
        # completes. Runs at 10 Hz.
        self.cmd_vel_watchdog = self.create_timer(0.1, self.cmd_vel_watchdog_cb)
        self.get_logger().info(
            f'UartOdomNode started at {update_rate}Hz '
            f'(cmd_vel bridge on "{cmd_vel_topic}", timeout {self.cmd_vel_timeout}s)'
        )

    def query_and_read(self):
        if self.ser is None or not self.ser.is_open:
            self.get_logger().warn_once('Serial port not open')
            return

        try:
            # Send query
            query = QueryToEmbedded(magic=ord('a'), messageType=ord('q'))
            self.ser.write(bytes(query))

            # Wait for response
            deadline = time.time() + 0.02  # 20ms timeout
            while self.ser.in_waiting < self.expected_size:
                if time.time() > deadline:
                    self.get_logger().debug('Timeout waiting for odom response')
                    return
                time.sleep(0.001)

            data = self.ser.read(self.expected_size)
            if len(data) != self.expected_size:
                self.get_logger().debug(f'Short read: {len(data)} bytes')
                return

            msg = EmbeddedOdometryMessage.from_buffer_copy(data)

            self.get_logger().info(
                f'[UART RAW]        x={msg.x:.4f}  y={msg.y:.4f}  theta={math.degrees(msg.theta):.2f}deg'
            )
            self.publish_embedded_odom(msg.x, msg.y, msg.theta)
            self.publish_transformed_odom(msg.x, msg.y, msg.theta)

        except Exception as e:
            self.get_logger().error(f'Error in query_and_read: {e}')

    def cmd_vel_callback(self, msg):
        """Convert a base_link Twist to the embedded frame and send over UART.

        ROS base_link: x=forward, y=left. Embedded: x=right, y=forward.
        This is the velocity analog of the inverse of the position mapping in
        publish_transformed_odom (odom_x=emb_y, odom_y=-emb_x):
            emb_vx (right)   = -msg.linear.y
            emb_vy (forward) =  msg.linear.x
        msg.angular.z is intentionally discarded — yaw is owned by the MCU.
        """
        emb_vx = -msg.linear.y
        emb_vy = msg.linear.x
        self._last_cmd_vel_time = time.monotonic()
        self._send_velocity(emb_vx, emb_vy)

    def cmd_vel_watchdog_cb(self):
        """Command zero velocity if /cmd_vel has gone stale (safety stop)."""
        if self._last_cmd_vel_time == 0.0:
            return  # never received a command yet — don't fight other senders
        if time.monotonic() - self._last_cmd_vel_time > self.cmd_vel_timeout:
            self._send_velocity(0.0, 0.0)

    def _send_velocity(self, emb_vx, emb_vy):
        """Write a VelocityToEmbedded message to the MCU (no response expected)."""
        if self.ser is None or not self.ser.is_open:
            self.get_logger().warn_once('Serial port not open; dropping cmd_vel')
            return
        try:
            cmd = VelocityToEmbedded(
                magic=ord('a'),
                messageType=ord('v'),
                vx=float(emb_vx),
                vy=float(emb_vy),
            )
            self.ser.write(bytes(cmd))
            self.get_logger().debug(
                f'[UART CMD] emb_vx={emb_vx:.4f}  emb_vy={emb_vy:.4f}'
            )
        except Exception as e:
            self.get_logger().error(f'Error sending velocity: {e}')

    def _publish_static_embedded_to_odom_transform(self):
        """Publish static TF from embedded_odom to odom frame.
        
        Embedded frame: x=right, y=forward, z=up
        Odom frame: x=forward, y=left, z=up
        Transformation: -90° rotation around z-axis
        """
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'embedded_odom'
        t.child_frame_id = self.odom_frame
        
        # No translation
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        
        # -90° rotation around z-axis (quaternion)
        # qz = sin(-π/4) = -0.7071, qw = cos(-π/4) = 0.7071
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = -0.7071
        t.transform.rotation.w = 0.7071
        
        self.broadcaster.sendTransform(t)
        self.get_logger().info('Published static TF: embedded_odom -> odom')

    def publish_embedded_odom(self, x, y, theta):
        """Publish raw odometry in embedded_odom frame."""
        now = self.get_clock().now().to_msg()
        
        # Quaternion for embedded frame theta (yaw around z)
        qz = math.sin(theta / 2.0)
        qw = math.cos(theta / 2.0)
        
        embedded_odom = Odometry()
        embedded_odom.header.stamp = now
        embedded_odom.header.frame_id = 'embedded_odom'
        embedded_odom.child_frame_id = self.base_frame
        
        embedded_odom.pose.pose.position.x = x
        embedded_odom.pose.pose.position.y = y
        embedded_odom.pose.pose.position.z = 0.0
        embedded_odom.pose.pose.orientation.x = 0.0
        embedded_odom.pose.pose.orientation.y = 0.0
        embedded_odom.pose.pose.orientation.z = qz
        embedded_odom.pose.pose.orientation.w = qw
        
        self.embedded_odom_pub.publish(embedded_odom)

    def publish_transformed_odom(self, x, y, theta):
        """Publish transformed odometry in odom frame.
        
        Transforms from embedded frame (x=right, y=forward) to odom frame (x=forward, y=left).
        """
        now = self.get_clock().now().to_msg()
        
        # Transform embedded coordinates to odom coordinates
        # embedded x (right) -> odom -y, embedded y (forward) -> odom x
        odom_x = y
        odom_y = -x
        
        # theta=0 means facing embedded +y (forward), which maps to ROS +x — no offset needed
        odom_theta = theta
        
        # Quaternion for transformed theta
        qz = math.sin(odom_theta / 2.0)
        qw = math.cos(odom_theta / 2.0)
        
        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        
        odom.pose.pose.position.x = odom_x
        odom.pose.pose.position.y = odom_y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        
        self.get_logger().info(
            f'[UART TRANSFORMED] x={odom_x:.4f}  y={odom_y:.4f}  theta={math.degrees(odom_theta):.2f}deg'
        )
        self.odom_pub.publish(odom)

        # --- Publish TF transform ---
        if self.publish_tf:
            t = TransformStamped()
            t.header.stamp = now
            t.header.frame_id = self.odom_frame
            t.child_frame_id = self.base_frame
            
            t.transform.translation.x = odom_x
            t.transform.translation.y = odom_y
            t.transform.translation.z = 0.0
            
            t.transform.rotation.x = 0.0
            t.transform.rotation.y = 0.0
            t.transform.rotation.z = qz
            t.transform.rotation.w = qw
            
            self.broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)

    uart_odom_node = UartOdomNode()

    try:
        rclpy.spin(uart_odom_node)
    except KeyboardInterrupt:
        pass
    finally:
        if hasattr(uart_odom_node, 'ser') and uart_odom_node.ser is not None:
            uart_odom_node.ser.close()
        uart_odom_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
