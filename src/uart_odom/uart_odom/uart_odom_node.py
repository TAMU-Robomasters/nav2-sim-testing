import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Twist
from tf2_ros import TransformBroadcaster
from ctypes import Structure, c_uint8, c_uint16, c_float, sizeof
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

    Velocities are in the embedded WORLD frame (embedded_odom: x=right,
    y=forward) — a fixed, map-fixed direction, NOT the body/chassis frame. The
    chassis spins independently under MCU control, so the MCU rotates this world
    velocity world->turret->chassis with its own live gyro/encoder angles before
    driving the wheels. cmd_vel_callback produces it by rotating the base_link
    Twist by the latest gyro heading. No omega is sent — yaw is owned by the MCU.
    """
    _pack_ = 1
    _fields_ = [
        ("magic", c_uint8),        # 'a'
        ("messageType", c_uint8),  # 'v'
        ("vx", c_float),           # embedded +x = right    (m/s)
        ("vy", c_float),           # embedded +y = forward  (m/s)
    ]

# ── MCU bridge structs (mirror AutoX src/subsystems/embedded_communicator.py) ───
# Only used when enable_mcu_bridge is true. The MCU dispatches by messageType, so
# the 3-byte transform query ('t') coexists on the wire with the 2-byte odom
# query ('q') above.

class TransformQuery(Structure):
    """Host -> MCU: request the camera->ballistic transform (messageType 't')."""
    _pack_ = 1
    _fields_ = [
        ("magic", c_uint8),         # 'a'
        ("messageType", c_uint8),   # 't'
        ("frameDelay_ms", c_uint8),
    ]

class EmbeddedTransformationMessage(Structure):
    """MCU -> Host: current turret transformation (69 bytes packed)."""
    _pack_ = 1
    _fields_ = [
        ("magic", c_uint8),
        ("yaw", c_float),
        ("pitch", c_float),
        ("matrix", c_float * 16),
    ]

class JetsonMessage(Structure):
    """Host -> MCU: firing solution (messageType 'd', 12 bytes packed)."""
    _pack_ = 1
    _fields_ = [
        ("magic", c_uint8),
        ("messageType", c_uint8),
        ("pitch", c_float),
        ("yaw", c_float),
        ("timeUntilNextFire", c_uint16),
        ("cvState", c_uint8),
    ]

# ── Node ──────────────────────────────────────────────────────────────────────

class UartOdomNode(Node):
    def __init__(self):
        super().__init__('uart_odom_node')

        # Declare parameters with defaults
        self.declare_parameter('port', '/dev/ttyTHS1')
        self.declare_parameter('baudrate', 460800)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('update_rate', 100.0)
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('cmd_vel_timeout', 0.3)
        # When true, this node also serves AutoX's MCU traffic (camera->ballistic
        # transform query + firing-solution sink) over the same serial port. Off
        # by default so odom-only mode is unchanged.
        self.declare_parameter('enable_mcu_bridge', False)

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
        # Latest gyro heading (theta) from the odom poll, cached so cmd_vel_callback
        # can rotate base_link commands into the world frame. The single-threaded
        # executor serializes the poll and the callback, so no lock is needed. Starts
        # at 0.0 (identity) — the world-frame send then reduces to the legacy
        # body-frame mapping until the first odom sample arrives.
        self._latest_theta = 0.0
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

        # MCU bridge (AutoX). Lazy-import the interfaces so odom-only mode does not
        # require mcu_msgs to be built/sourced. The transform service does the
        # serial round-trip synchronously; at 460800 the 69-byte reply is ~1.5 ms.
        # All callbacks share this node's default MutuallyExclusiveCallbackGroup
        # and the executor is single-threaded, so the service can never overlap the
        # odom poll on the one serial line.
        self.enable_mcu_bridge = self.get_parameter('enable_mcu_bridge').value
        if self.enable_mcu_bridge:
            from mcu_msgs.srv import QueryTransform
            from mcu_msgs.msg import FiringSolution

            self._FiringSolution = FiringSolution
            self._transform_size = sizeof(EmbeddedTransformationMessage())
            self.query_transform_srv = self.create_service(
                QueryTransform, '/mcu/query_transform', self.on_query_transform
            )
            self.firing_solution_sub = self.create_subscription(
                FiringSolution, '/mcu/firing_solution', self.on_firing_solution, 1
            )
            self.get_logger().info(
                'MCU bridge enabled: /mcu/query_transform (srv) + /mcu/firing_solution (topic)'
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
            self._latest_theta = msg.theta  # cache for the cmd_vel world-frame rotation

            self.get_logger().info(
                f'[UART RAW]        x={msg.x:.4f}  y={msg.y:.4f}  theta={math.degrees(msg.theta):.2f}deg'
            )
            self.publish_embedded_odom(msg.x, msg.y, msg.theta)
            self.publish_transformed_odom(msg.x, msg.y, msg.theta)

        except Exception as e:
            self.get_logger().error(f'Error in query_and_read: {e}')

    # ── MCU bridge handlers (only created when enable_mcu_bridge) ────────────
    def on_query_transform(self, request, response):
        """Service: do a synchronous transform round-trip and fill the response.

        Runs in the node's default (mutually-exclusive) callback group on the
        single-threaded executor, so it never overlaps query_and_read on the wire.
        """
        result = self._query_transform(request.frame_delay_ms)
        if result is None:
            response.success = False
            return response
        yaw, pitch, matrix = result
        response.success = True
        response.yaw = yaw
        response.pitch = pitch
        response.matrix = matrix
        return response

    def _query_transform(self, frame_delay_ms):
        """Write a TransformQuery and read the 69-byte reply, or None on failure."""
        if self.ser is None or not self.ser.is_open:
            self.get_logger().warn_once('Serial port not open; cannot query transform')
            return None
        try:
            # Clear any stale bytes so the reply is frame-aligned to this query.
            self.ser.reset_input_buffer()
            query = TransformQuery(
                magic=ord('a'), messageType=ord('t'),
                frameDelay_ms=int(frame_delay_ms) & 0xFF,
            )
            self.ser.write(bytes(query))

            deadline = time.time() + 0.02  # ample at 460800 (~1.5 ms reply)
            while self.ser.in_waiting < self._transform_size:
                if time.time() > deadline:
                    self.get_logger().debug('Timeout waiting for transform response')
                    return None
                time.sleep(0.0005)

            data = self.ser.read(self._transform_size)
            if len(data) != self._transform_size:
                self.get_logger().debug(f'Short transform read: {len(data)} bytes')
                return None
            msg = EmbeddedTransformationMessage.from_buffer_copy(data)
            return float(msg.yaw), float(msg.pitch), [float(v) for v in msg.matrix]
        except Exception as e:
            self.get_logger().error(f'Error in _query_transform: {e}')
            return None

    def on_firing_solution(self, msg):
        """Topic: forward a firing solution to the MCU (fire-and-forget)."""
        if self.ser is None or not self.ser.is_open:
            self.get_logger().warn_once('Serial port not open; dropping firing solution')
            return
        try:
            sol = JetsonMessage(
                magic=ord('a'), messageType=ord('d'),
                pitch=float(msg.pitch), yaw=float(msg.yaw),
                timeUntilNextFire=int(msg.time_until_fire) & 0xFFFF,
                cvState=int(msg.cv_state) & 0xFF,
            )
            self.ser.write(bytes(sol))
        except Exception as e:
            self.get_logger().error(f'Error sending firing solution: {e}')

    def cmd_vel_callback(self, msg):
        """Rotate a base_link Twist into the world frame and send it over UART.

        DWB publishes /cmd_vel in base_link (x=forward, y=left). The chassis spins
        independently under MCU control, so a body-frame command would point the
        wrong way by the time the MCU applies it (and at 80 Hz minus serial latency
        ROS can't chase the spin). Instead we express the command in the fixed world
        frame and let the MCU do world->turret->chassis with its own live angles.

        base_link -> odom (world): rotate by theta, base_link's yaw in odom — which
        is exactly the gyro heading we publish as the odom yaw, so the MCU's gyro and
        this rotation share one reference and the map<->gyro offset cancels:
            v_odom_x (forward) = cos(th)*vx - sin(th)*vy
            v_odom_y (left)    = sin(th)*vx + cos(th)*vy
        then relabel odom (x=fwd, y=left) to the embedded world convention the MCU
        reasons in (x=right, y=forward):
            emb_vx (right)   = -v_odom_y
            emb_vy (forward) =  v_odom_x
        At theta=0 this reduces to the legacy mapping (emb_vx=-vy, emb_vy=vx).
        msg.angular.z is intentionally discarded — yaw is owned by the MCU.
        """
        th = self._latest_theta
        cos_th = math.cos(th)
        sin_th = math.sin(th)
        v_odom_x = cos_th * msg.linear.x - sin_th * msg.linear.y
        v_odom_y = sin_th * msg.linear.x + cos_th * msg.linear.y
        emb_vx = -v_odom_y
        emb_vy = v_odom_x
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
