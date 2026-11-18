#!/usr/bin/env python3
"""Headless ROS2 integration checks. Source install/setup.bash before running.
Uses an isolated ROS_DOMAIN_ID supplied by the caller and cleans up child processes.
"""
import math
import os
from pathlib import Path
import signal
import struct
import subprocess
import time

import rclpy
from ament_index_python.packages import get_package_prefix
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, Image, Imu, CameraInfo, PointField
from geometry_msgs.msg import PoseStamped, Vector3Stamped
from std_msgs.msg import Header
from visualization_msgs.msg import Marker
from quadrotor_msgs.msg import PositionCommand, GoalSet

ROOT = Path(__file__).resolve().parents[1]
LOG = ROOT / 'log' / 'runtime_smoke'
LOG.mkdir(parents=True, exist_ok=True)

class Runner:
    def __init__(self, node):
        self.node = node
        self.processes = []
        self.files = []
        self.subscriptions = []
        self.latest = {}
        self.counts = {}

    def start(self, name, args):
        # Run standalone nodes directly so SIGINT reaches the actual node.
        # ros2 launch remains the process supervisor for multi-node backends.
        if args[:2] == ['ros2', 'run']:
            package, executable = args[2:4]
            args = [str(Path(get_package_prefix(package))/'lib'/package/executable), *args[4:]]
        f = open(LOG / (name+'.log'), 'w')
        p = subprocess.Popen(args, stdout=f, stderr=subprocess.STDOUT, start_new_session=True)
        self.processes.append(p)
        self.files.append(f)
        return p

    def subscribe(self, key, typ, topic):
        def receive(msg):
            self.latest[key] = msg
            self.counts[key] = self.counts.get(key, 0)+1
        self.subscriptions.append(self.node.create_subscription(typ, topic, receive, qos_profile_sensor_data))

    def wait(self, condition, seconds=15, tick=None):
        deadline = time.monotonic()+seconds
        next_tick = 0.0
        while time.monotonic() < deadline:
            for p in self.processes:
                assert p.poll() is None, f'Child exited ({p.returncode}); inspect {LOG}'
            if tick and time.monotonic() >= next_tick:
                tick()
                next_tick = time.monotonic()+0.05
            rclpy.spin_once(self.node, timeout_sec=0.05)
            if condition(): return
        raise AssertionError(f'Timed out; received {self.counts}; inspect {LOG}')

    def close(self):
        forced = []
        for p in self.processes:
            if p.poll() is None: p.send_signal(signal.SIGINT)
        for p in self.processes:
            try: p.wait(timeout=8)
            except subprocess.TimeoutExpired:
                forced.append(p.pid)
                os.killpg(p.pid, signal.SIGKILL)
                p.wait()
        for f in self.files:
            name = f.name
            f.close()
            contents = Path(name).read_text()
            assert 'process has died' not in contents, f'Unclean child exit in {name}'
            assert 'terminate called' not in contents, f'Uncaught exception in {name}'
        for sub in self.subscriptions: self.node.destroy_subscription(sub)
        assert not forced, f'Nodes required forced termination: {forced}'


def backend_check(node, backend):
    r = Runner(node)
    ns = 'test_'+backend
    try:
        r.subscribe('odom', Odometry, f'/{ns}/odometry')
        r.subscribe('cloud', PointCloud2, f'/{ns}/cloud')
        r.subscribe('depth', Image, f'/{ns}/depth')
        r.subscribe('camera', CameraInfo, f'/{ns}/camera_info')
        r.subscribe('robot', Marker, f'/{ns}/robot')
        if backend == 'so3': r.subscribe('imu', Imu, f'/{ns}/imu')
        pub = node.create_publisher(PositionCommand, f'/{ns}/position_cmd', 10)
        r.start(backend, ['ros2', 'launch', 'so3_quadrotor_simulator', 'simulator_example.launch.py',
                          'rviz:=false', f'backend:={backend}', f'namespace:={ns}', 'depth:=true'])
        r.wait(lambda: all(k in r.latest for k in ['odom','cloud','depth','camera','robot']))
        assert node.count_publishers(f'/{ns}/odometry') == 1, 'Duplicate state publishers'
        assert r.latest['odom'].child_frame_id == ns+'/base_link'
        assert r.latest['cloud'].width > 0
        assert r.latest['depth'].encoding == '32FC1'
        assert len(r.latest['depth'].data) == 640*480*4
        assert r.latest['camera'].k[0] > 0
        assert r.latest['odom'].header.stamp.sec > 0
        cmd = PositionCommand()
        cmd.position.x,cmd.position.y,cmd.position.z = 0.5,0.25,1.2
        cmd.trajectory_flag = PositionCommand.TRAJECTORY_STATUS_READY
        def publish():
            cmd.header.stamp = node.get_clock().now().to_msg()
            pub.publish(cmd)
        def error():
            p = r.latest['odom'].pose.pose.position
            return math.dist((p.x,p.y,p.z),(0.5,0.25,1.2))
        r.wait(lambda: error() < (1e-5 if backend == 'fake' else 0.15), seconds=20, tick=publish)
        q=r.latest['odom'].pose.pose.orientation
        assert abs(math.sqrt(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w)-1)<0.01
        if backend=='so3':
            hold_until = time.monotonic()+5
            r.wait(lambda:time.monotonic()>=hold_until,seconds=7,tick=publish)
            assert error()<0.15, f'SO3 steady tracking error {error()}'
            r.wait(lambda: 'imu' in r.latest)
            assert r.latest['imu'].header.stamp.sec>0
            assert all(math.isfinite(v) for v in (q.x,q.y,q.z,q.w,r.latest['imu'].linear_acceleration.z))
        else:
            cmd.acceleration.x=1.0
            old=r.counts['odom']
            r.wait(lambda:r.counts['odom']>old+5,tick=publish)
            assert abs(r.latest['odom'].twist.twist.angular.x)<1e-9, 'Acceleration leaked into angular velocity'
        print(f'PASS {backend}: commands, one odometry publisher, markers, cloud, depth; position error={error():.4f}',flush=True)
        node.destroy_publisher(pub)
    finally: r.close()


def sensor_check(node):
    r=Runner(node)
    ns='test_sensor'
    try:
        r.subscribe('cloud',PointCloud2,f'/{ns}/cloud')
        r.subscribe('depth',Image,f'/{ns}/depth')
        r.start('sensor',['ros2','run','local_sensing_node','pcl_render_node','--ros-args','-r',f'__ns:=/{ns}',
                         '-p','publish_depth:=true','-p','voxel_size:=0.01'])
        pub=node.create_publisher(PointCloud2,f'/{ns}/global_map',QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        dynamic=node.create_publisher(PointCloud2,f'/{ns}/local_map',10)
        odom_pub=node.create_publisher(Odometry,f'/{ns}/odometry',10)
        def cloud(points):
            return PointCloud2(header=Header(frame_id='world',stamp=node.get_clock().now().to_msg()),
                height=1,width=len(points),fields=[PointField(name=n,offset=i*4,datatype=PointField.FLOAT32,count=1) for i,n in enumerate(['x','y','z'])],
                point_step=12,row_step=12*len(points),is_dense=True,
                data=b''.join(struct.pack('<fff',*p) for p in points))
        odom=Odometry();odom.header.frame_id='world';odom.child_frame_id='sensor_body';odom.pose.pose.orientation.w=1.0
        points=[(2.0,0.0,0.0),(4.0,0.0,0.0),(30.0,0.0,0.0)]
        def tick():
            odom.header.stamp=node.get_clock().now().to_msg()
            odom_pub.publish(odom);pub.publish(cloud(points))
        r.wait(lambda: 'depth' in r.latest and 'cloud' in r.latest,tick=tick)
        values=struct.unpack('<'+'f'*(640*480),bytes(r.latest['depth'].data))
        assert abs(values[243*640+321]-2.0)<1e-5, 'Nearest point must win depth projection'
        assert r.latest['cloud'].width==2, 'Out-of-range point must be excluded'
        points[:]=[(3.0,0.0,0.0)]
        r.wait(lambda:r.latest['cloud'].width==1,tick=tick)
        r.wait(lambda:abs(struct.unpack_from('<f',bytes(r.latest['depth'].data),(243*640+321)*4)[0]-3)<1e-5,tick=tick)
        def dynamic_tick():
            tick();dynamic.publish(cloud([(1.0,0.0,0.0)]))
        r.wait(lambda:r.latest['cloud'].width==2,tick=dynamic_tick)
        print('PASS sensor: range filtering, nearest depth, global map refresh, dynamic map input',flush=True)
        for p in [pub,dynamic,odom_pub]:node.destroy_publisher(p)
    finally:r.close()


def map_check(node):
    r=Runner(node)
    try:
        r.subscribe('map',PointCloud2,'/test_mock/mock_map')
        r.start('mockamap',['ros2','run','mockamap','mockamap_node','--ros-args','-r','__ns:=/test_mock',
                          '-p','x_length:=8','-p','y_length:=8','-p','z_length:=3','-p','type:=1'])
        r.wait(lambda:'map' in r.latest)
        assert r.latest['map'].width>0 and r.latest['map'].header.stamp.sec>0
        print('PASS mockamap: nonempty stamped point cloud',flush=True)
    finally:r.close()


def utility_check(node):
    r = Runner(node)
    ns = '/test_utils'
    try:
        r.subscribe('euler', Vector3Stamped, ns+'/euler')
        r.subscribe('goal', GoalSet, ns+'/goal_user2brig')
        for executable in ('send_odom.py', 'odom_to_euler.py', 'topic_statistics.py'):
            args = ['ros2', 'run', 'uav_utils', executable, '--ros-args', '-r', '__ns:='+ns]
            if executable == 'topic_statistics.py': args += ['-p', 'topic:=odom']
            r.start(executable, args)
        r.start('random_goals', ['ros2', 'run', 'random_goals', 'random_goals_node', '--ros-args',
                                '-r', '__ns:='+ns, '-r', 'drone_0/odometry:=odom',
                                '-p', 'goal0:=[2.0, 0.5, 0.0]'])
        r.wait(lambda: 'euler' in r.latest and 'goal' in r.latest)
        e = r.latest['euler'].vector
        assert max(abs(e.x), abs(e.y), abs(e.z)) < 1e-9
        assert r.latest['goal'].drone_id == 0
        assert list(r.latest['goal'].goal) == [2.0, 0.5, 0.0]
        finish = time.monotonic()+3
        r.wait(lambda: time.monotonic() >= finish, seconds=5)
        assert 'Hz' in (LOG/'topic_statistics.py.log').read_text()
        print('PASS utilities: odometry source, Euler conversion, traffic statistics, random goal dispatch', flush=True)
    finally:
        r.close()

if __name__=='__main__':
    rclpy.init()
    node=rclpy.create_node('simulation_integration_probe')
    try:
        backend_check(node,'fake')
        backend_check(node,'so3')
        sensor_check(node)
        map_check(node)
        utility_check(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
