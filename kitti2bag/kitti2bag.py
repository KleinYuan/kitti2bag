#!env python
# -*- coding: utf-8 -*-

import sys
import fire

try:
    import pykitti
except ImportError as e:
    print('Could not load module \'pykitti\'. Please run `pip install pykitti`')
    sys.exit(1)

import tf
import os
import cv2
import rospy
import rosbag
import progressbar
from tf2_msgs.msg import TFMessage
from datetime import datetime
from std_msgs.msg import Header
from sensor_msgs.msg import CameraInfo, Imu, PointField, NavSatFix
import sensor_msgs.point_cloud2 as pcl2
from geometry_msgs.msg import TransformStamped, TwistStamped, Transform
from cv_bridge import CvBridge
import numpy as np


def save_imu_data(bag, kitti, imu_frame_id, topic):
    print("Exporting IMU")
    for timestamp, oxts in zip(kitti.timestamps, kitti.oxts):
        q = tf.transformations.quaternion_from_euler(oxts.packet.roll, oxts.packet.pitch, oxts.packet.yaw)
        imu = Imu()
        imu.header.frame_id = imu_frame_id
        imu.header.stamp = rospy.Time.from_sec(float(timestamp.strftime("%s.%f")))
        imu.orientation.x = q[0]
        imu.orientation.y = q[1]
        imu.orientation.z = q[2]
        imu.orientation.w = q[3]
        imu.linear_acceleration.x = oxts.packet.af
        imu.linear_acceleration.y = oxts.packet.al
        imu.linear_acceleration.z = oxts.packet.au
        imu.angular_velocity.x = oxts.packet.wf
        imu.angular_velocity.y = oxts.packet.wl
        imu.angular_velocity.z = oxts.packet.wu
        bag.write(topic, imu, t=imu.header.stamp)


def save_dynamic_tf(bag, kitti, kitti_type, initial_time, T_cam0_imu=None):
    print("Exporting time dependent transformations")
    if kitti_type.find("raw") != -1:
        T_cam0_init_w = None
        ts_init = None
        ts_0 = datetime.utcfromtimestamp(1.0)
        for timestamp, oxts in zip(kitti.timestamps, kitti.oxts):
            if ts_init is None:
                ts_init = timestamp
                print(ts_init)
            tf_oxts_msg = TFMessage()
            tf_oxts_transform = TransformStamped()
            tf_oxts_transform.header.stamp = rospy.Time.from_sec(float((timestamp - ts_init + ts_0).strftime("%s.%f")))
            tf_oxts_transform.header.stamp.nsecs = round(tf_oxts_transform.header.stamp.nsecs / 1e6) * 1000000
            tf_oxts_transform.header.frame_id = '/local_cs'
            tf_oxts_transform.child_frame_id = '/sensor/camera'  # /sensor/camera is  camera_left

            # http://wiki.ros.org/geometry/CoordinateFrameConventions
            # the transform shall be T_cam0_local_cs
            """
            print(type(oxts.T_w_imu))
            print(oxts.T_w_imu)
            print(oxts.T_w_imu.shape)
            print(oxts.T_w_imu[0:3, 3])
            print(inv(oxts.T_w_imu))
            exit()
            """
            if T_cam0_init_w is None:
                T_cam0_init_w = np.matmul(T_cam0_imu, inv(oxts.T_w_imu)) #inv(oxts.T_w_imu).dot(T_cam0_imu)
                transform = np.eye(4)
            else:
                transform = np.matmul(T_cam0_imu, inv(np.matmul( T_cam0_init_w, oxts.T_w_imu)))
                #transform = inv(np.matmul(T_cam0_init_w, np.matmul(oxts.T_w_imu, inv(T_cam0_imu)))) #inv(T_cam0_imu).dot(oxts.T_w_imu).dot(T_cam0_init_w)
            t = transform[0:3, 3]
            q = tf.transformations.quaternion_from_matrix(transform)
            oxts_tf = Transform()

            oxts_tf.translation.x = t[0]
            oxts_tf.translation.y = t[1]
            oxts_tf.translation.z = t[2]

            oxts_tf.rotation.x = q[0]
            oxts_tf.rotation.y = q[1]
            oxts_tf.rotation.z = q[2]
            oxts_tf.rotation.w = q[3]

            tf_oxts_transform.transform = oxts_tf
            tf_oxts_msg.transforms.append(tf_oxts_transform)

            bag.write('/tf', tf_oxts_msg, tf_oxts_msg.transforms[0].header.stamp)

    elif kitti_type.find("odom") != -1:
        timestamps = map(lambda x: initial_time + x.total_seconds(), kitti.timestamps)
        for timestamp, tf_matrix in zip(timestamps, kitti.T_w_cam0):
            tf_msg = TFMessage()
            tf_stamped = TransformStamped()
            tf_stamped.header.stamp = rospy.Time.from_sec(timestamp)
            tf_stamped.header.frame_id = '/local_cs'
            tf_stamped.child_frame_id = '/sensor/camera'  # /sensor/camera is  camera_left

            t = tf_matrix[0:3, 3]
            q = tf.transformations.quaternion_from_matrix(tf_matrix)
            transform = Transform()

            transform.translation.x = t[0]
            transform.translation.y = t[1]
            transform.translation.z = t[2]

            transform.rotation.x = q[0]
            transform.rotation.y = q[1]
            transform.rotation.z = q[2]
            transform.rotation.w = q[3]

            tf_stamped.transform = transform
            tf_msg.transforms.append(tf_stamped)

            bag.write('/tf', tf_msg, tf_msg.transforms[0].header.stamp)


def save_camera_data(bag, kitti_type, kitti, util, bridge, camera, camera_frame_id, topic, initial_time):
    print("Exporting camera {}".format(camera))
    if kitti_type.find("raw") != -1:
        camera_pad = '{0:02d}'.format(camera)
        image_dir = os.path.join(kitti.data_path, 'image_{}'.format(camera_pad))
        image_path = os.path.join(image_dir, 'data')
        image_filenames = sorted(os.listdir(image_path))
        with open(os.path.join(image_dir, 'timestamps.txt')) as f:
            image_datetimes = map(lambda x: datetime.strptime(x[:-4], '%Y-%m-%d %H:%M:%S.%f'), f.readlines())

        calib = CameraInfo()
        calib.header.frame_id = camera_frame_id
        calib.height, calib.width = tuple(util['S_rect_{}'.format(camera_pad)].tolist())
        calib.distortion_model = 'plumb_bob'
        calib.K = util['K_{}'.format(camera_pad)]
        calib.R = util['R_rect_{}'.format(camera_pad)]
        calib.D = util['D_{}'.format(camera_pad)]
        calib.P = util['P_rect_{}'.format(camera_pad)]

    elif kitti_type.find("odom") != -1:
        camera_pad = '{0:01d}'.format(camera)
        image_path = os.path.join(kitti.sequence_path, 'image_{}'.format(camera_pad))
        image_filenames = sorted(os.listdir(image_path))
        image_datetimes = map(lambda x: initial_time + x.total_seconds(), kitti.timestamps)

        calib = CameraInfo()
        calib.header.frame_id = camera_frame_id
        calib.P = util['P{}'.format(camera_pad)]

    iterable = zip(image_datetimes, image_filenames)
    bar = progressbar.ProgressBar()
    ts_init = None
    ts_0 = datetime.utcfromtimestamp(1.0)
    for dt, filename in bar(iterable):
        # print("dt ({}): {}".format(dt, filename))
        image_filename = os.path.join(image_path, filename)
        cv_image = cv2.imread(image_filename)
        calib.height, calib.width = cv_image.shape[:2]
        if camera in (0, 1):
            cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        encoding = "mono8" if camera in (0, 1) else "bgr8"
        image_message = bridge.cv2_to_imgmsg(cv_image, encoding=encoding)
        image_message.header.frame_id = camera_frame_id
        if kitti_type.find("raw") != -1:
            if ts_init is None:
                ts_init = dt
            image_message.header.stamp = rospy.Time.from_sec(float(datetime.strftime((dt - ts_init + ts_0), "%s.%f")))
            image_message.header.stamp.nsecs = round(image_message.header.stamp.nsecs / 1e6) * 1000000
            # the synced folder contains image that is already rectified. And it's not correct to call it raw
            if kitti_type.find("sync") != -1:
                topic_ext = "/image_rect"
            else:
                topic_ext = "/image_raw"
        elif kitti_type.find("odom") != -1:
            image_message.header.stamp = rospy.Time.from_sec(dt)
            topic_ext = "/image_rect"
        calib.header.stamp = image_message.header.stamp
        bag.write(topic + topic_ext, image_message, t=image_message.header.stamp)
        bag.write(topic + '/camera_info', calib, t=calib.header.stamp)


def save_velo_data(bag, kitti, velo_frame_id, topic):
    print("Exporting velodyne data")
    velo_path = os.path.join(kitti.data_path, 'velodyne_points')
    velo_data_dir = os.path.join(velo_path, 'data')
    velo_filenames = sorted(os.listdir(velo_data_dir))

    ts_init = None
    ts_0 = datetime.utcfromtimestamp(1.0)
    with open(os.path.join(velo_path, 'timestamps.txt')) as f:
        lines = f.readlines()
        velo_datetimes = []
        for line in lines:
            if len(line) == 1:
                continue
            dt = datetime.strptime(line[:-4], '%Y-%m-%d %H:%M:%S.%f')
            if (ts_init is None) and (dt is not  None):
                ts_init = dt
            velo_datetimes.append(dt)
    print("ts_init: {}".format(ts_init))

    iterable = zip(velo_datetimes, velo_filenames)
    bar = progressbar.ProgressBar()
    for dt, filename in bar(iterable):
        if dt is None:
            continue

        velo_filename = os.path.join(velo_data_dir, filename)

        # read binary data
        scan = (np.fromfile(velo_filename, dtype=np.float32)).reshape(-1, 4)

        # create header
        header = Header()
        header.frame_id = velo_frame_id
        header.stamp = rospy.Time.from_sec(float(datetime.strftime((dt - ts_init + ts_0), "%s.%f")))
        header.stamp.nsecs = round(header.stamp.nsecs / 1e6) * 1000000
        #print("header.stamp: {}".format(header.stamp))
        # fill pcl msg
        fields = [PointField('x', 0, PointField.FLOAT32, 1),
                  PointField('y', 4, PointField.FLOAT32, 1),
                  PointField('z', 8, PointField.FLOAT32, 1),
                  PointField('intensity', 12, PointField.FLOAT32, 1)]
        pcl_msg = pcl2.create_cloud(header, fields, scan)

        bag.write(topic, pcl_msg, t=pcl_msg.header.stamp)


def get_static_transform(from_frame_id, to_frame_id, transform):
    t = transform[0:3, 3]
    q = tf.transformations.quaternion_from_matrix(transform)
    tf_msg = TransformStamped()
    tf_msg.header.frame_id = from_frame_id
    tf_msg.child_frame_id = to_frame_id
    tf_msg.transform.translation.x = float(t[0])
    tf_msg.transform.translation.y = float(t[1])
    tf_msg.transform.translation.z = float(t[2])
    tf_msg.transform.rotation.x = float(q[0])
    tf_msg.transform.rotation.y = float(q[1])
    tf_msg.transform.rotation.z = float(q[2])
    tf_msg.transform.rotation.w = float(q[3])
    return tf_msg


def inv(transform):
    "Invert rigid body transformation matrix"
    R = transform[0:3, 0:3]
    t = transform[0:3, 3]
    t_inv = -1 * R.T.dot(t)
    transform_inv = np.eye(4)
    transform_inv[0:3, 0:3] = R.T
    transform_inv[0:3, 3] = t_inv
    return transform_inv


def save_static_transforms(bag, transforms, timestamps):
    print("Exporting static transformations")
    tfm = TFMessage()
    ts_0 = datetime.utcfromtimestamp(1.0)
    for transform in transforms:
        t = get_static_transform(from_frame_id=transform[0], to_frame_id=transform[1], transform=transform[2])
        tfm.transforms.append(t)
    for timestamp in timestamps:
        #print("timestamp: {}".format(timestamp))
        time = rospy.Time.from_sec(float(ts_0.strftime("%s.%f")))
        for i in range(len(tfm.transforms)):
            tfm.transforms[i].header.stamp = time
            tfm.transforms[i].header.stamp.nsecs = round(tfm.transforms[i].header.stamp.nsecs / 1e6) * 1000000
        bag.write('/tf_static', tfm, t=time)


def save_gps_fix_data(bag, kitti, gps_frame_id, topic):
    for timestamp, oxts in zip(kitti.timestamps, kitti.oxts):
        navsatfix_msg = NavSatFix()
        navsatfix_msg.header.frame_id = gps_frame_id
        navsatfix_msg.header.stamp = rospy.Time.from_sec(float(timestamp.strftime("%s.%f")))
        navsatfix_msg.latitude = oxts.packet.lat
        navsatfix_msg.longitude = oxts.packet.lon
        navsatfix_msg.altitude = oxts.packet.alt
        navsatfix_msg.status.service = 1
        bag.write(topic, navsatfix_msg, t=navsatfix_msg.header.stamp)


def save_gps_vel_data(bag, kitti, gps_frame_id, topic):
    for timestamp, oxts in zip(kitti.timestamps, kitti.oxts):
        twist_msg = TwistStamped()
        twist_msg.header.frame_id = gps_frame_id
        twist_msg.header.stamp = rospy.Time.from_sec(float(timestamp.strftime("%s.%f")))
        twist_msg.twist.linear.x = oxts.packet.vf
        twist_msg.twist.linear.y = oxts.packet.vl
        twist_msg.twist.linear.z = oxts.packet.vu
        twist_msg.twist.angular.x = oxts.packet.wf
        twist_msg.twist.angular.y = oxts.packet.wl
        twist_msg.twist.angular.z = oxts.packet.wu
        bag.write(topic, twist_msg, t=twist_msg.header.stamp)


def run_kitti2bag(kitti_type, data_dir, date, drive, sequence=None):
    drive = drive[1:]
    print("run_kitti2bag called: \nkitti_type shall be one of ['raw_synced', 'odom_color', 'odom_gray'], "
          "date shall be something like 2011_09_26\n"
          "drive shall be something like d0002\n")
    date = str(date)
    drive = str(drive)
    print("data: {} ; drive: {}".format(date, drive))
    # Accepted argument values
    kitti_types = ["raw_synced", "odom_color", "odom_gray"]
    odometry_sequences = []
    for s in range(22):
        odometry_sequences.append(str(s).zfill(2))

    bridge = CvBridge()
    compression = rosbag.Compression.NONE
    # compression = rosbag.Compression.BZ2
    # compression = rosbag.Compression.LZ4

    # CAMERAS
    cameras = [
        (0, '/sensor/camera/grayscale/left', '/sensor/camera/grayscale/left'),
        (1, '/sensor/camera/grayscale/right', '/sensor/camera/grayscale/right'),
        (2, '/sensor/camera/color/left', '/sensor/camera/color/left'),
        (3, '/sensor/camera/color/right', '/sensor/camera/color/right')
    ]

    if kitti_type.find("raw") != -1:

        if date == None:
            print("Date option is not given. It is mandatory for raw dataset.")
            print("Usage for raw dataset: kitti2bag raw_synced [dir] -t <date> -r <drive>")
            sys.exit(1)
        elif drive == None:
            print("Drive option is not given. It is mandatory for raw dataset.")
            print("Usage for raw dataset: kitti2bag raw_synced [dir] -t <date> -r <drive>")
            sys.exit(1)

        bag = rosbag.Bag("kitti_{}_drive_{}_{}.bag".format(date, drive, kitti_type[4:]), 'w', compression=compression)
        kitti = pykitti.raw(data_dir, date, drive)
        if not os.path.exists(kitti.data_path):
            print('Path {} does not exists. Exiting.'.format(kitti.data_path))
            sys.exit(1)

        if len(kitti.timestamps) == 0:
            print('Dataset is empty? Exiting.')
            sys.exit(1)

        try:
            # IMU
            imu_frame_id = 'imu_link'
            imu_topic = '/kitti/oxts/imu'
            gps_fix_topic = '/kitti/oxts/gps/fix'
            gps_vel_topic = '/kitti/oxts/gps/vel'
            velo_frame_id = '/sensor/velodyne'
            velo_topic = '/sensor/velodyne/cloud_euclidean'

            T_base_link_to_imu = np.eye(4, 4)
            T_base_link_to_imu[0:3, 3] = [-2.71 / 2.0 - 0.05, 0.32, 0.93]

            # tf_static
            # /sensor/camera (/sensor/camera/grayscale/left) -> /sensor/velodyne
            # /sensor/camera -> /sensor/camera/color/left
            # /sensor/camera -> /sensor/camera/color/right
            # /sensor/camera -> /sensor/camera/grayscale/left
            # /sensor/camera -> /sensor/camera/grayscale/left
            """
            # Below is the old code
            # I modify below for my own experiments
            transforms = [
                ('base_link', imu_frame_id, T_base_link_to_imu),
                (imu_frame_id, velo_frame_id, inv(kitti.calib.T_velo_imu)),
                (imu_frame_id, cameras[0][1], inv(kitti.calib.T_cam0_imu)),
                (imu_frame_id, cameras[1][1], inv(kitti.calib.T_cam1_imu)),
                (imu_frame_id, cameras[2][1], inv(kitti.calib.T_cam2_imu)),
                (imu_frame_id, cameras[3][1], inv(kitti.calib.T_cam3_imu))
            ]
            """
            # T_a_b means the transformation from b -> a

            """
            ['K_cam0', 'K_cam1', 'K_cam2', 'K_cam3', 'P_rect_00', 'P_rect_10', 'P_rect_20', 
            'P_rect_30', 'R_rect_00', 'R_rect_10', 'R_rect_20', 'R_rect_30', 'T_cam0_imu', 
            'T_cam0_velo', 'T_cam0_velo_unrect', 'T_cam1_imu', 'T_cam1_velo', 'T_cam2_imu', 
            'T_cam2_velo', 'T_cam3_imu', 'T_cam3_velo', 'T_velo_imu', '__add__', 
            '__class__', '__contains__', '__delattr__', '__dict__', '__doc__', 
            '__eq__', '__format__', '__ge__', '__getattribute__', '__getitem__', 
            '__getnewargs__', '__getslice__', '__getstate__', '__gt__', '__hash__',
             '__init__', '__iter__', '__le__', '__len__', '__lt__', '__module__', 
             '__mul__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', 
             '__rmul__', '__setattr__', '__sizeof__', '__slots__', '__str__', 
             '__subclasshook__', '_asdict', '_fields', '_make', '_replace', 
             'b_gray', 'b_rgb', 'count', 'index']

            """

            transforms = [
                ('/sensor/camera', '/sensor/velodyne', kitti.calib.T_cam0_velo),
                ('/sensor/camera', cameras[0][1], np.matmul(kitti.calib.T_cam0_imu, inv(kitti.calib.T_cam0_imu))),
                ('/sensor/camera', cameras[1][1], np.matmul(kitti.calib.T_cam1_imu, inv(kitti.calib.T_cam0_imu))),
                ('/sensor/camera', cameras[2][1], np.matmul(kitti.calib.T_cam2_imu, inv(kitti.calib.T_cam0_imu))),
                ('/sensor/camera', cameras[3][1], np.matmul(kitti.calib.T_cam3_imu, inv(kitti.calib.T_cam0_imu)))
            ]

            util = pykitti.utils.read_calib_file(os.path.join(kitti.calib_path, 'calib_cam_to_cam.txt'))

            # Export
            save_static_transforms(bag, transforms, kitti.timestamps)
            save_dynamic_tf(bag, kitti, kitti_type, initial_time=None, T_cam0_imu=kitti.calib.T_cam0_imu)
            #save_imu_data(bag, kitti, imu_frame_id, imu_topic)
            #save_gps_fix_data(bag, kitti, imu_frame_id, gps_fix_topic)
            #save_gps_vel_data(bag, kitti, imu_frame_id, gps_vel_topic)

            save_velo_data(bag, kitti, velo_frame_id, velo_topic)
            for camera in cameras:
                save_camera_data(bag, kitti_type, kitti, util, bridge, camera=camera[0], camera_frame_id=camera[1],
                                 topic=camera[2], initial_time=None)


        finally:
            print("## OVERVIEW ##")
            print(bag)
            bag.close()

    elif kitti_type.find("odom") != -1:

        if sequence == None:
            print("Sequence option is not given. It is mandatory for odometry dataset.")
            print("Usage for odometry dataset: kitti2bag {odom_color, odom_gray} [dir] -s <sequence>")
            sys.exit(1)

        bag = rosbag.Bag("kitti_data_odometry_{}_sequence_{}.bag".format(kitti_type[5:], sequence), 'w',
                         compression=compression)

        kitti = pykitti.odometry(data_dir, sequence)
        if not os.path.exists(kitti.sequence_path):
            print('Path {} does not exists. Exiting.'.format(kitti.sequence_path))
            sys.exit(1)

        kitti.load_calib()
        kitti.load_timestamps()

        if len(kitti.timestamps) == 0:
            print('Dataset is empty? Exiting.')
            sys.exit(1)

        if sequence in odometry_sequences[:11]:
            print("Odometry dataset sequence {} has ground truth information (poses).".format(sequence))
            kitti.load_poses()

        try:
            util = pykitti.utils.read_calib_file(os.path.join(data_dir, 'sequences', sequence, 'calib.txt'))
            current_epoch = (datetime.utcnow() - datetime(1970, 1, 1)).total_seconds()
            # Export
            if kitti_type.find("gray") != -1:
                used_cameras = cameras[:2]
            elif kitti_type.find("color") != -1:
                used_cameras = cameras[-2:]

            save_dynamic_tf(bag, kitti, kitti_type, initial_time=current_epoch)
            for camera in used_cameras:
                save_camera_data(bag, kitti_type, kitti, util, bridge, camera=camera[0], camera_frame_id=camera[1],
                                 topic=camera[2], initial_time=current_epoch)

        finally:
            print("## OVERVIEW ##")
            print(bag)
            bag.close()


if __name__ == '__main__':
    fire.Fire()
