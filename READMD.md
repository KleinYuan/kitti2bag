# kitti2bag


### Step1 Download Data

Download raw data from KITTI, for example

```
$ wget https://s3.eu-central-1.amazonaws.com/avg-kitti/raw_data/2011_09_26_drive_0093/2011_09_26_drive_0093_sync.zip
$ wget https://s3.eu-central-1.amazonaws.com/avg-kitti/raw_data/2011_09_26_calib.zip
$ unzip 2011_09_26_drive_0093_sync.zip
$ unzip 2011_09_26_calib.zip
```


Then you shall see your folder structured as:

```
├── 2011_09_26_drive_0093_sync
│   ├── image_00
│   │   ├── data
│   │   └── timestamps.txt
│   ├── image_01
│   │   ├── data
│   │   └── timestamps.txt
│   ├── image_02
│   │   ├── data
│   │   └── timestamps.txt
│   ├── image_03
│   │   ├── data
│   │   └── timestamps.txt
│   ├── oxts
│   │   ├── data
│   │   ├── dataformat.txt
│   │   └── timestamps.txt
│   └── velodyne_points
│       ├── data
│       ├── timestamps_end.txt
│       ├── timestamps_start.txt
│       └── timestamps.txt
├── calib_cam_to_cam.txt
├── calib_imu_to_velo.txt
└── calib_velo_to_cam.txt
```

### Step2 Build Docker Image


First of all, clone this repo:

```
git clone git@github.com:KleinYuan/kitti2bag.git
```

Then, build the docker image:

```
docker build -t kitti2bag .
```


### Step3 Spin up a Container and EXE Shell

Imagine you download your data under `/foo/`, then create a folder named as `${year}_${month}_${day}`.

Taking the `2011_09_26_drive_0093_sync` above as an exmaple, you shall put those files under: `/foo/2011_09_26`

```
docker run -it  \
 -v /foo/2011_09_26:/data \
 -v ${PWD}/kitti2bag:/kitti2bag/kitti2bag \
 kitti2bag -v bash
```

And then you will be in the shell and if you do `ls`, you shall see:

```
root@8d9c20c69c15:/data# ls
2011_09_26
```

### Step4 Run the Conversion

```
source "/opt/ros/$ROS_DISTRO/setup.bash"
kitti2bag -t 2011_09_26 -r 0093 raw_synced .
```

And you shall see the following logs:
```
Exporting static transformations
Exporting time dependent transformations
Exporting IMU
Exporting camera 0
100% (77 of 77) |##########################| Elapsed Time: 0:00:00 Time: 0:00:00
Exporting camera 1
100% (77 of 77) |##########################| Elapsed Time: 0:00:00 Time: 0:00:00
Exporting camera 2
100% (77 of 77) |##########################| Elapsed Time: 0:00:01 Time: 0:00:01
Exporting camera 3
100% (77 of 77) |##########################| Elapsed Time: 0:00:01 Time: 0:00:01
Exporting velodyne data
100% (77 of 77) |##########################| Elapsed Time: 0:00:15 Time: 0:00:15
## OVERVIEW ##
path:        kitti_2011_09_26_drive_0002_synced.bag
version:     2.0
duration:    7.8s
start:       Sep 26 2011 13:02:44.33 (1317042164.33)
end:         Sep 26 2011 13:02:52.16 (1317042172.16)
size:        417.2 MB
messages:    1078
compression: none [308/308 chunks]
types:       geometry_msgs/TwistStamped [98d34b0043a2093cf9d9345ab6eef12e]
             sensor_msgs/CameraInfo     [c9a58c1b0b154e0e6da7578cb991d214]
             sensor_msgs/Image          [060021388200f6f0f447d0fcd9c64743]
             sensor_msgs/Imu            [6a62c6daae103f4ff57a132d6f95cec2]
             sensor_msgs/NavSatFix      [2d3a8cd499b9b4a0249fb98fd05cfa48]
             sensor_msgs/PointCloud2    [1158d486dd51d683ce2f1be655c3c181]
             tf2_msgs/TFMessage         [94810edda583a504dfda3829e70d7eec]
topics:      /kitti/camera_color_left/camera_info    77 msgs    : sensor_msgs/CameraInfo    
             /kitti/camera_color_left/image_raw      77 msgs    : sensor_msgs/Image         
             /kitti/camera_color_right/camera_info   77 msgs    : sensor_msgs/CameraInfo    
             /kitti/camera_color_right/image_raw     77 msgs    : sensor_msgs/Image         
             /kitti/camera_gray_left/camera_info     77 msgs    : sensor_msgs/CameraInfo    
             /kitti/camera_gray_left/image_raw       77 msgs    : sensor_msgs/Image         
             /kitti/camera_gray_right/camera_info    77 msgs    : sensor_msgs/CameraInfo    
             /kitti/camera_gray_right/image_raw      77 msgs    : sensor_msgs/Image         
             /kitti/oxts/gps/fix                     77 msgs    : sensor_msgs/NavSatFix     
             /kitti/oxts/gps/vel                     77 msgs    : geometry_msgs/TwistStamped
             /kitti/oxts/imu                         77 msgs    : sensor_msgs/Imu           
             /kitti/velo/pointcloud                  77 msgs    : sensor_msgs/PointCloud2   
             /tf                                     77 msgs    : tf2_msgs/TFMessage        
             /tf_static                              77 msgs    : tf2_msgs/TFMessage
```

And then you will see a bag named as `kitti_2011_09_26_drive_0093_synced.bag` lying there.



### Step5 Change the Code

Sometimes, you wanna change the rule of the conversion, namely, editing the code in [kitti2bag.py](kitti2bag/kitti2bag.py),
then you can just edit it and then run

```
pip install -e /kitti2bag
```

And do the conversion again!
