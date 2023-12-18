## gps_localization
gps_localization is a localization package for GNSS device.

## 1. Prerequisites
### 1.1 **Ubuntu** and **ROS**
Ubuntu 64-bit 16.04 , 18.04 or 20.04.

### 1.2. **nmea_navsat_driver**

## 2. Build
Clone the repository and catkin_make:

```
    cd ~/catkin_ws/src
    git clone https://github.com/HM-Lii/gps_localization.git
    cd ..
    catkin_make
    source ~/catkin_ws/devel/setup.bash
```

## 3. Directly run
### 3.1 with mqtt
```
    ....
    roslaunch gps_localization gps_localization.launch 
    
```

### 3.2 without mqtt