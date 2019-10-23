# axis_imu

Driver to Axis IMU.

Use app file: motionloggerudp_0_6-1_mipsisa32r2el.eap

at least.


Usage:
```bash
rosrun axis_imu axis_imu _frame_id:=imu_frame _rate:=100 _root_passwd:=pass _camera_ip:=192.168.7.205 _driver_ip:=192.168.7.160 _port:=5000 _pos_enabled:=0
```

The orientation is not filled in since I do not get sensible values
from my camera. The POSITION is really the pan tilt value so should
not be used as imu rotaation.

Patch the code if you need.
