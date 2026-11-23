# RASPBOT V2 TEST

Run the following command on RASPBOTV2 via ssh

```
#!/bin/bash
docker run -it \
--privileged=true \
--net=host \
--env="DISPLAY" \
--env="QT_X11_NO_MITSHM=1" \
-v /tmp/.X11-unix:/tmp/.X11-unix \
--security-opt apparmor:unconfined \
-v /home/pi/temp:/root/temp \
-v /dev/i2c-1:/dev/i2c-1 \
-v /dev/i2c-0:/dev/i2c-0 \
--device=/dev/video0 \
--device=/dev/video1 \
--device=/dev/gpiomem \
yahboomtechnology/ros-humble:0.1.0 /bin/bash -i -c "ros2 launch yahboomcar_bringup bringup.launch.py & ros2 run usb_cam usb_cam_node_exe"
```

then

```
cd notebooks
python ros2-agent.py
```