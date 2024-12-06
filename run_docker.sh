docker run \
	-it \
	--rm \
	--net=host \
	--ipc=host \
	--privileged \
	-v /boot:/boot \
	-v /home/analog/ros2_ws:/root/ros2_ws \
	-v /home/Workspace:/root/Workspace \
	adtf31xx
