run-application:
	- sudo modprobe v4l2loopback -r
	- sudo modprobe v4l2loopback video_nr=10 card_label="TEST"
	bash -c "cd view_rosbag; sudo make run-compose;"