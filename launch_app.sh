#!/bin/bash

sudo pkill gst

echo 'Reset v4l2loopback device'
sudo modprobe v4l2loopback -r

if [ ! -c /dev/video0 ]; then
    echo "Did not recognized video device at /dev/video0"
	sudo modprobe v4l2loopback video_nr=0,10 card_label="DUMMY","TEST"
	echo "Stream video to device"
	gst-launch-1.0 -v videotestsrc pattern=0 ! video/x-raw,width=200,height=200 ! videoconvert ! tee ! v4l2sink device=/dev/video0 &
else
	echo "Recognized video device at /dev/video0"
    sudo modprobe v4l2loopback video_nr=10 card_label="TEST"
    videoOutput=$(timeout 5 cat /dev/video0 | read -n 1) # try fetch value froooom video device for 5 seconds
    if [ -z "$videoOutput" ]; then
        gst-launch-1.0 -v videotestsrc pattern=0 ! video/x-raw,width=200,height=200 ! videoconvert ! tee ! v4l2sink device=/dev/video0 &
    fi

fi

sudo docker-compose -f video_record/docker-compose.yaml -f view_rosbag/docker-compose.yaml up -d --force-recreate

# View video inTerminal using: "DISPLAY= cvlc v4l2:///dev/video0"
# In order to kill video run: Ctrl+q
# If running as sudo is necessary, run video as: "sudo DISPLAY= vlc-wrapper v4l2:///dev/video0"
# In order to kill video in sudo mode: "sudo killall vlc"