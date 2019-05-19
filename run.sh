#!/bin/bash

# Create v4l2loopback device
sudo odprobe v4l2loopback -r
sudo modprobe v4l2loopback video_nr=10 card_label="TEST"

# Stream video to device
gst-launch-1.0 -v videotestsrc pattern=0 ! video/x-raw,width=200,height=200 ! videoconvert ! tee ! v4l2sink device=/dev/video10
# View video inTerminal using: "DISPLAY= cvlc v4l2:///dev/video10"
# In order to kill video run: Ctrl+q
# If running as sudo is necessary, run video as: "sudo DISPLAY= vlc-wrapper v4l2:///dev/video10"
# In order to kill video in sudo mode: "sudo killall vlc"
