
#! /bin/bash
ssh -i "/home/pi/Documents/uw_capstone_key.pem" -o CheckHostIP=no -tt ubuntu@ec2-54-156-115-99.compute-1.amazonaws.com <<EOT
cd /home/capstone/capstone-slam/RPIScannerWorkspace
source ./devel/setup.bash
roslaunch processing_pkg scan_processing.launch &
sleep 5
cd /home/capstone/capstone-slam/rosnode
node /home/capstone/capstone-slam/rosnode/index.js &
EOT
