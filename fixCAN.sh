sudo ip link set can0 down
sudo ip link set can0 type can bitrate 1000000
sudo ifconfig can0 up
sudo ifconfig can0 txqueuelen 1000