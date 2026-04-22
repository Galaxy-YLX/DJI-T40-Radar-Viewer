CAN="can0"
BITRATE=1000000
sudo ip link set $CAN type can bitrate $BITRATE
sudo ip link set $CAN up

