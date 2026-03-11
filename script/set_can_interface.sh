sudo ip link set can0 down
sudo ip link set can0 type can bitrate 1000000
sudo ip link set up can0
sleep 0.5
ip link show can0

echo 'CAN interface(can0) Set DONE.'
