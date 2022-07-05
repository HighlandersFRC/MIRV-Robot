echo "Launched MIRV" > startup.log
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 1000000
sudo ifconfig can0 up
sudo ifconfig can0 txqueuelen 1000


source setenv.sh
#roslaunch mirv_real mirv.launch

function exposeDI() {
    echo $1 >/sys/class/gpio/export 2>/dev/null
    echo in >/sys/class/gpio/gpio$1/direction
}
function exposeDO() {
    echo $1 >/sys/class/gpio/export 2>/dev/null
    echo out >/sys/class/gpio/gpio$1/direction
}

exposeDI 325
exposeDI 331
exposeDI 326
exposeDO 320
exposeDO 321
exposeDO 322
exposeDO 323

