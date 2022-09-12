

# Use these commands to actually reset the canbus 
#sudo sh -c "echo 0 > /sys/bus/usb/devices/1-4.6/authorized"
#sudo sh -c "echo 1 > /sys/bus/usb/devices/1-4.6/authorized"

for X in /sys/bus/usb/devices/*; do 
	echo "$X"
	cat "$X/manufacturer" 2>/dev/null 
	cat "$X/idVendor" 2>/dev/null 
	cat "$X/idProduct" 2>/dev/null
	sudo sh -c "echo 0 > $X/authorized"
	sudo sh -c "echo 1 > $X/authorized"

	
	echo
done
