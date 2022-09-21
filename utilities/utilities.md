# Utilities Overview

## Depth IP Setter
This script is used for changing the IP address of connected Depth Cameras. When run the script will search for available depth cameras on the network and will ask the user if they want to update the configuration for each camera. 

## Prioritise Eth1
This script is just a convienient way to run the command.
```
sudo ip route replace 0.0.0.0/0 via <ip-address> dev eth1 metric 0 
```

When run, this script sets the priority of the adapter connected to eth1 to 0. This will cause the eth1 route to take presidence over the LTE router connected to eth2. The net result of this is that the AGX will use the connection on eth1 when it would normally use eth2 LTE. This can be useful for when users want to download a large file quickly over hardwire, or when a weak LTE signal is present. Users will need to enter the IP for the eth1 adapter into this command in order to run the command.

