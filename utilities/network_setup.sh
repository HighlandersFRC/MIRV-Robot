

# Delete existing route to router
#sudo route del -net 0.0.0.0 gw 172.250.250.104 netmask 0.0.0.0 dev eth2


# Change the metric on the ppp0 route to prioritize it over the router
sudo ip route replace 0.0.0.0/0 via 0.255.80.0 dev ppp0 metric 1 

