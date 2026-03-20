#!/bin/sh
tcpreplay -q -l 0 -i lo /halo_and_0183.pcap > /dev/null 2>&1 &
mayara-server -i lo --replay --brand navico --navigation-address udp:255.255.255.255:10110 --nmea0183
