### TODO.md

For functionality parity with `radar_pi`:

* EBL/VRM support in GUI
* Timed Transmit

Bugs:

* Check doppler packets sent when no chartplotter present and disallow doppler status when
  no heading is on radar spokes.
* Furuno brand support needs more work. (-> Dirk)
* Garmin HD needs PCAP file and testing.
* Garmin xHD needs testing, mostly testing of setting controls.
* 

For parity with branch `v2`: 

* Re-implement the radar recording and playback. 
* Re-implement the debugger. Or a better one?
