echo  'KERNEL=="ttyCH343USB*", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="55d4",ATTRS{serial}=="0001", MODE:="0777", GROUP:="dialout", SYMLINK+="wheeltec_N100"' >/etc/udev/rules.d/wheeltec_wheeltec.rules
service udev reload
sleep 2
service udev restart


