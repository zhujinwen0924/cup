cd /sys/class/gpio/
echo hvt |sudo -S chmod 777 export
echo 393 >export
cd gpio393/
echo hvt |sudo -S chmod 777 direction value
echo out > direction
echo 0 > value
