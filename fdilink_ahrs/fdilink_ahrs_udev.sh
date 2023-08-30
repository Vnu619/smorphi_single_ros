echo "start copy fdilink.rules to /etc/udev/rules.d/"
sudo cp fdilink.rules /etc/udev/rules.d

service udev reload
sleep 2
service udev restart
echo "Finish!!!"
