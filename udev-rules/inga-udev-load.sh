cp 99-inga-usb.ruls /etc/udev/rules.d/99-inga-usb.rules
udevadm control --reload rules
udevadm trigger
