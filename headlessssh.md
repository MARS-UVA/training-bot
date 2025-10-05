# General steps?!?

### SSH
```
- install ssh (any method)
```

`- sudo systemctl enable ssh --now` OR `- sudo systemctl enable sshd --now`

### Verify SSH
```
- sudo systemctl status ssh (verify ssh)
```

### Firewall
```
- sudo ufw disable (we did this but idk if its the right thing to do)
- sudo ufw allow ssh
```

### Verify firewall
```
- sudo ufw status
```

**Static IP configs are not necessary**

### Avahi:
```
- sudo apt install avahi-daemon avahi-utils -y
- sudo systemctl status avahi-daemon
```

### Verify avahi:
```
- ping hostname.local
- systemctl status avahi-daemon
```

### Gadget mode:
```
- echo "dtoverlay=dwc2" | sudo tee -a /boot/config.txt
- sudo sed -i 's/rootwait/rootwait modules-load=dwc2,g_ether/' /boot/cmdline.txt
```

### Verify gadget mode:
- idk tbh


### other cmd you must do
```
- sudo nmcli connection add type ethernet ifname usb0 con-name usb0 autoconnect yes
```
