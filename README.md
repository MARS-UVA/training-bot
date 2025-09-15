This repository is for the training bots


# Setup for wired connection
1. Download rndis Drivers folder from [drive](https://drive.google.com/drive/folders/1iG192wNeSPvb9UlEwjJTGAYXjVNH1b7i?usp=drive_link)

2. Plug in raspberry pi to laptop via usb-c port

3. Open Device Manager

4. Scroll down to Ports (COM & LPT)

5. Find Pi's USB port (Usually called COM3 or similar, if you unplug the pi it disappears)

6. Right click on the port, and select update driver> browse my computer for drivers> Select the driver folder you downloaded in step 1





command to build docker image:
```
docker build --platform=linux/arm64/v8 -t image_name:tag .
```

docker save -o filename.tar image_name:tag

scp filename.tar mars@larry.local:~/Downloads



ssh mars@larry.local

ON PI
docker load -i ~/Downloads/filename.tar

to run
docker run -it --rm --device=/dev/input/event4 --device=/dev/ttyACM0 image_name:tag
                                                                     training-bot-base:pi-1.0