#!/bin/bash

sudo echo -e "[Unit] \nDescription=CVT60 service \nAfter=multi-user.target \n\n[Service] \nType=simple \nExecStart=/usr/bin/python3 /home/pi/cvt60/cvt60daemon.py \nUser=pi \nWorkingDirectory=/home/pi/cvt60 \nRestart=always \n\n[Install] \nWantedBy=multi-user.target" >> /etc/systemd/system/cvt60.service
sudo chmod +x /home/pi/cvt60/cvt60daemon.py
sudo systemctl enable cvt60.service
sudo systemctl daemon-reload
sudo systemctl start cvt60.service
sudo systemctl status cvt60.service
