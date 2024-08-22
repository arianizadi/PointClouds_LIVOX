#!/bin/bash
sudo ip addr del 192.168.1.50/24 dev eth0
sudo systemctl restart NetworkManager
