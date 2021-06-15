#!/bin/bash
echo "Reinstallerer protobuf og google"
echo "Avinstallers protobuf og google"
sudo pip3 uninstall protobuf
sudo pip3 uninstall google
echo "Installerer google og protobuf"
sudo pip3 install google
echo "Installerer protobuf"
sudo pip3 install protobuf
echo "Installerer google-cloud"
sudo pip3 install google-cloud
echo "FERDIG!!"

