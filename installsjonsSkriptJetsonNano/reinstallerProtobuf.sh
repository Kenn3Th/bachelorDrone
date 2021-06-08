#!/bin/bash
echo "Re installerer protobuf og google"

pip3 uninstall protobuf
pip3 uninstall google
echo "Avinstallers protobuf og google\nInstallerer google"
pip3 install google
echo "Installerer protobuf"
pip3 install protobuf
echo "Installerer google-cloud"
pip3 install google-cloud
echo "FERDIG!!"

