#!/bin/bash

# Change directory to the current path
cd $(dirname $(readlink -f $0))/build

#Wait until networking is active
while true; do
  if (systemctl -q is-active network-online.target)
  then
    x-terminal-emulator -e "./kits/example_mobile_io_control"
    break
  fi
  echo "Waiting for network..."
  sleep 1
done

