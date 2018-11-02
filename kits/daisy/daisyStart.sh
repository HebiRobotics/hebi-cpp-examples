# Change directory to the current path
cd $(dirname $(readlink -f $0))/build
sleep 3 && x-terminal-emulator -e "./hexapod_control -q"

