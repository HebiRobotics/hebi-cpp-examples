# Change directory to the current path
cd $(dirname $(readlink -f $0))/build
x-terminal-emulator -e "./hexapod_control -q"

