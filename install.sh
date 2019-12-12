ARM_FILE="/armStart.sh"
sudo ln -s $(dirname $(readlink -f $0))$ARM_FILE /usr/bin/armStart
chmod +x armStart.sh
echo "you can now run the program by typing: armStart"

