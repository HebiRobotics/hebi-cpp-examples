DAISY_FILE="/daisyStart.sh"
sudo ln -s $(dirname $(readlink -f $0))$DAISY_FILE /usr/bin/daisyStart
chmod +x daisyStart.sh
echo "you can now run the program by typing: daisyStart"

