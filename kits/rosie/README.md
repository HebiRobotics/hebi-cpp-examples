# Instructions

## Rosie - Build and Setup Instructions :

1. Turn on both mounted batteries and power the NUC and actuator modules.  
2. Connect your laptop to the Rosie's network (HEBI_Rosie) that is hosted by the internal router and SSH into the NUC (credentials mentioned on Rosie).  
  `ssh hebi@10.10.1.2`  
3. Ensure the required dependencies are installed  
  `sudo apt install git cmake build-essential`  
4. Clone hebi-cpp-examples repo and Create a build directory    
  `git clone --recurse-submodules https://github.com/HebiRobotics/hebi-cpp-examples.git`  
  `cd hebi-cpp-examples`  
  `mkdir build`  
  `cd build`  
5. Configure the build environment using cmake and compile the code  
  `cmake ../projects/cmake`  
  `make`  
6. Power On and Configure the Mobile IO Device  
  `Family = HEBI`  
  `Name = mobileIO`  
7. Navigate to the folder containing the built executable and launch it:  
  `cd kits/rosie`  
  `./rosie_demo`  


## Rosie - Startup Configuration :

1. Create and edit a new systemd service (rosie.service) :  
  `sudo nano /etc/systemd/system/rosie.service`  
2. Copy the following contents to specify how the executable should be run on startup:  
		
  `[Unit]`  
  `Description=HEBI Rosie C++ Demo`  
  `After=network-online.target`  
  `[Service]`  
  `ExecStart=/bin/bash -c "cd /home/hebi/hebi-cpp-examples/build/kits/rosie; ./rosie_demo"`  
  `User=hebi`  
  `Type=simple`  
  `Restart=always`  
  `[Install]`  
  `WantedBy=default.target`  
		
3. Reload systemd to recognize the new service  
  `sudo systemctl daemon-reexec`  
         `sudo systemctl daemon-reload`  
4. Ensure the Mobile IO device is powered on, then enable and launch the service:  
  `sudo systemctl enable rosie.service`  
  `sudo systemctl start rosie.service`  
5. Verify the service is running correctly:  
  `sudo systemctl status rosie.service`  
  `journalctl -u rosie.service`  
6. On rebooting the NUC, the Rosie demo should automatically start execution.  
