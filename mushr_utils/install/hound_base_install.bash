# wget -qO - https://download.sublimetext.com/sublimehq-pub.gpg | gpg --dearmor | sudo tee /etc/apt/trusted.gpg.d/sublimehq-archive.gpg > /dev/null
# echo "deb https://download.sublimetext.com/ apt/stable/" | sudo tee /etc/apt/sources.list.d/sublime-text.list
# distribution=$(. /etc/os-release;echo $ID$VERSION_ID)       && curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg       && curl -s -L https://nvidia.github.io/libnvidia-container/experimental/$distribution/libnvidia-container.list |          sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' |          sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list

# sudo apt-get -y update
# sudo apt-get -y install ca-certificates curl gnupg lsb-release pip sublime-text htop

# curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg echo   "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.gpg] https://download.docker.com/linux/ubuntu $(lsb_release -cs) stable" | sudo tee /etc/apt/sources.list.d/docker.list > /dev/null

# sudo mkdir -m 0755 -p /etc/apt/keyrings

# sudo apt-get -y install nvidia-jetpack
# sudo usermod -aG docker $USER
# sudo pip install jetson-stats
# sudo reboot now
# sudo apt-get -y install nvidia-container-toolkit
# sudo apt -y --fix-broken install

# sudo apt -y install docker-compose-plugin

# sudo cp ~/catkin_ws/src/mushr/daemon.json /etc/docker/daemon.json
# ## add mushr_start.service to services (where?)
# sudo cp ~/catkin_ws/src/mushr/mushr_utils/install/mushr_start.service /etc/systemd/system
# ## this does not "enable the service", that requires another command 
# git clone -b orin-docker https://github.com/sidtalia/jetson-containers.git
# sudo ./scripts/docker_build_jetpack.sh
# sudo ./scripts/docker_build_ml.sh pytorch

wget -qO - https://download.sublimetext.com/sublimehq-pub.gpg | gpg --dearmor | sudo tee /etc/apt/trusted.gpg.d/sublimehq-archive.gpg > /dev/null
echo "deb https://download.sublimetext.com/ apt/stable/" | sudo tee /etc/apt/sources.list.d/sublime-text.list distribution=$(. /etc/os-release;echo $ID$VERSION_ID)
sudo apt-get -y update
sudo apt-get -y install ca-certificates curl gnupg lsb-release pip sublime-text htop
sudo usermod -aG docker $USER
sudo pip install jetson-stats
sudo cp ~/catkin_ws/src/mushr/daemon.json /etc/docker/daemon.json
## add mushr_start.service to services (where?)
sudo cp ~/catkin_ws/src/mushr/mushr_utils/install/mushr_start.service /etc/systemd/system
## this does not "enable the service", that requires another command
cd /home/hound
git clone -b orin_docker https://github.com/sidtalia/jetson-containers.git
cd jetson-containers
sudo ./scripts/docker_build_jetpack.sh
sudo ./scripts/docker_build_ml.sh pytorch
# sudo reboot now