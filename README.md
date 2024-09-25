# Bootcamp

## Install simulation 

clone this repository into your workspace and install required packages:

mkdir -p ~/yourworkspace_name/src

cd ~/yourworkspace_name/src

git clone https://github.com/HMCL-UNIST/MEN491_2021.git

sudo apt-get install python3-pip 

pip3 install --upgrade --ignore-installed pip setuptools

apt-get install unixodbc-dev

pip3 install pyodbc

pip3 install llvmlite==0.34.0

cd ~/yourworkspace_name/src/MEN491_2021/f1tenth-riders-quickstart

pip3 install --user -e gym

cd ~/yourworkspace_name

catkin_make



#### If you do not have "pip"

sudo apt update 

sudo apt install python3-pip

#### if you face the "Could not find SDL" error, download the followings

sudo apt-get install libsdl-image1.2-dev

sudo apt-get install libsdl-dev
