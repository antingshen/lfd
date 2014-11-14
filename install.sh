#!/bin/sh
set -e

# cd ~

sudo echo "deb http://ppa.launchpad.net/openrave/testing/ubuntu precise main" > openrave-testing.list 
sudo echo "deb-src http://ppa.launchpad.net/openrave/testing/ubuntu precise main" >> openrave-testing.list 
sudo mv openrave-testing.list /etc/apt/sources.list.d

sudo apt-get install openrave0.9-dp openrave0.9-dp-base openrave0.9-dp-dev
sudo ln -s /usr/lib/python2.7/dist-packages/openravepy/_openravepy_0_9 /usr/lib/python2.7/dist-packages/openravepy/_openravepy_

sudo apt-get install -y libopenscenegraph-dev cmake libboost-all-dev libeigen3-dev
sudo apt-get install -y python-numpy python-networkx python-joblib
sudo apt-get install -y libpcl-1.7-all

git clone https://github.com/erictzeng/trajopt
cd trajopt
git checkout trajopt-jointopt
cd ..
git clone https://github.com/hojonathanho/bulletsim.git
cd bulletsim
git checkout lite
cd ..

mkdir -p build/trajopt
mkdir -p build/bulletsim

cd build/trajopt
cmake ../../trajopt -DBUILD_CLOUDPROC=ON
make -j

cd ../bulletsim
cmake ../../bulletsim
make -j

cd ..
export PYTHONPATH=$PYTHONPATH:$PWD/trajopt/lib:$PWD/bulletsim/lib:$PWD/../trajopt:$PWD/../bulletsim

