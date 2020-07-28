# MASS(Muscle-Actuated Skeletal System)

![Teaser](png/Teaser.png)
## Abstract

This code implements a basic simulation and control for full-body **Musculoskeletal** system. Skeletal movements are driven by the actuation of the muscles, coordinated by activation levels. Interfacing with python and pytorch, it is available to use Deep Reinforcement Learning(DRL) algorithm such as Proximal Policy Optimization(PPO).

## Publications

Seunghwan Lee, Kyoungmin Lee, Moonseok Park, and Jehee Lee 
Scalable Muscle-actuated Human Simulation and Control, 
ACM Transactions on Graphics (SIGGRAPH 2019), Volume 37, Article 73. 

Project Page : http://mrl.snu.ac.kr/research/ProjectScalable/Page.htm

Youtube : https://youtu.be/a3jfyJ9JVeM

Paper : http://mrl.snu.ac.kr/research/ProjectScalable/Paper.pdf

## How to install

Highly recommended Ubuntu 18.04 or above.
MacOS is also supported (use homebrew for installing libraries).

### Install Boost, TinyXML, Eigen, OpenGL, FLTK, assimp, Python3, etc...

```bash
sudo apt-get install libboost-python-dev libtinyxml-dev libeigen3-dev libxi-dev libxmu-dev freeglut3-dev libfltk1.3-dev libassimp-dev libpython3-dev python3-tk python3-numpy virtualenv ipython3 cmake-curses-gui
```

### Install DART

Please refer to http://dartsim.github.io/

Manual from DART(http://dartsim.github.io/install_dart_on_ubuntu.html)
1. install required dependencies

```bash
sudo apt-get install build-essential cmake pkg-config git
sudo apt-get install libeigen3-dev libassimp-dev libccd-dev libfcl-dev libboost-regex-dev libboost-system-dev
sudo apt-get install libopenscenegraph-dev
```
2. install DART

```bash
git clone git://github.com/dartsim/dart.git
cd dart
mkdir build
cd build
cmake ..
make -j4
sudo make install
```

### Install PIP things

You should first activate virtualenv.
```bash
virtualenv /path/to/venv --python=python3
source /path/to/venv/bin/activate
```
- pytorch(https://pytorch.org/)

```bash
pip3 install torch torchvision
```

- numpy, matplotlib

```bash
pip3 install numpy matplotlib ipython
```

## How to compile and run

### Resource

Our system require a reference motion to imitate. We provide sample references such as walking, running, and etc... 

To learn and simulate, we should provide such a meta data. We provide default meta data in /data/metadata.txt. We parse the text and set the environment. Please note that the learning settings and the test settings should be equal.(metadata.txt should not be changed.)


### Compile and Run

```bash
mkdir build
cd build
cmake .. 
make -j8
```

- Run Training
```bash
cd python
source /path/to/virtualenv/
python3 main.py -d ../data/metadata.txt
```

All the training networks are saved in /nn folder.

- Run UI
```bash
source /path/to/virtualenv/
./render/render ../data/metadata.txt
```

- Run Trained data
```bash
source /path/to/virtualenv/
./render/render ../data/metadata.txt ../nn/xxx.pt ../nn/xxx_muscle.pt
```

If you are simulating with the torque-actuated model, 
```bash
source /path/to/virtualenv/
./render/render ../data/metadata.txt ../nn/xxx.pt
```
