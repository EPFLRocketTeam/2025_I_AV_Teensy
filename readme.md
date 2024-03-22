## For Ubuntu

```
sudo apt update
sudo apt upgrade
sudo apt install -y wget
```

```
wget https://apt.kitware.com/kitware-archive.sh
sudo bash kitware-archive.sh
```

```
sudo apt install --no-install-recommends git cmake ninja-build gperf \
  ccache dfu-util device-tree-compiler wget \
  python3-dev python3-pip python3-setuptools python3-tk python3-wheel xz-utils file \
  make gcc gcc-multilib g++-multilib libsdl2-dev 
```


```
python3 -m venv ~/zephyrproject/.venv
source ~/zephyrproject/.venv/bin/activate
pip install west
```

```
west init ~/zephyrproject
cd ~/zephyrproject
pip install -r ~/zephyrproject/zephyr/scripts/requirements.txt
git clone https://github.com/EPFLRocketTeam/2024_I_AV_SW_TEENSY 2024_I_AV_SW_TEENSY
west update
west build -b teensy41 2024_I_AV_SW_TEENSY
```