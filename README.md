### 下载代码

```sh
sudo apt install git
git clone https://github.com/gaoxiang12/slambook
#安装cmake
sudo apt install cmake
#安装kdevelop
sudo apt install kdevelop
sudo apt install kdevelop-l10n
```

### 源码安装cmake

[下载](https://cmake.org/download/)

```sh
#编译安装
tar -xzvf cmake-3.14.5.tar.gz
cd cmake-3.14.5
./bootstrap
make
sudo make install
```

### 安装Eigen

```sh
sudo apt install libeigen3-dev
```

### 安装Pangolin

https://www.cnblogs.com/newneul/p/8270137.html

### 安装Sophus

```sh
#不能install，所以不能删除源码
git clone https://github.com/strasdat/Sophus
cd Sophus/
#安装非模板的Sophus，执行下面一句
git checkout a621ff
mkdir build
cd build
cmake ..
make
```

### 安装OpenCV

```sh
#安装依赖
sudo apt-get install build-essential libgtk2.0-dev libvtk5-dev libjpeg-dev libtiff5-dev libjasper-dev libopenexr-dev libtbb-dev
```

[安装包](https://opencv.org/releases/)，标准cmake安装（安装3）

### 安装点云库PCL

**apt安装不能用！**

```sh
sudo add-apt-repository ppa:v-launchpad-jochen-sprickerhof-de/pcl
#update出错可以忽略
sudo apt-get update
sudo apt-get install libpcl-dev
```

**源码安装：**[教程](https://blog.csdn.net/dantengc/article/details/78446600)

```sh
#依赖
sudo apt-get update
sudo apt-get install git build-essential linux-libc-dev
sudo apt-get install cmake cmake-gui 
sudo apt-get install libusb-1.0-0-dev libusb-dev libudev-dev
sudo apt-get install mpi-default-dev openmpi-bin openmpi-common
sudo apt-get install libflann1.8 libflann-dev
sudo apt-get install libeigen3-dev
sudo apt-get install libboost-all-dev
sudo apt-get install libvtk5.10-qt4 libvtk5.10 libvtk5-dev
sudo apt-get install libqhull* libgtest-dev
sudo apt-get install freeglut3-dev pkg-config
sudo apt-get install libxmu-dev libxi-dev
sudo apt-get install mono-complete
sudo apt-get install qt-sdk openjdk-8-jdk openjdk-8-jre
```

```sh
#源码
git clone https://github.com/PointCloudLibrary/pcl.git
#1.9有问题，安装1.8
https://github.com/PointCloudLibrary/pcl/releases/tag/pcl-1.8.1
```

```sh
#编译
cd pcl
mkdir release
cd release
cmake -DCMAKE_BUILD_TYPE=None -DCMAKE_INSTALL_PREFIX=/usr -DBUILD_GPU=ON -DBUILD_apps=ON -DBUILD_examples=ON -DCMAKE_INSTALL_PREFIX=/usr ..
make
sudo make install
```

### 安装 Ceres

```sh
#依赖
sudo apt-get install liblapack-dev libsuitesparse-dev libcxsparse3.1.4 libgflags-dev libgoogle-glog-dev libgtest-dev
```

```sh
#编译
git clone https://github.com/ceres-solver/ceres-solver
#cmake编译并install
```

### 安装g2o

```sh
sudo apt-get install libqt4-dev
sudo apt-get install qt4-qmake
sudo apt-get install libqglviewer-dev
git clone https://github.com/RainerKuemmerle/g2o
cd g2o
mkdir build
cd build
cmake ..
make
sudo make install
sudo ldconfig
```

### TUM数据集

[下载](https://vision.in.tum.de/data/datasets/rgbd-dataset/download)

```sh
#生成rgb和depth关联的文件
python associate.py rgb.txt depth.txt >associate.txt
```

### *因子图优化：GTSAM

```sh
sudo apt-get install libtbb-dev
git clone https://github.com/borglab/gtsam
#cmake安装
#注意：使用时会报错，没解决
```

### BoW词袋库：DBoW3

```sh
git clone https://github.com/rmsalinas/DBow3
#cmake安装
```

### OctMap

```sh
sudo apt-get install libqglviewer-dev-qt4
git clone https://github.com/OctoMap/octomap
#cmake安装
sudo apt-get install libqglviewer-dev
#第一步把它替换掉了，现在重新装上
```
