### project内容

- [x] 0.1

  基本数据结构框架：包括camera、frame、map、mappoint、config。只是定义了这些类，没有使用他们，所以没有执行文件。

- [x] 0.2

  加入了visual_odometry类，其中使用RANSAC求解PnP，用于计算帧间位姿变换，采用TUM的rgbd_dataset_freiburg1_xyz数据集，跟踪相机的运动。

- [x] 0.3

  加入了g2o_types类，用于构建g2o的边：computeError()计算误差、linearizeOplus()计算观测方程的雅克比。本project中g2o的边为一元边，使重投影误差最小，从而优化帧间位姿。

- [ ] 

  