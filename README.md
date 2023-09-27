# Astar_with_traj_opt
## 运行效果

见B站：https://www.bilibili.com/video/BV1qF411m7mD/?spm_id_from=333.999.0.0&vd_source=62bfb7720b0b2f9941f7f34210ba6a18
## 安装求解器
需要安装casadi，C++需要源码安装casadi，casadi要调用ipopt这个非线性求解器,所以安装casadi前需要安装ipopt,我也是参考别人博客安装的，这个需要费点时间。
## 编译和运行
直接把src放入一个工作空间，终端catkin_make编译,source之后输入

```
roslaunch map_gen costmap_optimal_control.launch
```



## packages介绍

## 1 map_gen

`功能：` 生成A*需要的地图，启动launch

## 2 path_searcher

`功能：` A*搜索算法

## 3 simple_optimal_control

`功能：` 用casadi构建的后端优化
