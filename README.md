## 安装方式

clone本仓库代码到ros工作空间中

编译

```bash
catkin_make
source devel/setup.sh
```

运行

```bash
roslaunch astar_path_planner astar_planner.launch
```

当编译无报错、rviz显示随机生成圆柱障碍物和绿色直线初始路径时表明编译安装成功，其中绿色显示的是A*算法规划的离散折线轨迹，在rviz可视化界面中选择添加marker，选择其中的smooth_trajectory，即可看到平滑处理后的蓝色曲线。
