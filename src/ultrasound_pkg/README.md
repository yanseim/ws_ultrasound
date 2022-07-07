## worklog
---
**0522**
将kinect加入`my_stone.sdf`后
```
roslaunch franka_gazebo panda.launch x:=-0.5 world:=$(rospack find franka_gazebo)/world/my_stone.sdf controller:=cartesian_impedance_example_controller rviz:=true
```
open another terminal
```
rqt
```
---
**0525**
```
roslaunch franka_gazebo panda.launch x:=-0.5 world:=$(rospack find franka_gazebo)/world/my_stone.sdf controller:=my_cartesian_impedance_controller rviz:=true
```
添加新的controller，原本`Cmakelist.txt`,`plugin.xml`,`franka_example_controller.yaml`三处添加好就可以了，如果要gazebo仿真的话还有`franka_gazebo/config/sim_controllers.yaml`一处要添加

---

**0526**

完成经典阻抗控制代码初步完成，但是方向貌似还有点问题。

---

**0528**

他好像就应该是几何jacobian？？？因为他角度是用四元数转换而来的。

---

**0529**

interactive markers别忘了在panda.launch里面加上对应的要启动interactive markers的控制器名
```
roslaunch franka_gazebo panda.launch x:=-0.5 controller:=my_cartesian_impedance_controller rviz:=true
```
调了一组参数,在$M_d=J^{-T}(q)M(q)J^{-1}(q)$的情况下,不和外界物体接触能work.我发现抖动的情况一般是角度的`cartesian_stiffness_`和`cartesian_stiffness_`过大了,调小一点就不抖动了.**但是调小了方向调整就调不到了**.
```
  cartesian_stiffness_.block<3,3>(0,0) = 10*Eigen::MatrixXd::Identity(3, 3);
  cartesian_stiffness_.block<3,3>(3,3) = 0.8*Eigen::MatrixXd::Identity(3, 3);
  cartesian_damping_.block<3,3>(0,0) = 5*Eigen::MatrixXd::Identity(3, 3);
  cartesian_damping_.block<3,3>(3,3) = 0.4*Eigen::MatrixXd::Identity(3, 3);
  nullspace_stiffness_ = 5.0;
```

**0530**

试了原来pd+控制的参数,并利用$M_d$的选择让外力项消掉,结果直接飞了
```
  M_d = jacobian_transpose_pinv * M * jacobian_pinv;
  K_d.block<3,3>(0,0) = 200*Eigen::MatrixXd::Identity(3, 3);
  K_d.block<3,3>(3,3) = 10*Eigen::MatrixXd::Identity(3, 3);
  D_d.block<3,3>(0,0) = 28.2843*Eigen::MatrixXd::Identity(3, 3);
  D_d.block<3,3>(3,3) = 6.32455*Eigen::MatrixXd::Identity(3, 3);
  K_d = K_d * M_d;
  D_d = D_d * M_d;
```
昨天调出来的参数可以再大一点,周三再继续调,我觉得到时候需要用上Ott书上一些调参理论的公式.

**0601**
为什么`nullspace_stiffness_`会影响到末端位姿的？今天先跑了个重力补偿pd控制检查末端位姿的收敛性，发现把这个系数调小了之后，期望末端位姿就能达到了，大了就达不到。
今天还把参数改了，k_d = 2*sqrt(k_p)。
就改了这么两个地方，就通了，很神奇。

**0602**
发现之前通了是假的，我忘记把重力补偿pd控制注释掉了。

**0604**
* 完成数据的记录以及用python画图
  * 如果要让controller stop，pub这个命令`rosrun controller_manager unspawner my_cartesian_impedance_controller`
* 有一点很奇怪：`male_torso`一旦加入到my_stone.sdf中，机械臂一开始就会笔直，第4关节也不能弯。去掉就好了。不知道为什么。
* 现在还是需要先把阻抗调通啊啊啊啊.目前的问题是
  * 当把外力那一项设为0时，收敛不到期望位置，尤其是alphay方向上有很大误差
  * 当把外力那一项放出来，直接飞了。如果前面设一个0.1的系数，和外界无接触的时候能不飞，但是一旦接触上就有问题：本来应该让夹爪顶着桌面，结果夹爪直接倒了，还反复振荡。
    * 怀疑是误差的导数不能这么简单地数值作差，参考了`teleop_joint_pd_example_controller.cpp`中的`prev_alignment_error_`.但是发现他是关节空间中的误差，简单地数值差分是没问题的。笛卡尔空间的还是不清楚

**0605**
例程里的pd+控制和pd+gravity控制都是用到了转置雅可比矩阵，所以没有多大问题，但是我的阻抗控制和逆动力学控制都用到了雅可比矩阵的逆，所以有问题。先检查逆动力学控制。
```
roslaunch franka_gazebo panda.launch x:=-0.5 world:=$(rospack find franka_gazebo)/world/my_stone.sdf controller:=my_inverse_dynamic_controller rviz:=true
```
可能是gazebo有bug
https://github.com/frankaemika/franka_ros/issues/160
https://github.com/frankaemika/franka_ros/issues/197
感觉代码本身应该没有任何问题了

**0617**
* 在`my_inverse_dynamic_controller.cpp`中验证了古月居在mujoco里面定义方向误差的方式
  ```
    Eigen::Matrix<double,3,1> error_o = 0.5 * 
    (orientation_matrix.block<3,1>(0,0).cross(orientation_d_matrix_.block<3,1>(0,0)) + 
    orientation_matrix.block<3,1>(0,1).cross(orientation_d_matrix_.block<3,1>(0,1)) + 
    orientation_matrix.block<3,1>(0,2).cross(orientation_d_matrix_.block<3,1>(0,2)));
    error.tail(3) << error_o;
  ```
  没有更好。用四元数的分析雅可比的效果尚可（只是y方向的角度有点偏差），用几何雅可比矩阵的效果就非常离谱。
* 总结一下之前调的结果（不用古月居的方向误差定义方式）
  * `my_inverse_dynamic_controller.cpp`
    * 用四元数分析雅可比，在y方向和z方向的orientation都有肉眼可见的偏差
    * 用几何雅可比，只在y方向的orientation有偏差。总结下来inverse dynamic controller还是这种方式效果最好
  * `my_pd_plus_gravity_controller.cpp`
    * 只用几何雅可比，效果很好。凡是用到雅可比转置的控制器效果都很好。（比如例程`pd+`和这个`pd+g`）
  * `my_cartesian_impedance_controller.cpp`
    * 目前仿真中，如下设置的效果是最好的：
      ```
        M_d = jacobian_transpose_pinv * M * jacobian_pinv;
        tau_task << M * jacobian_pinv * M_d.inverse() * 
                        (-K_d * error - D_d * error_dot - M_d * jacobian_dot *dq)
                        +0* (jacobian.transpose() - M * jacobian_pinv * M_d.inverse()) * F_ext;
      ```
      无接触时无论是位置还是方向上的误差都很小；有接触时，各个方向上会有耦合（也就是说z方向上受力可能会影响到其他方向，这也符合理论）。但是，如果把`F_ext`那一项前面的0去掉，仿真里就会乱飞，因为`(jacobian.transpose() - M * jacobian_pinv * M_d.inverse()) * F_ext`这一项还是很大。不知道实物会怎样。

**0625**
* 完成了**末端还是gripper**，**水平桌面上**的轨迹跟踪程序`my_cartesian_impedance_traj_controller.cpp`。`F_ext`这一项仍然是没有放出来，所以阻抗模型还是有耦合。效果尚可。运行指令如下：
  ```
  roslaunch franka_gazebo panda.launch x:=-0.5 world:=$(rospack find franka_gazebo)/world/my_stone.sdf controller:=my_cartesian_impedance_traj_controller rviz:=false
  ```

**0707**
* 加上方向轨迹
  ```
  roslaunch franka_gazebo panda.launch x:=-0.5 controller:=my_cartesian_impedance_traj_controller rviz:=false
  ```