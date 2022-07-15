# ws_ultrasound

* 探头已写进franka的urdf中，文件位于包`franka_description`中。主要文件有文件夹`robots`中的`panda_arm.urdf.xacro`,`panda_arm.xacro`,`transducer.xacro`以及文件夹`meshes`
    ```
    roslaunch franka_gazebo panda.launch x:=-0.5 world:=$(rospack find franka_gazebo)/world/my_stone.sdf use_transducer：=true
    ```