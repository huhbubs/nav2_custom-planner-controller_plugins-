自定义导航控制器，一样
1.创建功能包，包含pluginlib nav2_core(重写基类)
2.定义头文件，创建cpp实现
3.添加描述文件,<library path="nav2_custom_controller"> nav2_custom_controller与add_library(${PROJECT_NAME} SHARED src/custom_controller.cpp)相同
4.编辑CMakeLists
5.编辑package.xml添加<nav2_core plugin="${prefix}/custom_controller_plugin.xml"/> 与文件名一致（其实不加也行）
6.完善函数:getNearstTargetPose/calculateAngleDifference/computeVelocityCommandss
7.修改仿真配置，插件名与<class name="nav2_custom_controller/CustomController"> 中的name参数一致