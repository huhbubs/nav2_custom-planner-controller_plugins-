自定义规划器
1.创建功能包，包含pluginlib nav2_core(重写基类)
2.定义头文件，创建cpp实现
3.添加描述文件
4.编辑CMakeLists
5.编辑package.xml添加<nav2_core plugin="${prefix}/custom_planner_plugin.xml"/>，custom_planner_plugin是文件名不是库名
6.完善create_path函数
7.修改仿真配置planner_server