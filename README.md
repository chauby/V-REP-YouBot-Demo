# V-REP-YouBot-Demo
基于V-REP平台的入门级教程及Demo，从零开始学习如何在V-REP平台上进行机器人的仿真和交互。我们定了一个小目标，完成一个Demo。使用官方提供的KUKA公司的YouBot机器人模型来实验机器人的感知和控制过程，控制机器人从A点抓取物品，然后移动到B点将物品放置在B点的工作台上，这其中涉及到V-REP环境中的机器人感知和控制过程。

![image](https://github.com/chauby/V-REP-YouBot-Demo/blob/master/Kuka-YouBot.png)


本项目是在CoppeliaSim 4.1版本的软件上测试的，由于最新的软件版本是CoppeliaSim 4.2，与4.1在某些控件和函数的使用上有较大变化，因此如果是要使用本项目的代码，建议大家还是安装4.1版本的软件。


本项目包含多个YouBot机器人的控制Demo，现分别说明如下：

## 1. Demo_FirstDemo

YouBot机器人的第一个简单控制代码，可以作为入门级学习代码。



## 2. Demo_KeyboardControlMovement

使用键盘的方向键来控制YouBot机器人向“前、后、左、右”方向移动。



## 3. Demo_ChassisKinematics

构建YouBot机器人底盘的运动学模型，规划底盘在地面上的运动路径Path。

![image](https://github.com/chauby/V-REP-YouBot-Demo/blob/master/3_Demo_ChassisKinematics/YouBotPathDemo.gif)



## 4. Demo_DummyPath

构建了Dummy和Path，并让Dummy跟随Path运动。

![image](https://github.com/chauby/V-REP-YouBot-Demo/blob/master/4_Demo_DummyPath/DummyPathDemo.gif)



## 5. Demo_ArmInverseKinematicsIK

基于V-REP的逆运动学模块构建YouBot机器人机械臂的逆运动学模型，并使用dummy和path来控制机械臂按照预定轨迹运动。

![image](https://github.com/chauby/V-REP-YouBot-Demo/blob/master/5_Demo_ArmInverseKinematicsIK/ArmIKDemo.gif)



## 6. Demo_MatlabAndPythonControl

使用Matlab和Python编写控制代码远程控制V-REP中的YouBot机器人模型，可基于此Demo开发其他算法。

![image](https://github.com/chauby/V-REP-YouBot-Demo/blob/master/6_Demo_MatlabAndPythonControl/MatlabPythonVREP.gif)


## 7. Demo_youBotPickAndPlace

使用Python编写控制代码远程控制V-REP中的YouBot机器人模型完成从A点抓取物体并移动到B点的任务。

![image](https://github.com/chauby/V-REP-YouBot-Demo/blob/master/7_Demo_youBotPickAndPlace/youBotPickAndPlace.gif)



## 8. Demo_PathTracking

更新了youBot的路径跟随demo，现在可以自定义路径了。

![image](https://github.com/chauby/V-REP-YouBot-Demo/blob/master/8_Demo_youBotTrackingPath/path_tracking.gif)



---

## 文章教程

本教程将同步发布在个人微信公众号、知乎专栏和CSDN上，想要获取文章教程有以下3种方式：
1. 访问知乎专栏【AI与机器人】：https://zhuanlan.zhihu.com/c_1212783320150577152
2. 访问CSDN博客【博士的沙漏】：https://blog.csdn.net/shakehands2012
3. 关注微信公众号：博士的沙漏。

![image](https://github.com/chauby/V-REP-YouBot-Demo/blob/master/qrcode.jpg)

更多联系方式（More contact information）：
- Email: chaubyZou@163.com
- QQ: 779490568

