# PID实验文档

### 1.基本功能实现

##### 实现过程

1. 在catkin_ws文件夹src目录中创建huangyiwen_pid功能包，并将wall_follower.cpp代码放入功能包的src目录中，修改CMakeLists.txt文件相关配置并添加功能包为可执行文件；

   ```
   catkin_make
   source devel/setup.bash
   ```

2. 启动仿真环境；

   ```
   roslaunch f1tenth_simulator simulator.launch
   ```

3. 启动PID节点；

   ```
   rosrun huangyiwen_pid wall_follower
   ```

4. 观察小车的行进状态与路线。

##### 算法逻辑

1. 发布与订阅的话题；

   ```c++
   SubscribeAndPublish() {
       drive_pub = nh.advertise<ackermann_msgs::AckermannDriveStamped>("/drive", 1000);
       scan_sub = nh.subscribe("/scan", 1000, &SubscribeAndPublish::callback, this);
   }
   ```

2. 获取激光雷达测量距离；

   ```c++
   unsigned int b_index = (unsigned int)(floor((90.0 / 180.0 * PI - lidar_info.angle_min) / lidar_info.angle_increment));
   //两条射线之间的角度
   double b_angle = 90.0 / 180.0 * PI;    
   double a_angle = 45.0 / 180.0 * PI; 
   unsigned int a_index;
   if (lidar_info.angle_min > 45.0 / 180.0 * PI) {
       a_angle = lidar_info.angle_min;
       a_index = 0;
   } else {
       a_index = (unsigned int)(floor((45.0 / 180.0 * PI - lidar_info.angle_min) / lidar_info.angle_increment));
   }            
   double a_range = 0.0;   
   double b_range = 0.0;
   if (!std::isinf(lidar_info.ranges[a_index]) && !std::isnan(lidar_info.ranges[a_index])) {   
       //得到a的长度
       a_range = lidar_info.ranges[a_index];   
   } else {
       a_range = 100.0;
   }
   if (!std::isinf(lidar_info.ranges[b_index]) && !std::isnan(lidar_info.ranges[b_index])) {
       //得到b的长度
       b_range = lidar_info.ranges[b_index];  
   } else {
       b_range = 100.0;
   }
   ```

3. 获得pid计算所需的参数；

   ```c++
   //在车的右边得到两条射线a,b来确定车到右墙的距离AB和相对于AB的方向
   double alpha = atan((a_range * cos(b_angle - a_angle) - b_range) / (a_range * sin(b_angle - a_angle))); 
   //实际离右墙的距离
   double AB = b_range * cos(alpha);   
   double projected_dis = AB + LOOK_AHEAD_DIS * sin(alpha); 
   //求出误差
   error = DESIRED_DISTANCE_RIGHT - projected_dis;  
   ROS_INFO("projected_dis = %f",projected_dis);
   ROS_INFO("error = %f",error);
   ROS_INFO("del_time = %f\n",del_time);
   SubscribeAndPublish::pid_control();
   ```

4. 实现pid控制器；

   ```c++
   void pid_control() {
       ackermann_msgs::AckermannDriveStamped ackermann_drive_result;
       double tmoment = ros::Time::now().toSec();
       //当前时刻-上一个时刻=间隔时刻
       del_time = tmoment - prev_tmoment;  
       //对误差积分，也就是误差的无限和：积分=积分+累计误差
       integral += prev_error * del_time;  
       ackermann_drive_result.drive.steering_angle = -(KP * error + KD * (error - prev_error) / del_time + KI * integral);
       //时间的迭代
       prev_tmoment = tmoment;   
       //不同情况下的速度调整，转弯时速度降低，直行时速度加快
       if (abs(ackermann_drive_result.drive.steering_angle) > 20.0 / 180.0 * PI) {
           //速度为1.3
           ackermann_drive_result.drive.speed = 2.0;
       } else if (abs(ackermann_drive_result.drive.steering_angle) > 10.0 / 180.0 * PI) {
           //速度为2.0
           ackermann_drive_result.drive.speed = 3.0;
       } else {
           //速度为3.0
           ackermann_drive_result.drive.speed = 5.0;
       }
   ```

##### 实现结果

1. 实现效果

   ![pid实现截图](C:\Users\H\Desktop\huangyiwen_pid\pid实现截图.png)

2. 录屏截屏

   <video src="C:\Users\H\Desktop\huangyiwen_pid\pid实现录屏.mp4"></video>

### 2.功能优化

##### 优化原理

1. pid计算公式为：

   ![pid原理](C:\Users\H\Desktop\huangyiwen_pid\images\pid原理.png)

   其中KP为比例增益，TI为积分时间常数，TD为微分时间常数，u(t)为控制量，e(t)为被控制量与给定值的偏差；

2. 各个参数对系统性能的影响：

   - KP：随着比例系数KP增大，小车震荡会变得更加严重，同时系统响应速度加快；
   - TI：积分作用的强弱取决于积分常数TI，TI越小，积分作用就越强；TI越大，积分作用就越弱。其中积分控制的主要作用是改善系统的稳态性能，消除系统的稳态误差。加入积分控制可使系统的相对稳定性变差，TI值的增大可能使得系统响应趋于稳态值的速度减慢。
   - TD：随着微分时间常数TD的增加，闭环系统响应的响应速度加快。其中微分环节的主要作用是提高系统的响应速度，能够在误差产生较大的变化趋势时施加合适的控制。

3. 调参结果评估：

   最大超调量：是响应曲线的最大峰值与稳态值的差，是评估系统稳定性的一个重要指标；

   上升时间：是指响应曲线从原始工作状态出发，第一次到达输出稳态值所需的时间，是评估系统快速性的一个重要指标；

   静差：是被控量的稳定值与给定值之差，一般用于衡量系统的准确性。

##### 调参过程

1. 整体思路

   调整KP、KI、KD的数值以达到系统的最高效率。

2. 具体过程

   - KP=500，KI=0，KD=0响应曲线为：

     ![调参尝试1](C:\Users\H\Desktop\huangyiwen_pid\images\调参尝试1.png)

     其中，KP值调整较大，响应曲线出现了震荡。

   - KP=500，KI=0，KD=400响应曲线为：

     ![调参尝试2](C:\Users\H\Desktop\huangyiwen_pid\images\调参尝试2.png)

     其中，KP值调整较大，在首次比例控制较强的情况下，增大微分控制的作用，可以观察到响应曲线震荡次数变小，但是由于微分控制KD较大，使得系统响应时间变长。

   - KP=50，KI=0，KD=0响应曲线为：

     ![调参尝试3](C:\Users\H\Desktop\huangyiwen_pid\images\调参尝试3.png)

     其中，KP值调整较小，响应曲线没有出现震荡，但是系统响应时间明显变慢了。

   - KP=120，KI=0.1，KD=500响应曲线为：

     ![调参尝试4](C:\Users\H\Desktop\huangyiwen_pid\images\调参尝试4.png)

     其中，因为系统响应时间要求并不严格，因此调小比例控制值KP；同时因为KP值调整较小，因此加入了积分控制减小静差；同时加大系统的阻尼，即调大微分参数KD以防止超调。

### 3.算法提高

##### 算法原理

1. 首次实现的pid控制器所使用的算法是数字PID控制型控制算法，具体原理为：

   ![数字PID控制型控制算法](C:\Users\H\Desktop\huangyiwen_pid\images\数字PID控制型控制算法.png)

2. 使用增量型控制算法进行算法提高，具体原理为：

![算术优化](C:\Users\H\Desktop\huangyiwen_pid\images\算术优化.png)

![增量型控制算法](C:\Users\H\Desktop\huangyiwen_pid\images\增量型控制算法.png)

##### 代码实现

```c++
int Incremental_PI (int Encoder,int Target)
{ 	
    static float Bias,Pwm,Last_bias;
    //计算偏差
    Bias=Encoder-Target;
    //增量式PI控制器
    Pwm+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias;
    //保存上一次偏差 
    Last_bias=Bias;	    
    //增量输出
    return Pwm;                                           
}
```

##### 实验结果对比

1. 对比

   在使用位置控制型控制算法运行程序时，观察到小车在沿墙行进过程中经常会发生车身摇摆不稳定，在多次启动时部分启动运行中小车在转弯时会撞到墙壁的现象；而在使用增量型控制算法时，在多次启动时小车行进过程中车身摇摆有明显减少，并且没有出现小车撞到墙壁的现象。

2. 结论

   增量型控制算法不需要做累加，控制量增量仅仅与最近几次误差采样值有关，计算误差或计算精度问题，对控制量的计算影响较小。而位置控制算法要用到过去的误差的累加值，容易产生大的累加误差。其中增量型算法得出的是控制量的增量，在一次动作执行中只输出变化部分，误动作影响小，必要时通过逻辑判断限制或禁止本次的输出；而位置型控制算法输出的是控制量的全量输出，误动作影响大。

