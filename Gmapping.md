---
typora-copy-images-to: markdown-Image
---

# Gmapping

最近在阅读 Gmapping 源码，做了笔记、注释，如下

Github 地址

Gitee 地址

B站 Up

Csdn 地址

## 概览

论文地址

> 说明：分别于 2005 年、2007 年发布了
>
>  2007 :[Improved Techniques for Grid Mapping With Rao-Blackwellized Particle Filters (researchgate.net)](https://www.researchgate.net/publication/3450390_Improved_Techniques_for_Grid_Mapping_With_Rao-Blackwellized_Particle_Filters)

源码地址

> [GitHub - ros-perception/slam_gmapping: http://www.ros.org/wiki/slam_gmapping](https://github.com/ros-perception/slam_gmapping)
>
> [GitHub - ros-perception/openslam_gmapping](https://github.com/ros-perception/openslam_gmapping)

注释代码地址：

特点

> + 基于粒子滤波
> + 深度信息（2D激光雷达或深度摄像机）
> + IMU信息
> + 里程计信息
> + 二维栅格地图
> + RBPF（Rao-Blackwellized Particle Filters）

优点

> + 小场景计算量少
> + 对激光雷达扫描频率要求较低

缺点

> + 内存大、计算量大（粒子滤波通病）
> + 不适合大场景

ROS 中使用

> + 话题
>   + 订阅
>     + scan 激光雷达扫描数据
>     + tf 用于激光雷达坐标系、基坐标系、里程计坐标系之间转换
>   + 发布
>     + map 地图栅格数据
>     + map_metadata 地图Meta数据
> + 服务
>   + dynamic_map，获取地图数据
> + 坐标系名称
>   + base_link：机器人基坐标系，可以理解成是机器人的中心（质心）
>   + map：地图坐标系
>   + odom：里程计坐标系
>   + laser：雷达坐标系
> + tf坐标变换
>   + laser->base_link
>     激光雷达坐标系与基坐标系之间的变换，静态坐标变换
>   + base_link->odom
>     地图坐标系与机器人里程计坐标系之间的变换，也就是告诉机器人，现在在地图的姿态（可以理解成“头朝向哪个方向”）
>   + map->odom
>     地图坐标系与机器人里程计坐标系之间的变换，告诉机器人现在在地图上哪个位置

解读

> 基于粒子滤波的配准工作，创新点：建议分布、选择性重采样
>
> 总结来说：贡献是，改进了粒子滤波在某些传感器上的表现

## 源码分段阅读

### 逻辑

大致框架梳理，然后一句一句读代码

![1663225315026](.\markdown-Image\1663225315026.png)

配合以下解说食用
Gmapping 的程序框架依托 Open_slam ，该框架主要分成 slam_gmapping 和 openslam_gmapping 

在 `slam_gmapping` `mian.cpp()` 的中初始化`ros`节点，调用`startLiveSlam()`先定义几个发布者、订阅者、服务器、TF 进而进入回调函数`laserCallback()`，`initMapper()`初始化地图，`addScan()`添加激光扫描数据，`updateMap()`更新地图

### 一句一句读代码

#### launch

`ros`语句  

```bash
rosrun gmapping slam_gmapping
```

这句话运行了`launch`文件夹下的`launch_gmapping_pr2.launch`文件，如下

```xml
<launch>
    <param name="use_sim_time" value="true"/>
    <node pkg="gmapping" type="slam_gmapping" name="slam_gmapping" output="screen">
      <remap from="scan" to="base_scan"/>
      <param name="map_update_interval" value="5.0"/>
      <param name="maxUrange" value="16.0"/>
      <param name="sigma" value="0.05"/>
      <param name="kernelSize" value="1"/>
      <param name="lstep" value="0.05"/>
      <param name="astep" value="0.05"/>
      <param name="iterations" value="5"/>
      <param name="lsigma" value="0.075"/>
      <param name="ogain" value="3.0"/>
      <param name="lskip" value="0"/>
      <param name="srr" value="0.1"/>
      <param name="srt" value="0.2"/>
      <param name="str" value="0.1"/>
      <param name="stt" value="0.2"/>
      <param name="linearUpdate" value="1.0"/>
      <param name="angularUpdate" value="0.5"/>
      <param name="temporalUpdate" value="3.0"/>
      <param name="resampleThreshold" value="0.5"/>
      <param name="particles" value="30"/>
      <param name="xmin" value="-50.0"/>
      <param name="ymin" value="-50.0"/>
      <param name="xmax" value="50.0"/>
      <param name="ymax" value="50.0"/>
      <param name="delta" value="0.05"/>
      <param name="llsamplerange" value="0.01"/>
      <param name="llsamplestep" value="0.01"/>
      <param name="lasamplerange" value="0.005"/>
      <param name="lasamplestep" value="0.005"/>
    </node>
</launch>
```

其中

> <param>设置了很多参数，后面会用到
>
> <node>打开了`gmapping`功能包下的`slam_gmapping`可执行文件，并将其命名为`slam_gmapping`
>
> <remap>将`scan`话题改名为`scan_base`，所以在开启节点时，执行以下语句`rosrun gmapping slam_gmapping scan:=base_scan`，来监听`scan`的消息

#### main()

上一步中执行了`slam_gmapping`可执行文件，而在 CMakeLists.txt 中

```bash
add_executable(slam_gmapping src/slam_gmapping.cpp src/main.cpp)
```

要求这个可执行文件来自 `main.cpp()` 和 `slam_gmapping.cpp()`

于是我们从 `main.cpp()` 进入

```cpp
//头文件申明函数来自哪里
#include <ros/ros.h> 

#include "slam_gmapping.h"

int
main(int argc, char** argv)
{
  ros::init(argc, argv, "slam_gmapping"); //初始化 ros 节点

  SlamGMapping gn; //声明一个 SlamGMapping 类，这里进入构造函数（下方做说明）
  gn.startLiveSlam(); //进入函数 startLiveSlam() 下方做说明
  ros::spin(); //回旋函数 进入循环处理回调 

  return(0);
}
```

#### 构造函数 SlamGMapping()

```cpp
SlamGMapping::SlamGMapping():
  map_to_odom_(tf::Transform(tf::createQuaternionFromRPY( 0, 0, 0 ), tf::Point(0, 0, 0 ))),
  laser_count_(0), private_nh_("~"), scan_filter_sub_(NULL), scan_filter_(NULL), transform_thread_(NULL)
{
  seed_ = time(NULL); //高斯分布随机数种子
  init(); //进入初始化函数
}

//一些参数的声明，目前都还没说明它的作用，在后面才会分配，在后面介绍
//map_to_odom_ 世界到里程计坐标转换器，存储变换关系的变量，等会儿用到要专门讲
//laser_count_ 在laserCallback()中限流
//private_nh_ ros句柄，在startLiveSlam()被定义为参数服务器
//scan_filter_sub_  消息过滤器，message_filters::Subscriber
//scan_filter_  tf下的消息过滤器，tf::MessageFilter
//transform_thread_ 线程 publishLoop

// seed_ 定义了高斯分布的随机数种子
//我们下一步进入 init() 函数
```

##### tf::MessageFilter 与 message_filters::Subscriber

相关文档

> [tf: tf::MessageFilter< M > Class Template Reference (ros.org)](http://docs.ros.org/en/api/tf/html/c++/classtf_1_1MessageFilter.html)
>
> [tf/Tutorials/Using Stamped datatypes with tf::MessageFilter - ROS Wiki](http://wiki.ros.org/tf/Tutorials/Using Stamped datatypes with tf::MessageFilter)
>
> [message_filters - ROS Wiki](http://wiki.ros.org/message_filters)

message_filters::Subscriber 

> 1. message_filters
>
> message_filters 集合了许多消息“过滤”算法，作用类似于消息缓存（当消息到达消息过滤器的时候，可能并不会立即输出，而是在稍后的时间点里满足一定条件下输出）
>
> 统一了接口：message_filters中所有的消息过滤器都遵循着相同的模式连接输入和输出
>
> 输入：connectInput()方法链接
>
> 输出：registerCallback()方法链接
>
> 2. message_filters::Subscriber 
>
> message_filters 对 ROS订阅 的封装，将 ROS 主题作为输入
>
> 3. 示例
>
> ```cpp
> message_filters::Subscriber<std_msgs::UInt32> sub(nh, "my_topic", 1);
> sub.registerCallback(myCallback);
> ```

tf::MessageFilter

> 是 tf 下的消息过滤器，继承于 message_filters::Subscriber 作用：缓存、消息订阅、坐标转换

需要的头文件

```cpp
#include "ros/ros.h"
#include "tf/transform_listener.h"
#include "tf/message_filter.h"
#include "message_filters/subscriber.h"
```

二者结合应用实例：

```cpp
//一般使用结构
/*
1. 定义数据：TransformListener、message_filters::Subscriber、tf::MessageFilter
2.用消息的名称来初始化message_filters::Subscriber
3.用tf、message_filters::Subscriber、目标坐标系来初始化tf::MessageFilter
4.给tf::MessageFilter注册callback。
编写callback，并在回调中完成坐标转换。至此完成消息订阅+坐标转换
*/

TransformListener tfl_;
message_filters::Subscriber<sensor_msgs::LaserScan> laser_sub_;
tf::MessageFilter<sensor_msgs::LaserScan> laser_notifier_;
 
laser_sub_(nh_, "scan", 10)
laser_notifier_(laser_sub_, tfl_, fixed_frame, 10)
 
laser_notifier_.registerCallback(boost::bind(&LegDetector::laserCallback, this, _1))
laser_notifier_.setTolerance(ros::Duration(0.01));
 
void laserCallback(const sensor_msgs::LaserScan::ConstPtr& scan){
    tfl_.transformPoint(fixed_frame, loc, loc);
}
```

#### init()

涉及到很多参数初始值设置，当然也可以在`launch`文件配置，后面使用的时候再说

```cpp
void SlamGMapping::init()
{
  // log4cxx::Logger::getLogger(ROSCONSOLE_DEFAULT_NAME)->setLevel(ros::console::g_level_lookup[ros::console::levels::Debug]);

  // The library is pretty chatty
  //gsp_ = new GMapping::GridSlamProcessor(std::cerr);
  //tf 坐标变换广播器-->在 publishTransform() 函数中用来广播地图 
  gsp_ = new GMapping::GridSlamProcessor();
  ROS_ASSERT(gsp_); //判断是否创建指针成功

  tfB_ = new tf::TransformBroadcaster();
  ROS_ASSERT(tfB_);//判断是否创建指针成功

  //声明两个指针
  gsp_laser_ = NULL;
  gsp_odom_ = NULL;

  //声明两个布尔量
  got_first_scan_ = false;
  got_map_ = false;
  
  // Parameters used by our GMapping wrapper
  //读取和配置参数
  if(!private_nh_.getParam("throttle_scans", throttle_scans_))
    throttle_scans_ = 1;
  if(!private_nh_.getParam("base_frame", base_frame_))
    base_frame_ = "base_link";
  if(!private_nh_.getParam("map_frame", map_frame_))
    map_frame_ = "map";
  if(!private_nh_.getParam("odom_frame", odom_frame_))
    odom_frame_ = "odom";

  //参数服务器，参数依次指 名称，值，默认值
  private_nh_.param("transform_publish_period", transform_publish_period_, 0.05);

  //两次更新之间的时间间隔阈值（至少达到这个时间长度才可以更新）
  double tmp;
  if(!private_nh_.getParam("map_update_interval", tmp))
    tmp = 5.0;
  //Duration 时间管理
  map_update_interval_.fromSec(tmp);
  
  // Parameters used by GMapping itself
  maxUrange_ = 0.0;  maxRange_ = 0.0; // preliminary default, will be set in initMapper()
  if(!private_nh_.getParam("minimumScore", minimum_score_))
    minimum_score_ = 0;
  if(!private_nh_.getParam("sigma", sigma_))
    sigma_ = 0.05;
  if(!private_nh_.getParam("kernelSize", kernelSize_))
    kernelSize_ = 1;
  if(!private_nh_.getParam("lstep", lstep_))
    lstep_ = 0.05;
  if(!private_nh_.getParam("astep", astep_))
    astep_ = 0.05;
  if(!private_nh_.getParam("iterations", iterations_))
    iterations_ = 5;
  if(!private_nh_.getParam("lsigma", lsigma_))
    lsigma_ = 0.075;
  if(!private_nh_.getParam("ogain", ogain_))
    ogain_ = 3.0;
  if(!private_nh_.getParam("lskip", lskip_))
    lskip_ = 0;
  if(!private_nh_.getParam("srr", srr_))
    srr_ = 0.1;
  if(!private_nh_.getParam("srt", srt_))
    srt_ = 0.2;
  if(!private_nh_.getParam("str", str_))
    str_ = 0.1;
  if(!private_nh_.getParam("stt", stt_))
    stt_ = 0.2;
  if(!private_nh_.getParam("linearUpdate", linearUpdate_))
    linearUpdate_ = 1.0;
  if(!private_nh_.getParam("angularUpdate", angularUpdate_))
    angularUpdate_ = 0.5;
  if(!private_nh_.getParam("temporalUpdate", temporalUpdate_))
    temporalUpdate_ = -1.0;
  if(!private_nh_.getParam("resampleThreshold", resampleThreshold_))
    resampleThreshold_ = 0.5;
  if(!private_nh_.getParam("particles", particles_))
    particles_ = 30;
  if(!private_nh_.getParam("xmin", xmin_))
    xmin_ = -100.0;
  if(!private_nh_.getParam("ymin", ymin_))
    ymin_ = -100.0;
  if(!private_nh_.getParam("xmax", xmax_))
    xmax_ = 100.0;
  if(!private_nh_.getParam("ymax", ymax_))
    ymax_ = 100.0;
  if(!private_nh_.getParam("delta", delta_))
    delta_ = 0.05;
  if(!private_nh_.getParam("occ_thresh", occ_thresh_))
    occ_thresh_ = 0.25;
  if(!private_nh_.getParam("llsamplerange", llsamplerange_))
    llsamplerange_ = 0.01;
  if(!private_nh_.getParam("llsamplestep", llsamplestep_))
    llsamplestep_ = 0.01;
  if(!private_nh_.getParam("lasamplerange", lasamplerange_))
    lasamplerange_ = 0.005;
  if(!private_nh_.getParam("lasamplestep", lasamplestep_))
    lasamplestep_ = 0.005;
    
    //
  if(!private_nh_.getParam("tf_delay", tf_delay_))
    tf_delay_ = transform_publish_period_;

}
```

#### startLiveSlam()

接着，构造完`gn`以后，执行`mian()`中下一句，`gn.startLiveSlam();`

涉及到的数据类型

> [nav_msgs/OccupancyGrid Documentation (ros.org)](http://docs.ros.org/en/api/nav_msgs/html/msg/OccupancyGrid.html)
>
> [nav_msgs/MapMetaData Documentation (ros.org)](http://docs.ros.org/en/api/nav_msgs/html/msg/MapMetaData.html)

```cpp
void SlamGMapping::startLiveSlam()
{
  //定义 3 个发布器，发布数据，分别是  
  //entropy 计算位姿的信息熵
  //map 地图(栅格地图占用率) header map_metadata data
  //map_metadata 地图信息 map_load_time width height origin(geometry_msgs/Pose) resolution(分辨率)
  entropy_publisher_ = private_nh_.advertise<std_msgs::Float64>("entropy", 1, true);
  sst_ = node_.advertise<nav_msgs::OccupancyGrid>("map", 1, true);
  sstm_ = node_.advertise<nav_msgs::MapMetaData>("map_metadata", 1, true);
  //节点服务器
  //mapCallback 是 service 的回调函数，留给使用者定义客户端
  ss_ = node_.advertiseService("dynamic_map", &SlamGMapping::mapCallback, this);
  //监听激光雷达的数据
  scan_filter_sub_ = new message_filters::Subscriber<sensor_msgs::LaserScan>(node_, "scan", 5);
  //odom_frame_ 是 target_frame_id
  //在能进行数据转换的情况下。就可以调用回调函数
  scan_filter_ = new tf::MessageFilter<sensor_msgs::LaserScan>(*scan_filter_sub_, tf_, odom_frame_, 5);
  //注册回调函数 laserCallback
  scan_filter_->registerCallback(boost::bind(&SlamGMapping::laserCallback, this, _1));
  //以成员函数 publishLoop 作为线程启动，参数为　transform_publish_period,默认　0.05
  //boost::bind 函数绑定器绑定了函数 publishLoop()
  transform_thread_ = new boost::thread(boost::bind(&SlamGMapping::publishLoop, this, transform_publish_period_));
}
```

##### mapCallback()

```cpp
//判断地图是否存在，及简单判断其是否合法
bool 
SlamGMapping::mapCallback(nav_msgs::GetMap::Request  &req,
                          nav_msgs::GetMap::Response &res)
{
  boost::mutex::scoped_lock map_lock (map_mutex_); //地图锁上
  if(got_map_ && map_.map.info.width && map_.map.info.height)
  {
    res = map_;
    return true;
  }
  else
    return false;
}

//got_map_ 就是在init()里面定义的 2 布尔值之一
```

##### publishLoop()

```cpp
void SlamGMapping::publishLoop(double transform_publish_period){
  //在 init() 里面指定的参数，0.05
  //这里先判断也是为了防止下一步 “除以0” 数学错误导致的崩溃
  if(transform_publish_period == 0)
    return;
	
    //设置频率-->也就是下面 r.sleep(); 的睡眠时间
  ros::Rate r(1.0 / transform_publish_period);
  while(ros::ok()){
    publishTransform();
    r.sleep();
  }
}
```

##### publishTransform()

```cpp
void SlamGMapping::publishTransform()
{
  map_to_odom_mutex_.lock(); //地图
  //tf_expiration 广播tf使用的时间戳定义时间戳
  ros::Time tf_expiration = ros::Time::now() + ros::Duration(tf_delay_);
  // 构造函数里定义的广播，　发布map和odom之间的变换
  //map_frame_ 父坐标系名称
  //base_link 子坐标系的名称
  tfB_->sendTransform( tf::StampedTransform (map_to_odom_, tf_expiration, map_frame_, map_frame_));
  map_to_odom_mutex_.unlock();
}

//tf::StampedTransform () 是 ros里代表TF变换的数据类型
//下面简要梳理一下，这个逻辑
```

##### tf(Transform Frame)变换

1. 在构造函数`SlamGMapping()`里，定义了`map_to_odom_ `存储了转换关系

   > 有 6 个自由度
   >
   > x,y,z 子坐标系在父坐标系下的位置
   >
   > RPY 子坐标系在父坐标系下的偏转角

2. 在初始化函数`init()`中，读取参数使`map_frame_ `=`map`,`odom_frame_`=`odom`

3. 接着在`publishTransform()`函数中定义事件，当前ros时间+tf_delay_也就是transform_publish_period时间

4. 广播坐标变换

   > tf::StampedTransform ()的参数
   >
   > map_to_odom_ 坐标变换关系
   >
   > tf_expiration 时间戳
   >
   > map_frame_ 父坐标系的名称
   >
   > map_frame_  子坐标系的名称

#### laserCallback()

```cpp
SlamGMapping::laserCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{   //回调函数

  laser_count_++; //记录
    /*每隔throttle_scans_ （默认值 1）帧数据计算一次，限流作用*/
    //throttle_scans_ 是 init() 中参数设置的
  if ((laser_count_ % throttle_scans_) != 0)//判断是否丢掉
    return;

  //last_map_update 在是否需要Update Map时需要，仅当两帧之间时间大于阈值时，才更新地图
  static ros::Time last_map_update(0,0);

  // We can't initialize the mapper until we've got the first scan
  //直到第一帧扫描时，初始化地图
  if(!got_first_scan_)//这是 init 里面初始化的 bool 值之一
  {
      //1.进行参数初始化
      //将 slam 里的参数传递到 openslam 里 ，设定坐标系、坐标原点、采样函数随机种子的初始化、调用了GridSlamProcessor::init : 初始化了粒子数、子地图大小
    if(!initMapper(*scan))
      return;
    got_first_scan_ = true;//之后不能再初始化地图
  }

  GMapping::OrientedPoint odom_pose; //定义里程计位姿信息

  if(addScan(*scan, odom_pose)) //2.调用openslam_gmappinig核心算法 添加激光扫描数据
  {
    ROS_DEBUG("scan processed");//输出目前运行地方
    // 从粒子集合中挑选最优的粒子， 以其地图坐标为基准， 计算从激光雷达到地图之间的坐标变换
    GMapping::OrientedPoint mpose = gsp_->getParticles()[gsp_->getBestParticleIndex()].pose;
    ROS_DEBUG("new best pose: %.3f %.3f %.3f", mpose.x, mpose.y, mpose.theta);
    ROS_DEBUG("odom pose: %.3f %.3f %.3f", odom_pose.x, odom_pose.y, odom_pose.theta);
    ROS_DEBUG("correction: %.3f %.3f %.3f", mpose.x - odom_pose.x, mpose.y - odom_pose.y, mpose.theta - odom_pose.theta);

    tf::Transform laser_to_map = tf::Transform(tf::createQuaternionFromRPY(0, 0, mpose.theta), tf::Vector3(mpose.x, mpose.y, 0.0)).inverse();
    tf::Transform odom_to_laser = tf::Transform(tf::createQuaternionFromRPY(0, 0, odom_pose.theta), tf::Vector3(odom_pose.x, odom_pose.y, 0.0));

    // 另一处的lock在 SlamGMapping::publishTransform()， 这里是获得map_to_odom_，前者是发布
    map_to_odom_mutex_.lock();
    map_to_odom_ = (odom_to_laser * laser_to_map).inverse(); //计算世界到里程计坐标
    map_to_odom_mutex_.unlock();

    // 如果没有地图则直接更新。 如果有地图了，两次扫描时间差大于参数，才更新地图
    if(!got_map_ || (scan->header.stamp - last_map_update) > map_update_interval_)
    {
      updateMap(*scan); //3.得到最优例子中的地图数据，利用栅格地图算法，更新地图
      last_map_update = scan->header.stamp; //更新上一桢时间戳
      ROS_DEBUG("Updated the map");
    }
  } else
    ROS_DEBUG("cannot process scan");
}
```

#### initMapper()

```cpp
bool
SlamGMapping::initMapper(const sensor_msgs::LaserScan& scan)
{
  //总作用：
  //1.判断激光雷达是否是水平放置的，如果不是 则报错
  //2.假设激光雷达数据的角度是对称的 & 递增的 为每个激光束分配角度
  //3.为gmapping算法设置各种需要的参数

  //frame_id：表征发布的数据是来自哪一个坐标系
  laser_frame_ = scan.header.frame_id;
  // Get the laser's pose, relative to base.
  // 雷达坐标系原点在base_link中的位置
  tf::Stamped<tf::Pose> ident;//定义位姿
  //TF的一些数据结构：(Quaternion, Vector, Point, Pose, Transform)
  //（四元数、3*1向量、点坐标、位姿<包括坐标及方向>、转换模板）
  //Stamped：继承自某一个基本数据类型，并增加了2个字段stamp_ frame_id_
  tf::Stamped<tf::Transform> laser_pose;//定义激光雷达位姿信息
  ident.setIdentity();//设为单位矩阵
  ident.frame_id_ = laser_frame_; //坐标系 ID
  ident.stamp_ = scan.header.stamp; //时间戳
  try
  {
    // 输出laser_pose，雷达坐标系原点在base_link中的位置
    //函数定义：void transformPose(const std::string &target_frame, const geometry_msgs::PoseStamped &stamped_in, geometry_msgs::PoseStamped &stamped_out) const
    //参数说明：将 ident 转换到 base_frame_ 坐标系下，结果是 laser_pose
    tf_.transformPose(base_frame_, ident, laser_pose);
  }
  // 作用：如果请求的坐标系id之间存在连接，但一个或多个变换已过期，则抛出。
  catch(tf::TransformException e)
  {
    ROS_WARN("Failed to compute laser pose, aborting initialization (%s)",
             e.what());
    return false;
  }

  // create a point 1m above the laser position and transform it into the laser-frame
  // 在激光位置（base_link）上方1米处创建一个点，并将其转换为激光帧
  tf::Vector3 v;
  v.setValue(0, 0, 1 + laser_pose.getOrigin().z());
  //数据说明：http://wiki.ros.org/tf/Overview/Data%20Types
  tf::Stamped<tf::Vector3> up(v, scan.header.stamp,base_frame_);

  try
  {
    //参数说明：将 up 转换到 laser_frame_ 坐标系下，结果是 up
    tf_.transformPoint(laser_frame_, up, up);
    ROS_DEBUG("Z-Axis in sensor frame: %.3f", up.z());
  }
  catch(tf::TransformException& e)
  {
    ROS_WARN("Unable to determine orientation of laser: %s",
             e.what());
    return false;
  }
  
  // gmapping doesnt take roll or pitch into account. So check for correct sensor alignment.
  // 这里是判断雷达是否倾斜    如果倾斜就不是标准的1
  if (fabs(fabs(up.z()) - 1) > 0.001)
  {
    ROS_WARN("Laser has to be mounted planar! Z-coordinate has to be 1 or -1, but gave: %.5f",
                 up.z());
    return false;
  }

  //扇面需要分成多少束
  gsp_laser_beam_count_ = scan.ranges.size();
  // 扇形扫描面的中心
  double angle_center = (scan.angle_min + scan.angle_max)/2;
  // 雷达是否正装
  if (up.z() > 0)
  {
    do_reverse_range_ = scan.angle_min > scan.angle_max;
    // laser坐标系中的点(0,0,0), 方向(0,0,angle_center)
    // 将中心点作为原点
    centered_laser_pose_ = tf::Stamped<tf::Pose>(tf::Transform(tf::createQuaternionFromRPY(0,0,angle_center),
                                                               tf::Vector3(0,0,0)), ros::Time::now(), laser_frame_);
    ROS_INFO("Laser is mounted upwards.");
  }
  else
  {
    do_reverse_range_ = scan.angle_min < scan.angle_max;
    centered_laser_pose_ = tf::Stamped<tf::Pose>(tf::Transform(tf::createQuaternionFromRPY(M_PI,0,-angle_center),
                                                               tf::Vector3(0,0,0)), ros::Time::now(), laser_frame_);
    ROS_INFO("Laser is mounted upside down.");
  }

  // Compute the angles of the laser from -x to x, basically symmetric and in increasing order
  laser_angles_.resize(scan.ranges.size());
  // Make sure angles are started so that they are centered
  // 把角度调成对称，也就是扇形要关于ｚ轴对称，扫描角度从-3.124～3.142变为-3.133～3.133
  double theta = - std::fabs(scan.angle_min - scan.angle_max)/2;
  for(unsigned int i=0; i<scan.ranges.size(); ++i)
  {
    laser_angles_[i]=theta;
    theta += std::fabs(scan.angle_increment);
  }

  ROS_DEBUG("Laser angles in laser-frame: min: %.3f max: %.3f inc: %.3f", scan.angle_min, scan.angle_max,
            scan.angle_increment);
  ROS_DEBUG("Laser angles in top-down centered laser-frame: min: %.3f max: %.3f inc: %.3f", laser_angles_.front(),
            laser_angles_.back(), std::fabs(scan.angle_increment));

  // 这个是 Gmapping 算法接受的点类型
  GMapping::OrientedPoint gmap_pose(0, 0, 0);

  // setting maxRange and maxUrange here so we can set a reasonable default
  // 设置激光雷达的最大观测距离和最大使用距离
  ros::NodeHandle private_nh_("~");
  if(!private_nh_.getParam("maxRange", maxRange_))
    maxRange_ = scan.range_max - 0.01;
  if(!private_nh_.getParam("maxUrange", maxUrange_))
    maxUrange_ = maxRange_;

  // The laser must be called "FLASER".
  // We pass in the absolute value of the computed angle increment, on the
  // assumption that GMapping requires a positive angle increment.  If the
  // actual increment is negative, we'll swap the order of ranges before
  // feeding each scan to GMapping.
  // 激光的名称必须是"FLASER"
  // 传入计算的angle increment绝对值, 因为gmapping接受正的angle increment
  // 根据上面得到的激光雷达的数据信息， 初始化一个激光传感器
  gsp_laser_ = new GMapping::RangeSensor("FLASER",
                                         gsp_laser_beam_count_,//多少束
                                         fabs(scan.angle_increment),//一束多少度
                                         gmap_pose,//初始位姿
                                         0.0,
                                         maxRange_);//最大长度

  ROS_ASSERT(gsp_laser_);//判断是否创建指针成功

  //为gmapping算法设置sensormap
  GMapping::SensorMap smap;// std::map<std::string, Sensor*>
  // make_pair将元素组成结构体
  smap.insert(make_pair(gsp_laser_->getName(), gsp_laser_));// getName()就是 FLASER
  gsp_->setSensorMap(smap);

  //初始化里程计传感器
  gsp_odom_ = new GMapping::OdometrySensor(odom_frame_);
  ROS_ASSERT(gsp_odom_);//判断是否创建指针成功


  /// @todo Expose setting an initial pose
  //得到里程计的初始位姿
  GMapping::OrientedPoint initialPose;
  if(!getOdomPose(initialPose, scan.header.stamp))
  {
    ROS_WARN("Unable to determine inital pose of laser! Starting point will be set to zero.");
    // 如果没有，则把初始位姿设置为(0,0,0)
    initialPose = GMapping::OrientedPoint(0.0, 0.0, 0.0);
  }
  // 以下全是参数的设置，纯粹变量赋值，不多不少
  gsp_->setMatchingParameters(maxUrange_, maxRange_, sigma_,
                              kernelSize_, lstep_, astep_, iterations_,
                              lsigma_, ogain_, lskip_);

  gsp_->setMotionModelParameters(srr_, srt_, str_, stt_);
  gsp_->setUpdateDistances(linearUpdate_, angularUpdate_, resampleThreshold_);
  gsp_->setUpdatePeriod(temporalUpdate_);
  gsp_->setgenerateMap(false);
  gsp_->GridSlamProcessor::init(particles_, xmin_, ymin_, xmax_, ymax_,
                                delta_, initialPose);
  gsp_->setllsamplerange(llsamplerange_);
  gsp_->setllsamplestep(llsamplestep_);
  /// @todo Check these calls; in the gmapping gui, they use
  /// llsamplestep and llsamplerange intead of lasamplestep and
  /// lasamplerange.  It was probably a typo, but who knows.
  gsp_->setlasamplerange(lasamplerange_);
  gsp_->setlasamplestep(lasamplestep_);
  gsp_->setminimumScore(minimum_score_);

  // Call the sampling function once to set the seed.
  // 生成1作为方差，均值为0的高斯分布
  //seed_是随机数种子，保证每次的高斯分布不同
  GMapping::sampleGaussian(1,seed_);

  ROS_INFO("Initialization complete");

  return true;
}
```

##### RangeSensor()

```cpp
// slam_gmapping : initMapper 方法调用
RangeSensor::RangeSensor(std::string name, unsigned int beams_num, double res, const OrientedPoint& position, double span, double maxrange):Sensor(name),
	m_pose(position), m_beams(beams_num)
{
	double angle=-.5*res*beams_num;
	for (unsigned int i=0; i<beams_num; i++, angle+=res){
		RangeSensor::Beam& beam(m_beams[i]);
		beam.span=span;
		beam.pose.x=0;
		beam.pose.y=0;
		beam.pose.theta=angle;
		beam.maxRange=maxrange;
	}
	newFormat=0;
	updateBeamsLookup();
}
```

##### OdometrySensor()

```cpp
// slam_gmapping : initMapper 方法调用
OdometrySensor::OdometrySensor(const std::string& name, bool ideal): Sensor(name){ m_ideal=ideal;}
```

#### addScan()

```cpp
// 加入一个激光雷达的数据, 里面会调用 processScan()函数
// gmap_pose为刚定义的里程计的累积位姿
bool
SlamGMapping::addScan(const sensor_msgs::LaserScan& scan, GMapping::OrientedPoint& gmap_pose)
{
    //得到与激光的时间戳相对应的机器人的里程计的位姿
  if(!getOdomPose(gmap_pose, scan.header.stamp))
     return false;
    //检测是否所有帧的数量都是相等的，如果不相等就return
    //即确保和上一帧激光帧数数量相等
  if(scan.ranges.size() != gsp_laser_beam_count_)
    return false;

  // GMapping wants an array of doubles...
  //ranges_double 数组存储每一帧的距离
  double* ranges_double = new double[scan.ranges.size()];
  // If the angle increment is negative, we have to invert the order of the readings.
  // 如果激光是反着装的，这激光的顺序需要反过来（下面解释一个）
  //同时这里会排除掉所有激光距离小于 range_min 的值
  //如果小于，则令其举例等于range_max
  if (do_reverse_range_)
  {
    ROS_DEBUG("Inverting scan");
    int num_ranges = scan.ranges.size();
    for(int i=0; i < num_ranges; i++)
    {
      // Must filter out short readings, because the mapper won't
      if(scan.ranges[num_ranges - i - 1] < scan.range_min)
        ranges_double[i] = (double)scan.range_max;
      else
        ranges_double[i] = (double)scan.ranges[num_ranges - i - 1];
    }
  } else 
  {
    for(unsigned int i=0; i < scan.ranges.size(); i++)
    {
      // Must filter out short readings, because the mapper won't
      if(scan.ranges[i] < scan.range_min)
        ranges_double[i] = (double)scan.range_max;
      else
        ranges_double[i] = (double)scan.ranges[i];
    }
  }

  // 把 ROS 的激光雷达数据信息 转换为 GMapping算法看得懂的形式
  // 激光传感器gsp_laser_ 在initMapper里定义，ranges数据赋值给m_dists，size给m_beams
  GMapping::RangeReading reading(scan.ranges.size(),
                                 ranges_double,
                                 gsp_laser_,
                                 scan.header.stamp.toSec());

  // ...but it deep copies them in RangeReading constructor, so we don't
  // need to keep our array around.
  // 内存释放：上面的初始化是进行深拷贝
  delete[] ranges_double;

  //设置和激光数据的时间戳匹配的机器人的位姿
  reading.setPose(gmap_pose);

  /*
  ROS_DEBUG("scanpose (%.3f): %.3f %.3f %.3f\n",
            scan.header.stamp.toSec(),
            gmap_pose.x,
            gmap_pose.y,
            gmap_pose.theta);
            */
  ROS_DEBUG("processing scan");//输出状态信息
    //调用gmapping算法进行处理读到的激光一帧各个分角度的数据
  return gsp_->processScan(reading);
}
```

##### getOdomPose()

```cpp
bool
SlamGMapping::getOdomPose(GMapping::OrientedPoint& gmap_pose, const ros::Time& t)
{
  //函数功能：输入时间戳，返回对应的里程计位姿
  // Get the pose of the centered laser at the right time
  centered_laser_pose_.stamp_ = t;
  // Get the laser's pose that is centered
  // 得到centered_laser_pose_ 在odom坐标系中的坐标 odom_pose
  tf::Stamped<tf::Transform> odom_pose;
  try
  {
    //将 centered_laser_pose_ 转换到 odom_frame_ 中，记录为 odom_pose
    tf_.transformPose(odom_frame_, centered_laser_pose_, odom_pose);
  }
  catch(tf::TransformException e)
  {
    ROS_WARN("Failed to compute odom pose, skipping scan (%s)", e.what());
    return false;
  }
  double yaw = tf::getYaw(odom_pose.getRotation());

  gmap_pose = GMapping::OrientedPoint(odom_pose.getOrigin().x(),
                                      odom_pose.getOrigin().y(),
                                      yaw);
  return true;
}
```

#### processScan()

思路：

1. 运动模型中采样 `drawFromMotion()`
2.  计算移动距离：`m_linearDistance`, `m_angularDistance`
   + 保证平移距离不过大，过大则认为位置发生了突变
   + 当移动距离过大，或是长时间没有更新，那么就需要进行一次配准
3. scanMatch
   + plainReading（也就是点云原始数据）进行配准，配准使用搜索算法
   + 

```cpp
// slam_gmapping : addScan 方法调用
bool GridSlamProcessor::processScan(const RangeReading & reading, int adaptParticles){

	/**retireve the position from the reading, and compute the odometry*/
    //当前雷达在里程计坐标系的位置  addScan最后的setPose
	OrientedPoint relPose=reading.getPose();
    //m_count表示这个函数被调用的次数，开始为0,如果是第0次调用,则所有的位姿都是一样的
    if (!m_count){
		m_lastPartPose=m_odoPose=relPose;
	}

    //更新t时刻粒子群（在模型中加入高斯噪声）
    //write the state of the reading and update all the particles using the motion model
	for (ParticleVector::iterator it=m_particles.begin(); it!=m_particles.end(); it++){
        OrientedPoint& pose(it->pose); // 上一时刻粒子的位姿
        // relPose是里程计记录的最新位姿， m_odoPose是建图引擎记录的上一时刻的位姿
        // pose就是it->pose加了噪声，由于上面是引用，所以m_particles容器的成员pose全更新为pose
        pose=m_motionModel.drawFromMotion(it->pose, relPose, m_odoPose);
	}
	//invoke the callback
	onOdometryUpdate(); //回调函数，实际上什么也没做 QAQ

    //根据两次里程计的数据,计算机器人的线性位移和角度位移的累积值
    //m_odoPose表示上一次的里程计位姿   relPose表示新的里程计的位姿
	// accumulate the robot translation and rotation
	OrientedPoint move=relPose-m_odoPose;
	move.theta=atan2(sin(move.theta), cos(move.theta));

    //统计机器人在进行激光雷达更新之前， 走了多远的距离  以及　平移了多少的角度
    //注意：这两个变量最后还是要清零
    m_linearDistance+=sqrt(move*move);
	m_angularDistance+=fabs(move.theta);

	// if the robot jumps throw a warning
        /*
         如果机器人在走了m_distanceThresholdCheck这么远的距离都没有进行激光雷达的更新
         则需要进行报警。这个误差很可能是里程计或者激光雷达的BUG造成的。
         例如里程计数据出错 或者 激光雷达很久没有数据等等
         每次进行激光雷达的更新之后 m_linearDistance这个参数就会清零
          m_distanceThresholdCheck在开头定义为5 */
	if (m_linearDistance>m_distanceThresholdCheck){
		cerr << "***********************************************************************" << endl;
		cerr << "********** Error: m_distanceThresholdCheck overridden!!!! *************" << endl;
		cerr << "m_distanceThresholdCheck=" << m_distanceThresholdCheck << endl;
		cerr << "Old Odometry Pose= " << m_odoPose.x << " " << m_odoPose.y
			 << " " <<m_odoPose.theta << endl;
		cerr << "New Odometry Pose (reported from observation)= " << relPose.x << " " << relPose.y
			 << " " <<relPose.theta << endl;
		cerr << "***********************************************************************" << endl;
		cerr << "** The Odometry has a big jump here. This is probably a bug in the   **" << endl;
		cerr << "** odometry/laser input. We continue now, but the result is probably **" << endl;
		cerr << "** crap or can lead to a core dump since the map doesn't fit.... C&G **" << endl;
		cerr << "***********************************************************************" << endl;
	}

    //更新:把当前的位置赋值给旧的位置
	m_odoPose=relPose;
    //先声明为false，最后如果成功就赋值 true
	bool processed=false;
    //只有当机器人走过一定的距离或者旋转过一定的角度,或者过一段指定的时间才处理激光数据
    //否则太低效了，period_被构造函数写死成5秒，可以考虑修改
	// process a scan only if the robot has traveled a given distance or a certain amount of time has elapsed
	if (! m_count
		|| m_linearDistance>=m_linearThresholdDistance
		|| m_angularDistance>=m_angularThresholdDistance
		|| (period_ >= 0.0 && (reading.getTime() - last_update_time_) > period_))
    {
        //记录最近一次更新时间
		last_update_time_ = reading.getTime();

        //输出 Laser 位姿
		cerr << "Laser Pose= " << reading.getPose().x << " " << reading.getPose().y
			 << " " << reading.getPose().theta << endl;


		//this is for converting the reading in a scan-matcher feedable form
		assert(reading.size()==m_beams);//断言，错误则终止程序
		//复制一帧数据 把激光数据转换为scan-match需要的格式
        double * plainReading = new double[m_beams];
		for(unsigned int i=0; i<m_beams; i++){
			plainReading[i]=reading[i];
		}

		RangeReading* reading_copy = new RangeReading(reading.size(), &(reading[0]),
				static_cast<const RangeSensor*>(reading.getSensor()),
				reading.getTime());

        //如果不是第一帧数据
		if (m_count>0){
            /*
             * 扫描匹配
	            为每个粒子进行scanMatch，计算出来每个粒子的最优位姿，同时计算该最优位姿的得分和似然  对应于gmapping论文中的用最近的一次测量计算proposal的算法
	            除了进行scanMatch之外，还对粒子进行了权重的计算，并计算了粒子的有效区域，但不进行内存分配 内存分配在resample()函数中
                这个函数在gridslamprocessor.hxx里
            */
			scanMatch(plainReading);

            //至此 关于proposal的更新完毕了，接下来是计算权重
			onScanmatchUpdate();
            /*
	            由于scanMatch中对粒子的权重进行了更新，那么这个时候各个粒子的轨迹上的累计权重都需要重新计算
	            这个函数即更新各个粒子的轨迹上的累计权重是更新
	            GridSlamProcessor::updateTreeWeights(bool weightsAlreadyNormalized) 函数在gridslamprocessor_tree.cpp里面实现
            */
			updateTreeWeights(false);
            /*
                粒子重采样  根据 neff 的大小来进行重采样  不但进行了重采样，也对地图进行更新
                GridSlamProcessor::resample 函数在gridslamprocessor.hxx里面实现
            */
			resample(plainReading, adaptParticles, reading_copy);

		} else {//如果是第一帧激光数据
            //直接计算activeArea。因为这个时候，对机器人的位置是非常确定的，就是(0,0,0)
			for (ParticleVector::iterator it=m_particles.begin(); it!=m_particles.end(); it++){
				m_matcher.invalidateActiveArea();
				m_matcher.computeActiveArea(it->map, it->pose, plainReading);
				m_matcher.registerScan(it->map, it->pose, plainReading);

				//为每个粒子创建路径的第一个节点。该节点的权重为0,父节点为it->node(这个时候为NULL)
                //因为第一个节点就是轨迹的根，所以没有父节点
                // cyr: not needed anymore, particles refer to the root in the beginning!
				TNode* node=new	TNode(it->pose, 0., it->node,  0);
				//node->reading=0;
				node->reading = reading_copy;
				it->node=node;

			}
		}
        //	"Tree: normalizing, resetting and propagating weights at the end..." ;
        //进行重采样之后，粒子的权重又会发生变化，因此需要再次更新粒子轨迹的累计权重
        //GridSlamProcessor::updateTreeWeights(bool weightsAlreadyNormalized) 函数在gridslamprocessor_tree.cpp里面实现
		updateTreeWeights(false);

		delete [] plainReading;
		m_lastPartPose=m_odoPose; //update the past pose for the next iteration
        //机器人累计行走的多远的路程没有进行里程计的更新 每次更新完毕之后都要把这个数值清零
        m_linearDistance=0;
		m_angularDistance=0;
		m_count++;
		processed=true;

		//keep ready for the next step
		for (ParticleVector::iterator it=m_particles.begin(); it!=m_particles.end(); it++){
			it->previousPose=it->pose;
		}

	}
	m_readingCount++;
	return processed;
}
```

##### drawFromMotion()

```cpp
//在 motionmode.cpp 里面
// motion model
OrientedPoint 
MotionModel::drawFromMotion(const OrientedPoint& p, const OrientedPoint& pnew, const OrientedPoint& pold) const{
	//p 表示粒子估计的最优位置(机器人上一个时刻的最优位置)
    //pnew 表示里程计算出来的新的位置
    //pold 表示建图引擎记录的上一时刻的位姿(即上一个里程计的位置)
    double sxy=0.3*srr;
    //计算出pnew 相对于 pold走了多少距离，得到控制量
    //这里的距离表达是相对于车身坐标系来说的
	OrientedPoint delta=absoluteDifference(pnew, pold);
	// 初始化一个点
    OrientedPoint noisypoint(delta);
	//走过的三个方向的距离，为控制量添加上噪声项
    noisypoint.x+=sampleGaussian(srr*fabs(delta.x)+str*fabs(delta.theta)+sxy*fabs(delta.y));
	noisypoint.y+=sampleGaussian(srr*fabs(delta.y)+str*fabs(delta.theta)+sxy*fabs(delta.x));
	noisypoint.theta+=sampleGaussian(stt*fabs(delta.theta)+srt*sqrt(delta.x*delta.x+delta.y*delta.y));
	//限制角度的范围为 -π～π
    noisypoint.theta=fmod(noisypoint.theta, 2*M_PI);
	if (noisypoint.theta>M_PI)
		noisypoint.theta-=2*M_PI;
	//把加入了噪声的控制量 加到粒子估计的最优的位置上，得到新的粒子位姿的预估(根据运动模型推算)
    return absoluteSum(p,noisypoint);
}
```

##### scanMatch()

```cpp
inline void GridSlamProcessor::scanMatch(const double* plainReading){
double sumScore=0;//总得分初始化
 //对每个粒子都进行优化
for (ParticleVector::iterator it=m_particles.begin(); it!=m_particles.end(); it++){
    OrientedPoint corrected;
    double score, l, s;
    score=m_matcher.optimize(corrected, it->map, it->pose, plainReading);
    if (score>m_minimumScore){//如果分值大于有效最小阈值,就进行更新位姿
		it->pose=corrected;
    }
    //粒子的最优位姿计算了之后，重新计算粒子的权重，optimize得到了最优位姿，likelihoodAndScore算最优位姿的得分，分值为:l
    m_matcher.likelihoodAndScore(s, l, it->map, it->pose, plainReading);
    sumScore+=score;
    it->weight+=l;
    it->weightSum+=l;

    m_matcher.invalidateActiveArea();
    m_matcher.computeActiveArea(it->map, it->pose, plainReading);
}
}
```



#### updateMap()

```cpp
void
SlamGMapping::updateMap(const sensor_msgs::LaserScan& scan)
{
  ROS_DEBUG("Update map");
  //更新地图的时候，加了一把锁
  boost::mutex::scoped_lock map_lock (map_mutex_);
  GMapping::ScanMatcher matcher;

  //设置scanmatcher的各个参数
  matcher.setLaserParameters(scan.ranges.size(), &(laser_angles_[0]),
                             gsp_laser_->getPose());

  matcher.setlaserMaxRange(maxRange_);
  matcher.setusableRange(maxUrange_);
  matcher.setgenerateMap(true);

  //得到累计权重(weightSum)最大的粒子，不是当前的最大权重的粒子
  // 累计权重即该粒子在各个时刻的权重之和(轨迹上的各个节点的权重之和)
  GMapping::GridSlamProcessor::Particle best =
          gsp_->getParticles()[gsp_->getBestParticleIndex()];
  std_msgs::Float64 entropy;
  // computePoseEntropy 遍历粒子集合计算熵
  entropy.data = computePoseEntropy();
  // 发布位姿的熵
  if(entropy.data > 0.0)
    entropy_publisher_.publish(entropy);
  //如果没有地图，则初始化一个地图
  if(!got_map_) {
    // nav_msgs::GetMap::Response   map_
    map_.map.info.resolution = delta_;
    map_.map.info.origin.position.x = 0.0;
    map_.map.info.origin.position.y = 0.0;
    map_.map.info.origin.position.z = 0.0;
    map_.map.info.origin.orientation.x = 0.0;
    map_.map.info.origin.orientation.y = 0.0;
    map_.map.info.origin.orientation.z = 0.0;
    map_.map.info.origin.orientation.w = 1.0;
  } 

  // 地图的中点
  GMapping::Point center;
  center.x=(xmin_ + xmax_) / 2.0;
  center.y=(ymin_ + ymax_) / 2.0;

  //初始化一个 scanmatcherMap 创建一个地图
  GMapping::ScanMatcherMap smap(center, xmin_, ymin_, xmax_, ymax_, 
                                delta_);

  //更新地图
  //遍历最优粒子的整条轨迹树， 按照轨迹上各个节点存储的信息，计算激活区域更新地图
  ROS_DEBUG("Trajectory tree:");
  for(GMapping::GridSlamProcessor::TNode* n = best.node;
      n;
      n = n->parent)
  {
    ROS_DEBUG("  %.3f %.3f %.3f",
              n->pose.x,
              n->pose.y,
              n->pose.theta);
    if(!n->reading)
    {
      ROS_DEBUG("Reading is NULL");
      continue;
    }
    //进行地图更新
    matcher.invalidateActiveArea();
    matcher.computeActiveArea(smap, n->pose, &((*n->reading)[0]));
    matcher.registerScan(smap, n->pose, &((*n->reading)[0]));
  }
```

##### ScanMatcher()

```cpp
ScanMatcher::ScanMatcher(): m_laserPose(0,0,0){
	//m_laserAngles=0;
	m_laserBeams=0;
	m_optRecursiveIterations=3;
	m_activeAreaComputed=false;

    // 5cm解析度的默认参数
    // 这个参数是计算似然位姿的时候使用的，实际的gmapping中没有用到
	// This  are the dafault settings for a grid map of 5 cm
	m_llsamplerange=0.01;
	m_llsamplestep=0.01;
	m_lasamplerange=0.005;
	m_lasamplestep=0.005;

    //地图进行拓展的大小
	m_enlargeStep=10.;
	m_fullnessThreshold=0.1;

    //指示里程计和陀螺仪是否可靠
    //可靠：进行score计算的时候，就需要对离里程计数据比较远的位姿增加惩罚
    //对于应用来说，陀螺仪在短期内还是很可靠的
	m_angularOdometryReliability=0.;
	m_linearOdometryReliability=0.;

    //理论上的离激光点的空闲距离 也就是说沿着激光束方向离激光点这么远距离的栅格一定是空闲的
	m_freeCellRatio=sqrt(2.);

    //跳过一帧激光数据的开始几束激光
	m_initialBeamsSkip=0;
    m_linePoints = new IntPoint[20000];
	/*
	// This  are the dafault settings for a grid map of 10 cm
	m_llsamplerange=0.1;
	m_llsamplestep=0.1;
	m_lasamplerange=0.02;
	m_lasamplestep=0.01;
*/	
	// This  are the dafault settings for a grid map of 20/25 cm
	/*
	m_llsamplerange=0.2;
	m_llsamplestep=0.1;
	m_lasamplerange=0.02;
	m_lasamplestep=0.01;
	m_generateMap=false;
*/
}
```

##### Particle()

```cpp
GridSlamProcessor::Particle::Particle(const ScanMatcherMap& m):   map(m), pose(0,0,0), weight(0), weightSum(0), gweight(0), previousIndex(0){   node=0;}
```

##### getBestParticleIndex()

```cpp
// slam_gmapping : updateMap 方法调用
// 获得累计权重最大粒子
int GridSlamProcessor::getBestParticleIndex() const{
	unsigned int bi=0;
	double bw=-std::numeric_limits<double>::max();
	for (unsigned int i=0; i<m_particles.size(); i++)
		if (bw<m_particles[i].weightSum){
			bw=m_particles[i].weightSum;
			bi=i;
		}
	return (int) bi;
}
```

##### computePoseEntropy()

```cpp
double
SlamGMapping::computePoseEntropy()
{
  //计算位姿的信息熵
  double weight_total=0.0;
  for(std::vector<GMapping::GridSlamProcessor::Particle>::const_iterator it = gsp_->getParticles().begin();
      it != gsp_->getParticles().end();
      ++it)
  {
    weight_total += it->weight;
  }
  double entropy = 0.0;
  for(std::vector<GMapping::GridSlamProcessor::Particle>::const_iterator it = gsp_->getParticles().begin();
      it != gsp_->getParticles().end();
      ++it)
  {
    if(it->weight/weight_total > 0.0)
      entropy += it->weight/weight_total * log(it->weight/weight_total);
  }
  return -entropy;
}
```

ScanMatcherMap()

```cpp

```



#### updateTreeWeights()

- 归一化权重
- 轨迹树重置
- 权重沿着树传播

## 总结

















