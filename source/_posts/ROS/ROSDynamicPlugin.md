---
title: 基于ROS的运行态插件切换机制
date: 2017-04-15 14:00:31
tags:
categories:
- 机器人事业
- ROS
description: 基于ROS，实现在系统运行中，插件的动态切换机制。
---
<!-- more -->

# 摘要
在研究生课题中需要基于ROS实现动态加载算法库，并且系统运行是可以手动切换算法库。比如系统刚开始运行时，调用的是算法库A。随着系统的运行，发现此时调用算法B会更好，但是又不能停止系统，导致信息丢失，因此运行过程中的动态切换。
经过调研，发现可以通过Pluginlib和Dynamic Reconfigure这两个库来实现。


# Pluginlib
pluginlib是一个C++库, 用于在ROS包里面动态的加载或卸载插件。插件本质上讲就是一个动态库，用这些动态库实现某些功能。因此通过切换动态库，系统就可以实现不同的功能。从而达到不用修改系统源码，只需要编写和添加插件的方式，就可以实现系统功能的扩展。

利用pluginlib编写插件有以下几个步骤：
**1.创建插件基类，定义统一接口。**
假设我要编写一个SLAM插件，则首先要编写SLAM插件的基类，假设其包名为slam_system。
```c++
namespace slam {

/**
 * @brief The SLAMBase class
 * all SLAM plugins should inherit this base class, and implement pluginlib-required methods.
 */
    class SLAMBase  {
    public:
    
        virtual void Activate() = 0;

        virtual void Shutdown() = 0;

    };

} // end namespace
```

**2.创建插件类**
所有SLAM插件都要继承SLAM插件的基类，并实现积累中定义好的接口。

```c++
namespace rgbd_slam{

	class Slam : public slam::SLAMBase {

	public:

        void Activate(){
        // implementment
        }

        void Shutdown(){
        // imimplementment
        }

	};
}

```

在这里插件类和插件基类往往不在一个package里面，因此要在插件类这边的CMakeLists.txt里面和package.xml里面定义好依赖。


**3.导出插件，并编译为动态链接库**

要使插件类能够被正确连接，需要四个步骤。
首先是使用PLUGINLIB_EXPORT_CLASS宏，对插件类进行标记，说明这个类可以被动态加载。这个宏被定义在pluginlib/class_list_macros.h头文件中，通常放置于cpp文件的底部。

```c++
#include<pluginlib/class_list_macros.h>
//宏操作的第一个参数为插件类的全名，第二个参数为插件类的基类全名，这两个名字都包括命名空间。
PLUGINLIB_EXPORT_CLASS(rgbd_slam::Slam, slam::SLAMBase);

```

然后将其编译为动态链接库。因此在CMakeLists.txt文件中添加。
```
add_library(rgbd_slam
	src/rgbd_slam.cpp
)
```
然后执行 catkin_make 就生成了动态链接库，默认生成库路径为工作空间下 devel/lib/ 文件夹。

**4.将插件加入ROS，使其可被加载**
接着编写插件描述文件。新建rgbd_slam.xml。

```
<library path="lib/librgbd_slam">
  <class name="rgbd_module" type="rgbd_slam::Slam" base_class_type="slam::SLAMBase">
    <description>This is a rgb_slam plugin.</description>
  </class>
</library>
```

library 标签指出了包含插件的动态库的相对路径，这里为 lib 文件夹下名为 librgbd_slam.so 动态链接库，library 标签中省去了库的后缀.so。class 标签指出了期望从动态库中导出的插件类。type，指明插件类的namespace::class。 base_class ，指明插件的基类的namespace::class。。name，使用name的方法标识插件。

最后在package.xml文件中申明rgbd_slam.xml的位置
```
  <export>
    <!-- Other tools can request additional information be placed here -->
    <hi_slam plugin="${prefix}/rgbd_module.xml" />
  </export>
```

**5.检测插件类**
```
rospack plugins --attrib=plugin slam_system

```
slam_system是定义插件基类的包名。

**6.使用插件类**

```
//slam_system是插件基类package的名字。slam::SLAMBase是namespace::基类
pluginlib::ClassLoader<slam::SLAMBase> loader("slam_system", "slam::SLAMBase");
boost::shared_ptr<slam::SLAMBase> slam = poly_loader.createInstance("rgbd_slam::Slam");
//or
boost::shared_ptr<slam::SLAMBase> slam = poly_loader.createInstance("rgbd_module");

slam->Activate();
slam->Shutdown();

```
到这里，一个完整的pluginlib机制就算完成了。
以后只需要修改rgbd_slam的实现，而不需要修改slam_system包，就能实现rgbd_slam功能的修改。如果需要多种slam的实现，比如laser_slam，则只需要像rgbd_slam的实现方式一样进行添加即可。

**总结Pluginlib:**
- 创建插件基类，定义统一接口
- 继承插件基类编写插件类，实现统一接口
- 编译插件类，得到动态链接库
- 编写xml描述文件，并在package.xml中申明描述
- 基于pluginlib::ClassLoader和boost::shared_ptr调用插件


# Dynamic Reconfigure
由于我们需要在系统运行是可以手动切换算法库，比如系统运行时，我们希望能够发送一个指令，使其从rgbd_slam转换为laser_slam。通过Dynamic Reconfigure机制可以非常方便的实现这种切换功能。

首先来了解下什么是Dynamic Reconfigure。Dynamic Reconfigure提供了不必重启节点，就可以更改节点参数的方法。因此借助ROS的参数的改变，即可实现插件的切换。


利用Dynamic Reconfigure有以下几个步骤


**1.创建和使用cfg文件。**

在cfg文件夹下创建slam.cfg文件
```
PACKAGE = 'slam_system'

from dynamic_reconfigure.parameter_generator_catkin import *

gen = ParameterGenerator()

gen.add("slam", str_t, 0, "The name of the plugin for the slam.", "rgbd_slam")

exit(gen.generate(PACKAGE, "slam_node", "slam_node"))

```
cfg其实是一个基于python的模块，首先创建一个ParameterGenerator对象，然后调用其add()函数将参数添加到参数列表中。add()的参数含义分别是：参数名，参数类型，级别，描述，缺省值，最小值，最大值。

这里我添加了一个参数slam，类型是string，缺省值是rgbd_slam。
最后一行是告知generator创建必要的文件并退出程序。第二个参数是cfg文件依附的节点名，第三个参数是生成头文件名称的前缀。这里会生成一个叫做slam_nodeConfig.h的头文件。

编辑完成之后，我们需要添加运行权限才能使用.cfg文件。
最后还要在CMakeList中添加依赖。
```
generate_dynamic_reconfigure_options(
    cfg/slam.cfg
)
add_dependencies(slam_node ${PROJECT_NAME}_gencfg)
```


**2.监听参数修改**
监听参数修改指在参数发生改变时，触发注册的回调函数。在我们的需求中，就是在回调函数中完成插件的切换。
```
#include <dynamic_reconfigure/server.h>
#include <slam_system/slam_nodeConfig.h>

dynamic_reconfigure::Server<slam_system::slam_nodeConfig> server;
dynamic_reconfigure::Server<slam_system::slam_nodeConfig>::CallbackType f;
f = boost::bind(&ConfigCb, _1, _2);
server.setCallback(f);

```
其中ConfigCb是一个回调函数，每次参数发生变化时会被触发。
```
void ConfigCb(slam_node::slam_nodeConfig &config, uint32_t level)
{
//利用config信息完成插件切换
}

```

**3.利用rqt_reconfigure可视化调整参数**
```
rosrun rqt_reconfigure rqt_reconfigure
```

**总结Dynamic Reconfigure：**
- 在package.xml文件中加入dynamic_reconfigure编译依赖和运行依赖。
- 创建.cfg文件，添加参数名到参数列表。
- 修改CMakeLists.txt。添加generate_dynamic_reconfigure_options以及add_dependencies。
- 在程序中加入两个头文件，并声明动态调整服务，绑定其回调函数。在回调函数中实现对参数对程序的影响


# 参考文献
1.[ROS中pluginlib的使用总结](http://www.rosclub.cn/post-157.html)

2.[ROS的参数应用以及动态调整（下）](http://www.rosclub.cn/post-159.html)



