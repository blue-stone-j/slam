# LOAM_NOTED
source code: https://github.com/cuitaixiang/LOAM_NOTED

[article1](https://github.com/blue-stone-j/papers/blob/main/2014%20LOAM%20Lidar%20Odometry%20and%20Mapping%20in%20Real-time.pdf): LOAM Lidar Odometry and Mapping in Real-time

[article2](https://github.com/blue-stone-j/papers/blob/main/2016%20Low-drift%20and%20real-time%20lidar%20odometry%20and%20mapping.pdf): LOAM Lidar Odometry and Mapping in Real-time

LOAM中文注解版，若有差错欢迎指正～

I fork from source code and add more comments.

Here you can see the loam code noted in Chinese and English.

For non-Chinese users, Google Translate will help you.

### helpful links
* 源码解析1——scanRegistration: https://zhuanlan.zhihu.com/p/145857388
* 细节分析: https://zhuanlan.zhihu.com/p/57351961
* 原理解析: https://zhuanlan.zhihu.com/p/111388877
* 原理与源码: https://zhuanlan.zhihu.com/p/29719106
* 论文与代码解析: https://blog.csdn.net/nksjc/article/details/76401092

###
* a [link](https://blog.csdn.net/weixin_46363611/article/details/108869992?utm_medium=distribute.pc_relevant.none-task-blog-2~default~BlogCommendFromMachineLearnPai2~default-10.pc_relevant_baidujshouduan&depth_1-utm_source=distribute.pc_relevant.none-task-blog-2~default~BlogCommendFromMachineLearnPai2~default-10.pc_relevant_baidujshouduan) that introduce some derivations 

### file and funciton

##### transformMaintenance
maintain tf.

##### scanRegistration
receive clouds and imus, deskew points and extract features.

##### laserOdometry
front end: odom

##### laserMapping
back end: optimization with local map