About the LibrePilot Project
============================

### Open - Collaborative - Free

The LibrePilot open source project was founded in July 2015. It focuses on
research and development of software and hardware to be used in a variety of
applications including vehicle control and stabilization, unmanned autonomous
vehicles and robotics. One of the project’s primary goals is to provide an open
and collaborative environment making it the ideal home for development of
innovative ideas.

LibrePilot welcomes and encourages exchange and collaboration with other
projects, like adding support for existing hardware or software in
collaboration under the spirit of open source.

LibrePilot finds its roots in the OpenPilot project and the founding members
are all long-standing contributors in that project.

The LibrePilot Project will be governed by a board of members using consensual
methods to make important decisions and to set the overall direction of the
project.

The LibrePilot source code is released under the OSI approved GPLv3 license.
Integral text of the license can be found at [www.gnu.org](http://www.gnu.org/licenses/gpl-3.0.en.html)


Links for the LibrePilot Project
--------------------------------

- [Main project web site](https://www.librepilot.org)
- [Project forums](https://forum.librepilot.org)
- [Software downloads](https://librepilot.atlassian.net/wiki/display/LPDOC/Downloads)
- [Wiki](https://librepilot.atlassian.net/wiki/display/LPDOC/Welcome)
- [Source code repository](https://bitbucket.org/librepilot)
- [Mirror](https://github.com/librepilot)
- [Issue tracker](https://librepilot.atlassian.net)
- [Gitter Chat](https://gitter.im/librepilot/LibrePilot)
- IRC: #LibrePilot on FreeNode


| Builder      | Status        |
|:-------------|:--------------|
| Travis CI    |[![Build Status](https://travis-ci.org/librepilot/LibrePilot.svg?branch=next)](https://travis-ci.org/librepilot/LibrePilot)|
| Tea CI       |[![Build Status](https://tea-ci.org/api/badges/librepilot/LibrePilot/status.svg?branch=next)](https://tea-ci.org/librepilot/LibrePilot)|


# MODIFY TO BUILD ON UBUNTU18
1. librepilot/ground/gcs/src/plugins/uploader/opdfu.h  
line 34, add Q_NAMESPACE
```
namespace OP_DFU {
Q_NAMESPACE
enum TransferTypes {
    FW,
    Descript
};
```

2. librepilot/ground/gcs/src/libs/osgearth/utils/utility.cpp  
line 483, origin:  
```
if (eq.getElevation(geoPoint, elevation, 0.0))
```
change into  
```
double tmp = 0.0;
if (eq.getElevation(geoPoint, elevation, &tmp))
```

# Add a uavobject to gcs and flight


1. 创建UAVObject描述文件（xx.xml)，并存放到\librepilot\shared\uavobjectdefinition目录中；
2. 在\librepilot\flight\targets\boards\（xxx飞控板）\firmware\UAVObjects.inc中加入新创建的.xml文件名，以便生成飞机端的.c和.h文件；注意coptercontrol没有UAVObjects.inc，定义在firmware\Makefile中
3. 在\librepilot\ground\gcs\src\plugins\uavobjects\uavobjects.pro中加入新创建的.xml文件名，以便生成地面站端的.cpp和.h文件；

4、在\librepilot\flight\targets\boards\（xxx飞控板）\firmware\Makefile中加入相应的模块，如MODULES += XXX；（如果新建模块的话）

5. make package 生成安装包；
生成安装包时需要在所有飞控版的文件夹相应文件中加入定义的uavobjectxml文件，默认版本生成安装包时会报version is not number的错误,这是因为自己删除了librepilot的默认git上传到自己的git后UPSTREAM_VER返回None导致的，解决办法是把当前版本16.09直接赋值  
对Ubuntu18 找到 librepilot/package/linux/deb.mk 中，UPASTREAM_VER改为  
```
UPSTREAM_VER         := 16.09  #$(subst -,~,$(subst RELEASE-,,$(PACKAGE_LBL)))
```

对win10
默认会报四位版本号错误的问题 解决方法：根据错误提示，在/msys64/home/xxx/librepilot/package/winx86中只有一个打包脚本gcs.nsi,打开该文件，修改line 56的版本信息  
```
 # VIProductVersion ${VERSION_FOUR_NUM}
VIProductVersion "2.46.0.0"
```
6. 打包make package 时出现如下错误：

   ```
   dpkg-shlibdeps: 错误: no dependency information found for /home/xindong324/Qt5.12.11/5.12.11/gcc_64/lib/libQt5Widgets.so.5 (used by debian/librepilot/usr/lib/librepilot-gcs/libQScienceSpinBox.so.1.0.0)
   ```

   主要原因是系统安装了多个版本的Qt，默认Qt库是default.config中的Qt9.5, 因此在构建安装包时需要切换Qt版本为5.9
   
   ```
   export QT_SELECT=qt5
   ```
   
   

# uavobject 定义的注意事项

## 定义单个变量，数组和结构体的区别：
1. 定义变量, elements="1"
```
<field name="Yaw" units="degrees" type="float" elements="1"/>

```
生成结果为:  
```C++
float Yaw;
```

2. 定义数组, elements="4"
```
<field name="ModeParameters" units="" type="float" elements="4" default="0"/>

```
生成结果为：
```C
float ModeParameters[4];

```

3. 定义结构体， elementnames="North,East,Down",是elementnames不是elements
```
<field name="Start" units="m" type="float" elementnames="North,East,Down" default="0"/>

```

生成结果为：
```
typedef struct __attribute__ ((__packed__)) {
    float North;
    float East;
    float Down;
}  PathDesiredStartData ;

```
## 飞行模式
uavobject中Flightmodesettings.xml关于Stabilization1Settings定义了每个通道不包含的姿态模式,以limits=“%NE”
```
<field name="Stabilization1Settings" units="" type="enum"
        elementnames="Roll,Pitch,Yaw,Thrust"
        options="Manual,Rate,RateTrainer,Attitude,AxisLock,WeakLeveling,VirtualBar,Acro+,Rattitude,AltitudeHold,AltitudeVario,CruiseControl,SystemIdent"
        defaultvalue="Attitude,Attitude,AxisLock,Manual"
        limits="%NE:AltitudeHold:AltitudeVario:CruiseControl:SystemIdent; \
            %NE:AltitudeHold:AltitudeVario:CruiseControl:SystemIdent; \
            %NE:RateTrainer:AltitudeHold:AltitudeVario:CruiseControl:Attitude:Rattitude:WeakLeveling:VirtualBar:SystemIdent; \
            %NE:Rate:RateTrainer:Attitude:AxisLock:WeakLeveling:VirtualBar:Acro+:Rattitude:SystemIdent;"
            />

```

#只编译飞控固件的纯净版
1. librepilot/flight/targets/Boards中其他飞控文件夹删除
2. librepilot/Makefile 11行删掉不用的boards定义
```
ALL_BOARDS    := coptercontrol oplinkmini revolution osd revoproto simposix discoveryf4bare gpsplatinum revonano sparky2
```
3. NB: 这样无法编译地面站



# 调试使用注意事项

* 在配合mocap使用mavlink时，需要在地面站 configuration -> hardware 中将MainPort设置为Mavlink，同时在System->设置->HwSetting中将Mainport波特率设置为115200