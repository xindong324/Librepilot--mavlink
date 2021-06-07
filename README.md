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
2. 在\librepilot\flight\targets\boards\（xxx飞控板）\firmware\UAVObjects.inc中加入新创建的.xml文件名，以便生成飞机端的.c和.h文件；
3. 在\librepilot\ground\gcs\src\plugins\uavobjects\uavobjects.pro中加入新创建的.xml文件名，以便生成地面站端的.cpp和.h文件；

4、在\librepilot\flight\targets\boards\（xxx飞控板）\firmware\Makefile中加入相应的模块，如MODULES += XXX；（如果新建模块的话）

5. make package 生成安装包；
生成安装包时需要在所有飞控版的文件夹相应文件中加入定义的下xml文件，默认版本生成安装包时会报version is not number的错误,这是因为自己删除了librepilot的默认git上传到自己的git后UPSTREAM_VER返回None导致的，解决办法是把当前版本16.09直接赋值  
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

