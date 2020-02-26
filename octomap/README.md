## octomap修改编译
### 安装与修改步骤
1. 参考origin_README.md中的安装描述将环境补全
> sudo apt-get install cmake doxygen libqt4-dev libqt4-opengl-dev libqglviewer-qt4-dev

2. 编译
> mkdir build && cd build	
> cmake ..
> 使用下面的参数可按Debug模式编译，方便后继调试
> cmake -DCMAKE_BUILD_TYPE=Debug ..

3. 生成链接库
> make install

4. 用编译好的链接库替换ros自带的
> cd <path to octomap>/octomap/
> sudo ./cp2roslib.sh



