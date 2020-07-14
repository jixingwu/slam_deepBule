1. $ /bin/zsh -c "$(curl -fsSL https://gitee.com/cunkai/HomebrewCN/raw/master/Homebrew.sh)"
brew 工具安装，用于安装各种软件。
2. $ brew updata
更新软件源，软件源表示的时软件下载地址，如果有新版本软件，软件源会更新
3. $ brew install cmake
用brew安装cmake软件，总结下来就是 “brew install +某个软件包”
3. $ mkdir cppFiles
创建cppFiles文件夹
4. $ cd cppFiles
进入cppFiles文件夹
5. $ gedit CMakeLists.txt
创建一个名字为CMakeLists的txt文本文件
6. 在CMakeLists.txt中输入
cmake_minimum_required(VERSION 2.8)
project(helloworld)
add_executable(helloworld_node helloworld.cpp)
6.1 cmake_minimum_required(VERSION 2.8)表示cmake（之前安装的软件名称）的最小需求版本为2.8。cmake_minimum_required（cmake的最小需求）、VERSION（版本）
6.2 project(helloworld)创建一个工程名字为helloworld。project（工程）
6.3 add_executable(helloworld_node helloworld.cpp)给helloworld.cpp文件生成名字为helloworld_node的可执行文件。另：helloworld.cpp必须包含main函数。add（添加）executable（可执行的）
总结以上三步：首先CMakeLists.txt文件是用来编译C++文件的。然后编译C++需要一个名为cmake的软件，在CMakeLists.txt中第一行说我需求的cmake最小版本为2.8，必须是比2.8高的版本采用执行我。第二行，创建工程。第三行，给编写好的C++文件生成可执行的文件。
7. $ mkdir build
在终端中输入mkdir build命令，表示创建一个名为build的文件夹。该文件夹用来存放编译后生成的各种类型的文件。
8. $ cd build
进入build文件夹
9. $ cmake ..
cmake是编译我们写好的CMakeLists.txt，告诉系统我想按照CMakeLists.txt里面写的三行步骤编译我的C++文件，也就是helloworld.cpp。".."表示在上一级文件路径中执行。因为build文件夹里什么都没有，你写好的两个文件都在cppFiles中呢。
10. $ make
执行完第9步会在build中出现很多文件，不用管他，直接执行make，当终端中出现“[100%] Built target helloworld_node”100%或successful等字样表示已经编译成功，你编写的代码没有错误。
11. $ ./helloworld_node
"./"表示执行某个可执行文件。helloworld_node还记不记得啦，是我们在CMakeLists.txt中告诉系统我们给helloworld.cpp生成的可执行文件，名字叫helloworld_node。这个命令的意思是执行这个可执行文件。
最后你可以在终端看到C++运行结果。“hello world！”这个就表示成功了。

另：如果你修改helloworld.cpp文件的话，需要重新编译，方法为：
1. $ cd build
2. $ cmake ..
3. $ make
4. $ ./helloworld_node

不需要修改CMakeLists.txt文件了，就像c语言一样编译，运行就OK了



