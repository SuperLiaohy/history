# CMake

> 本文档为重温CMake的学习记录

> 学习环境为Linux-mint22.1 基于(ubuntu24.04) 的CLion

## 创建一个基础的CMake项目

### 1. 新建一个CMakeLists.txt文件

````bash
touch CMakeLists.txt
````

### 2. 确定CMake的版本信息

````cmake
cmake_minimum_required(VERSION 3.28)
````

### 3. 确定项目名

````cmake
project(cmake_learn CXX)
````

```markdown
project(<PROJECT-NAME> [<language-name>...])
project(<PROJECT-NAME>
        [VERSION <major>[.<minor>[.<patch>[.<tweak>]]]]
        [DESCRIPTION <project-description-string>]
        [HOMEPAGE_URL <url-string>]
        [LANGUAGES <language-name>...])
```

### 4. 添加源文件

>  可以使用宏`${PROJECT_NAME}`来代指项目名

```cmake
add_executable(${PROJECT_NAME} main.cpp)
```

```markdown
add_executable(<name> <options>... <sources>...)
```

### 5. 编译

首先新建一个build目录

然后`cd`到build目录下执行`cmake ..`cmake会自动生成默认的构建文件

在linux下默认是`Unix Makefiles`

在windows下默认应该是`Visual Studio`的构建工具链

可以使用`cmake --help`进行查看

同时也可以通过宏`$CMAKE_GENERATOR`查看选择的构建工具链（默认是空），也可以通过修改该宏来改变默认的构建工具链

也可以通过生成时就指定好构建工具链`cmake .. -G "Ninja"`

生成完后，可以使用对应的构建工具链的命令执行，比如`Unix Makefiles`可以使用`make`，`Ninja`可以使用`Ninja`，同时也可以使用统一的构建命令`cmake --build .`

## 指定Cpp的标准

使用`set`函数修改宏`${CMAKE_CXX_STANDARD}`

```cmake
set(CMAKE_CXX_STANDARD 11) # 设置来cpp的标准为cpp11
set(CMAKE_CXX_STANDARD_REQUIRED ON) # 这是强制要求使用指定的CXX版本
```

如果`${CMAKE_CXX_STANDARD_REQUIRED}`为`ON`将强制要求使用指定的CXX版本来构建，如果当前编译器不支持所指定的CXX版本，将会报错，而如果使用`${CMAKE_CXX_STANDARD_REQUIRED}`为`OFF`的话，虽然也会使用指定的CXX版本进行构建，但是如果使用的编译器不支持的话，并不会报错，而是使用所能支持的之前的标准。

## CMake与Cpp之间的数据传输

### 1. 显示CMake里变量的值

使用`message`函数可以显示CMake宏的值。

````cmake
message(STATUS "${CMAKE_CXX_STANDARD}") # 这样便可以查看指定的cpp标准了，如果没有设置该宏的话，那便是显示空
````

message用法

```markdown
message([<mode>] "message text" ...)
The optional <mode> keyword determines the type of message, which influences the way the message is handled:
FATAL_ERROR
CMake Error, stop processing and generation.
The  cmake(1)   executable will return a non-zero exit code  .
SEND_ERROR
CMake Error, continue processing, but skip generation.
WARNING
CMake Warning, continue processing.
AUTHOR_WARNING
CMake Warning (dev), continue processing.
DEPRECATION
CMake Deprecation Error or Warning if variable  CMAKE_ERROR_DEPRECATED or  CMAKE_WARN_DEPRECATED is enabled, respectively, else no message.
(none) or NOTICE
Important message printed to stderr to attract user's attention.
STATUS
The main interesting messages that project users might be interested in. Ideally these should be concise, no more than a single line, but still informative.
VERBOSE
Detailed informational messages intended for project users. These messages should provide additional details that won't be of interest in most cases, but which may be useful to those building the project when they want deeper insight into what's happening.
DEBUG
Detailed informational messages intended for developers working on the project itself as opposed to users who just want to build it. These messages will not typically be of interest to other users building the project and will often be closely related to internal implementation details.
TRACE
Fine-grained messages with very low-level implementation details. Messages using this log level would normally only be temporary and would expect to be removed before releasing the project, packaging up the files, etc.
Added in version 3.15: Added the NOTICE, VERBOSE, DEBUG, and TRACE levels.
```

### 2. 将CMake里的宏变量导入到头文件里

在`CMake`里使用`configure_file`函数可以将特定的文件按一定规律转化为头文件，从而实现将CMake里的宏变量导入到头文件，和Cpp文件进行交互了。使用`configure_file`生成的头文件会在默认`${CMAKE_BINARY_DIR}`目录下

规则大致如下

如果要使用CMake里的变量则要用`${}`包裹或者用`@@`包裹，如果想让变量为字符串必须在外面包裹`""`，如要表示项目名这一字符串则要写为`"${PROJECT_NAME}"`

```cpp
#define NAME "${PROJECT_NAME}"
```

其他规则和头文件规则大致一样

注意不要忘记写头文件保护了

如`#pragma once`或者

````cpp
#ifndef __XX__
#define __XX__

#endif
````

用来进行头文件保护

### #cmakedefine

使用`#cmakedefine`时要接cmake里已经定义过的变量比如下面

```cmake
option(IS_CMAKE_VAR "is cmake var?" ON)
```

```cmake
#cmakedefine IS_CMAKE_VAR
#cmakedefine IS_CMAKE_VARS
```

在这种情况下`IS_CMAKE_VAR`在CMake被定义的情况下，且不为`OFF`的话，则可以在头文件里被定义，而`IS_CMAKE_VARS`在CMake里没有被定义所以不会在头文件里被定义

```cpp
#define IS_CMAKE_VAR
/* #undef IS_CMAKE_VARS */
```

而如果该变量在CMake里被定义了，但是却为`OFF`，在头文件里也不会被定义

```cmake
option(IS_CMAKE_VAR "is cmake var?" OFF)
```

```cmake
#cmakedefine IS_CMAKE_VAR
#cmakedefine IS_CMAKE_VARS
```

会变成

```cpp
/* #undef IS_CMAKE_VAR */
/* #undef IS_CMAKE_VARS */
```

> 章节小知识
>
> CMake里`"${}"`和`${}`的区别如果他们的值里没有空格的话表示目录使用效果应该一样，但是如果他们的值中有空格的话，则需要使用`"${}"`来表示一个整体。此时若使用`${}`的话则会被值中的空格分隔为几部分。

使用`option`可以创建一个bool变量，比如

````cmake
option(USE_LIB "Use my lib?" OFF)
````



## 生成和链接库

在原目录下新建一个目录，然后以该目录生成一个库的步骤如下

这是目录结构

````bash
.
├── build
├── cmake_header.h.in
├── CMakeLists.txt
├── main.cpp
└── MyLib
````

在MyLib下新建库的源文件和头文件以及库的CMakeLists.txt文件。

使用`add_library`函数便可以新建这个库

和`add_executable`用法大致一样，后面接源文件

然后在主CMakeLists.txt下可以使用`add_subdirectory`添加那个库所在的目录

然后使用`target_link_libraries`添加那个库所叫的名字。

这样便成功的添加完一个库来

从CMakeLists.txt

```cmake
add_library(MyLibs my_math.cpp my_math.hpp)
```

主CMakeLists.txt

```cmake
add_subdirectory(MyLib)

target_link_libraries(${PROJECT_NAME} PUBLIC MyLibs)
```

## 链接一个可选的库

依赖上面两节，我们能设置一个`bool`的变量来控制是否链接上这个库，如

```cmake
option(IS_USE_OWN_LIB "Whether to use my own library" ON)

if(IS_USE_OWN_LIB)
    list(APPEND EXTRA_LIB MyLibs)
    list(APPEND EXTRA_INCLUDE "${PROJECT_SOURCE_DIR}/MyLib")
endif()

target_link_libraries(${PROJECT_NAME} PUBLIC ${EXTRA_LIB})
target_include_directories(${PROJECT_NAME} PUBLIC
                            "${PROJECT_BINARY_DIR}"
                            "${EXTRA_INCLUDE}"
)
```

 这样就可以做到只修改变量`IS_USE_OWN_LIB`变量就可以是是否链接该库

## 给库添加依赖头文件

> 给生成的库文件添加好里依赖头文件就不需要我们再手动导入头文件了

在库文件目录下的CMakeLists.txt下使用`target_include_directories`函数可以为库文件添加依赖选项‘

有 `PUBLIC`，`INTERFACE`，`PRIVATE`三种类型。

`PUBLIC`：本目标需要，依赖这个目标的其他目标也需要

`INTERFACE`：本目标不需要，依赖这个目标的其他目标需要

`PRIVATE`：本目标需要，依赖这个目标的其他目标不需要

> 同时使用`${CMAKE_CURRENT_SOURCE_DIR}`可以代指当前CMakeLists.txt所在的目录。
>
> （`${CMAKE_SOURCE_DIR}`只输出主CMakeLists.txt所在的目录）

## 使用接口库设置Cpp标准

```cmake
# 新建一个接口库
add_llibrary(<name> INTERFACE)
# 设置接口库的特性
target_compile_feature(${PROJECT_NAME} INTERFACE cxx_std_11)
# 链接上接口库
target_link_library(${PROJECT_NAME} PUBLIC <name>)
```

这样便可以通过设置接口库来修改Cpp的标准了

但是实测似乎有点bug：当我设置的Cpp标准比默认的低时不生，标准Cpp没有变（使用宏`__cplusplus`查看的），只有设置的标准比默认的高时才发生改变（即`__cplusplus`发生变化）。

使用这种方法可以更细致的配置Cpp标准，因为这种不是在全局定义Cpp的标准而是针对指定目标设置Cpp标准。

## 初识生成器表达式

CMake构建过程大致可以分为三个阶段

1. **配置阶段（Configuration Phase）**
    在这个阶段，CMake 读取 CMakeLists.txt 文件，解析项目配置，检查系统环境、依赖项、编译器等，并生成缓存文件（CMakeCache.txt）。
   - 主要任务：确定构建系统的配置，生成中间文件。
   - 运行命令：cmake . 或 cmake -S . -B build
2. **生成阶段（Generation Phase）**
    CMake 根据配置阶段的结果生成构建系统（如 Makefile、Ninja 文件或 IDE 项目文件）。这是将 CMake 的高级描述转换为具体构建工具指令的阶段。
   - 主要任务：生成构建工具的原生配置文件。
   - 运行命令：通常与配置阶段一起由 cmake 命令完成。
3. **构建阶段（Build Phase）**
    使用生成的构建系统文件，调用底层构建工具（如 make、ninja 或 IDE）编译源代码并生成可执行文件或库。
   - 主要任务：编译和链接代码。
   - 运行命令：cmake --build build 或直接使用 make/ninja。

而生成器表达式作用于生成阶段，在配置阶段不解析。

古使用`message`是无法查看生成器表达式的值的

先将其中的两种生成器表达式

`$<condition:true_string>`

- 如果`condition`为1，则表达式的值为`true_string`
- 如果`condition`为0，则表达式的值为空

`$<COMPILE_LANG_AND_ID:language,compiler_ids>`

- 如果当前所用语言和`language`一致，且编译器ID在`compiler_ids`的列表中，则表达式的值为1,否则为0
- `compiler_ids`多个时之间用逗号隔开

某些生成器表达式可以使用以下手段来查看，比如`$<condition:true_string>`可以使用这种来查看，但是`$<COMPILE_LANG_AND_ID:language,compiler_ids>`似乎无法使用这种方法来查看

```cmake
add_custom_target(ged COMMAND ${CMAKE_COMMAND} -E echo "$<1:hello>")
```

## 安装规则

使用`install`可以进行安装

`install`可以接许多不同类型比如常用的有

```markdown
install(TARGETS <target>... [...])
install({FILES | PROGRAMS} <file>... [...])
install(DIRECTORY <dir>... [...])
```

使用`install`进行安装是默认安装在`${CMAKE_INSTALL_PREFIX}`的目录下，比如

````cmake
install(TARGETS ${PROJECT_NAME} DESTINATION bin)
````

目标`${PROJECT_NAME}`会被安装在`${CMAKE_INSTALL_PREFIX}/bin`目录下，而`${CMAKE_INSTALL_PREFIX}`在`linux-mint22.1`上默认是`/usr/local`。

`${CMAKE_INSTALL_PREFIX}`可以在`install`时指定，如下

```bash
cmake --install . --prefix=/usr/local # 将${CMAKE_INSTALL_PREFIX}设置为/usr/local
```

## 测试支持

开启测试使用函数`enable_testing()`

添加一个测试使用函数`add_test()`，基本用法如下。

```markdown
add_test(NAME <name> COMMAND <command> [<arg>...]
         [CONFIGURATIONS <config>...]
         [WORKING_DIRECTORY <dir>]
         [COMMAND_EXPAND_LISTS])
```

example:

```cmake
add_test(NAME Runs COMMAND git --version)
```

## 系统检测

需要使用系统检测要先引入`CheckCXXSourceCompiles`模块

`````cmake
inlcude(CheckCXXSourceCompiles)
`````

引入模块后，便可以使用`check_cxx_source_compiles`函数了。其用法为。

```markdown
check_cxx_source_compiles(<code> <resultVar>
                          [FAIL_REGEX <regex1> [<regex2>...]])
```

example:

```cmake
check_cxx_source_compiles("
#include <cmath>
int main() {
	std::exp(1.0);
	return 0;
}
" HAVE_EXP)
```

其为检测前面一段代码是否能过编译，如果能过编译则`HAVE_EXP`为`TRUE`，如果无法过编译则`HAVE_EXP`为`FALSE`。

然后可以使用`target_compile_definitions()`来控制宏定义是否定义，进而代码中判断是否使用相关特性。

example:

```cmake
# target_compile_definitions(<target> <INTERFACE|PUBLIC|PRIVATE> [item1 ...])
target_compile_definitions(${PROJECT_NAME} PRIVATE "HAVE_EXP")
```

在cpp文件里就可以使用

```c++
#ifdef HAVE_EXP
...
#else
...
#endif
```

## 自定义命令

使用`add_custom_command`函数可以使用自定义命令。常用用法如下。example:

```cmake
add_cusotm_command(
	OUTPUT ${CMAKE_SOURCE_DIR}
	COMMAND	${PROJECT_NAME}
	DEPENDS ${PROJEC_NAME}
)
```

上述用法表示，输出目录为`${CMAKE_SOURCE_DIR}`，执行的命令为`${PROJECT_NAME}`，执行的依赖是`${PROJECT_NAME}`。

## 打包程序

使用`cli`命令`cpack`来进行打包

首先在`CMakeLists.txt`末尾添加以下几段代码

```cmake
include(InstallRequiredSystemLibraries)
set(CPACK_RESOURCES_FILE_LICENSE "${CMAKE_CURRENT_SOURCE_DIR}/License.txt") # 可选 License
set(CPACK_PACKAGE_VERSION_MAJOR "${Tutorial_VERSION_MAJOR}") # 可选 版本号
set(CPACK_PACKAGE_VERSION_MINOR "${Tutorial_VERSION_MINOR}") # 可选 版本号
set(CPACK_SOURCE_GENERATOR "TGZ") # 指定源码的打包方式为.tar.gz
include(CPack)
```

可以使用`cpack --help`来查看支持的打包方式

```bash
(base) liaohy@liaohyHP:~$ cpack --help
Usage

  cpack [options]

Options
  -G <generators>              = Override/define CPACK_GENERATOR
  -C <Configuration>           = Specify the project configuration
  -D <var>=<value>             = Set a CPack variable.
  --config <configFile>        = Specify the config file.
  -V,--verbose                 = Enable verbose output
  --trace                      = Put underlying cmake scripts in trace mode.
  --trace-expand               = Put underlying cmake scripts in expanded
                                 trace mode.
  --debug                      = Enable debug output (for CPack developers)
  -P <packageName>             = Override/define CPACK_PACKAGE_NAME
  -R <packageVersion>          = Override/define CPACK_PACKAGE_VERSION
  -B <packageDirectory>        = Override/define CPACK_PACKAGE_DIRECTORY
  --vendor <vendorName>        = Override/define CPACK_PACKAGE_VENDOR
  --preset                     = Read arguments from a package preset
  --list-presets               = List available package presets
  -h,-H,--help,-help,-usage,/? = Print usage information and exit.
  --version,-version,/V [<file>]
                               = Print version number and exit.
  --help <keyword> [<file>]    = Print help for one keyword and exit.
  --help-full [<file>]         = Print all help manuals and exit.
  --help-manual <man> [<file>] = Print one help manual and exit.
  --help-manual-list [<file>]  = List help manuals available and exit.
  --help-command <cmd> [<file>]= Print help for one command and exit.
  --help-command-list [<file>] = List commands with help available and exit.
  --help-commands [<file>]     = Print cmake-commands manual and exit.
  --help-module <mod> [<file>] = Print help for one module and exit.
  --help-module-list [<file>]  = List modules with help available and exit.
  --help-modules [<file>]      = Print cmake-modules manual and exit.
  --help-policy <cmp> [<file>] = Print help for one policy and exit.
  --help-policy-list [<file>]  = List policies with help available and exit.
  --help-policies [<file>]     = Print cmake-policies manual and exit.
  --help-property <prop> [<file>]
                               = Print help for one property and exit.
  --help-property-list [<file>]= List properties with help available and
                                 exit.
  --help-properties [<file>]   = Print cmake-properties manual and exit.
  --help-variable var [<file>] = Print help for one variable and exit.
  --help-variable-list [<file>]= List variables with help available and exit.
  --help-variables [<file>]    = Print cmake-variables manual and exit.

Generators
  7Z                           = 7-Zip file format
  DEB                          = Debian packages
  External                     = CPack External packages
  IFW                          = Qt Installer Framework
  NSIS                         = Null Soft Installer
  NSIS64                       = Null Soft Installer (64-bit)
  NuGet                        = NuGet packages
  RPM                          = RPM packages
  STGZ                         = Self extracting Tar GZip compression
  TBZ2                         = Tar BZip2 compression
  TGZ                          = Tar GZip compression
  TXZ                          = Tar XZ compression
  TZ                           = Tar Compress compression
  TZST                         = Tar Zstandard compression
  ZIP                          = ZIP file format


```

在`windows`上默认的打包方式是`NSIS`，需要下载软件`NSIS`方可执行打包。

打包前需先配置好`CMake`。

````bash
# 配置CMake
cmake ..
# 打包 --使用默认方式
cpack
# 也可手动指定打包的方式
cpack -G ZIP
# 也可进行源码打包
cpack --config CPackSourceConfig.cmake
````

使用`cpack`打包的内容为`install`的内容，就是将`install`要安装的内容安装进入`cpack`指定的打包格式中。

## 再识库

在使用`add_library`函数 指定`STATIC`可以生成静态库，而指定`SHARED`可以生成静态库。

而如果缺省一般默认生成的是静态库。

可以通过指定`option`变量`BUILD_SHARED_LIBS`是否为`ON`和`OFF`，可以改变缺省后的默认行为。如果为`ON`则缺省后默认生成的为动态库。

更改`${CMAKE_ARCHIVE_OUTPUT_DIRECTORY}`和`${CMAKE_RUNTIME_OUTPUT_DIRECTORY}`和`${CMAKE_LIBRARY_OUTPUT_DIRECTORY}`可以改变库文件生成的路径

- `${CMAKE_ARCHIVE_OUTPUT_DIRECTORY}`为`.a`和`.lib`的生成路径
- `${CMAKE_RUNTIME_OUTPUT_DIRECTORY}`为``.dll`和`.exe`的生成路径
- `${CMAKE_LIBRARY_OUTPUT_DIRECTORY}`为`.so`的生成路径
