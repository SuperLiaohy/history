## Buildroot学习记录

## 启动流程

linux启动流程

bootloader引导程序 $\rightarrow$ Linux内核 $\rightarrow$ 根文件系统 $\rightarrow$ App 

编译嵌入式Linux系统

在`Cross-Compiling Toolchain`编译下

`Bootloader` $\rightarrow$ `u-boot.bin`

`Kernel` $\rightarrow$ `ulmage + dtb`

`Rootfs` $\rightarrow$ `/etc /lib /sbin`

`App` $\rightarrow$ `App(elf)`

`u-boot.bin`和`ulmage + dtb`和`/etc /lib /sbin`和`App(elf)`组成`Embedded Linux Image`

## 工具链

对于`Host PC`应该有

- `Cross Compiler`：在平台A上能使用它生成程序，这个程序运行时在平台B上。优点有很多，其中之一：将构建环境和目标环境分开，因为嵌入式计算机的资源非常有限，有时在本地上无法或者很难提供编译环境，或者编译环境非常差，我们便使用交叉编译器，在`Host PC`上使用交叉编译。
- `JTAG Debugger：`
- `Source Code`

`Toolchain tuple`定义：

CPU架构，操作系统，芯片厂商，ABI，C库

- `<arch>-<vendor>-<os>-<libc/abi>`(完整名称)
- `<arch>-<os>-<libc/abi>`

`<arch>`指CPU架构:arm,mips,powerpc,i386,i686等

`vendor`自定义格式的字符串(大部分),被autoconf忽略

`<os>`操作系统

`<libc/abi>`C库和使用中的ABI的组合

比如下面几个例子

- arm-foo-none-eabi 为针对ARM架构的裸机工具链，供应商为foo。
- arm-unknown-linux-gnueabihf 为针对ARM架构的Linux工具链，供应商未知，使用的EABIhf ABI和glibc C库。
- armeb-linux-uclibcgnueabi 为针对ARM big-endian 的 Linux 工具链架构，使用EABI ABI和 uClibc C库
- mips-img-linux-gnu 为针对MIPS架构的Linux工具链，使用glibc C库，有Imagination Technologies提供

`ABI`的分类

- OABI：过时的ABI。强制使用硬浮点指定，这需要在内核中模拟浮点运算。已不再任何地方支持
- EABI：由ARM标准化。允许将硬浮点代码与软浮点代码混合。在整数寄存器中传递浮点参数。
  - 硬浮点代码：直接使用浮点指令。
  - 软浮点代码：使用gcc提供的用户空间库模拟浮点指定
- EABIhf：也由ARM标准化。需要一个专门的浮点单元：只有硬浮点代码。在浮点寄存器中传递的浮点参数。
- gcc选项
  - EABI soft-float：	-mabi=aapcs-linux -mfloat-abi=soft
  - EABI hard-float：      -mabi=aapcs-linux -mfloat-abi=softfp
  - EABIhf：                     -mabi=aapcs-linux -mfloat-abi=hard

对于`<os>`的两个重要的值

- `none`用于裸机工具链`bare-metal toolchains`
  - 用于没有操作系统的开发
  - 使用的C库一般是newlibc
  - 提供不需要操作系统的C库服务
  - 允许为特定的硬件目标提供基本的系统调用
  - 可用于构建`bootloader`程序或`Linux kernel`，不能用于构建Linux用户空间代码
- `linux`为linux工具链`Linux toolchains`
  - 用于Linux操作系统开发。
  - 选择特定于Linux的C库：glibc，uclibc，musl
  - 支持Linux系统的调用
  - 可用与构建Linux用户空间代码，也可用于构建引导加载程序或内核本身等裸机代码。

Multilib工具链包含多个sysroot，每个都有一个针对不同架构ABI变体的目标库版本。

工具链和SDK的区别，工具链只有编译器，binutils和C库。而SDK为一个工具链，加上一些为Target目标架构构建的库头文件，以及在构建软件时有用的其他本地工具。

像Yocto和Buildroot等构建系统通常可以：

- 使用现有工具链作为输入，或苟安近自己的工具链
- 除了使用根文件系统外，他们还可以生成SDK以允许应用程序开发人员为目标构建应用程序/库

## Boot-loader

Boot-looader引导加载程序，是一种计算机程序，负责引导计算机系统启动。

- BIOS
- Uboot
- GRUB

一般存储在ROM里面，比如NandFlash，NorFlash，HardDisk。

NandFlash：不能直接运行代码，必须先加载到RAM中才能运行代码。

NorFlash：可以直接运行代码。

常见的W25Q64就是NorFlash支持字级别的随机读取，可以直接运行代码。

`Boot-loader`的主要功能有如下：

- 初始化CPU并设置时钟频率。
- 屏蔽所有中断
- 设置栈指针
- 启用电源管理
- 将linux内核和根文件系统的镜像加载到系统内存中，然后启动内核。
- 具备将数据写入闪存的能力，支持下载Linux内核，升级`bootloader`自身等功能

`Boot Loader`的启动过程通常分为两个阶段，如下：

1. 第一阶段
   - 初始化硬件组件
   - 为加载第二阶段程序准备内存空间
   - 将第二阶段程序复制到内存空间
   - 设置sp（栈指针）
   - 跳转到第二段程序入口点
2. 第二阶段
   - 初始化此阶段使用的硬件组件
   - 检查内存映射
   - 将内核和根文件系统映像复制到内存中
   - 设置启动参数
   - 启动内核

## Kernel

构建Kernel步骤：

1. 获取配套的交叉编译工具链
2. 下载kernel源码
3. Host下配置开发环境
   - 安装必要依赖包
   - 解压配置合适的工具链
4. 指定编译板子配置。
5. 编译
   - 编译内核镜像
   - 编译设备树
   - 编译安装模块驱动

Kernel源码目录的作用

- `arch`包含体系结构的文件。子目录下有不同的架构。
- `Documentation`包含内核文档。如果您想找到有关Linux某个方面的更多信息，可以直接从这里寻找
- `drivers`包含设备驱动出现，数以千计。有一个每种驱动程序的子目录。
- `fs`包含为文件系统代码
- `include`包含内核头文件，包括那些工具链需要的文件。
- `init`包含内核启动代码
- `kernel`包含核心功能，包括调度，锁，定时器，电源管理和调试/代码跟踪。
- `mm`包含网络协议
- `scripts`包含许多游泳的脚本，包括设备树编译器(DTC)等。
- `tools`包含许多有用的工具，包括Linux性能计数器工具perf，分析和追踪等。
