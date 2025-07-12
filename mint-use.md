# Linux-mint

> 记录本人使用Linux-mint的一些历程，方便以后迅速拾起

## 首先下载好必备的软件

> 首要当然是`sudo apt update && sudo apt upgrade`更新系统

- 社交媒体：QQ，Wechat，bilibili
- 代码工具：vscode，clion，git，miniconda，typora
- 浏览器：edge
- 输入法：fcitx5
- 网络工具：clash-verge

### 配置好软件

- QQ https://im.qq.com/linuxqq/index.shtml
- Wechat https://linux.weixin.qq.com/
- bilibili https://github.com/msojocs/bilibili-linux/releases/
- vscode https://code.visualstudio.com/
- clion https://www.jetbrains.com/clion/download/?section=linux
- git `sudo apt install git`
- miniconda https://repo.anaconda.com/miniconda/
- typora https://typoraio.cn/
- edge https://www.microsoft.com/zh-cn/edge/download
- fcitx5

安装fcitx5

```bash
sudo apt install fcitx5 \
fcitx5-chinese-addons \
fcitx5-frontend-gtk3 fcitx5-frontend-gtk2 \
fcitx5-frontend-qt5 kde-config-fcitx5
```

安装中文词库

```bash
# 下载词库文件
wget https://github.com/felixonmars/fcitx5-pinyin-zhwiki/releases/download/0.2.4/zhwiki-20220416.dict
# 创建存储目录
mkdir ~/.local/share/fcitx5/pinyin/dictionaries/
# 移动词库文件至该目录
mv zhwiki-20220416.dict ~/.local/share/fcitx5/pinyin/dictionaries/
```

设置为默认输入法

````bash
im-config
````

设置环境变量

````bash
sudo vim /etc/profile

export XMODIFIERS=@im=fcitx
export GTK_IM_MODULE=fcitx
export QT_IM_MODULE=fcitx
````

设置拼音

```bash
fcitx5-configtool
# 然后选中其中的Pinyin
```

解决光标问题

```bash
fcitx5-configtool
# 然后选中上面状态栏的Global Options选项，然后划到最下面有个Show preedit in appllication选项，取消勾选即可
```

修改皮肤

```bash
git clone https://github.com/witt-bit/fcitx5-theme-macos12.git

# 深色
cp -r fcitx5-theme-macos12/macos12-dark /usr/share/fcitx5/themes/
# 亮色
cp -r fcitx5-theme-macos12/macos12-light /usr/share/fcitx5/themes/

# 然后就是到设置里修改
fcitx5-configtool
# 选中Addons中的第一项Classic User Interface
# 划到最小面就可以选择更改theme里
```

其他配置

```bash
fcitx5-configtool
# 选中Global Options选项，将Temporally switch between first and current input Method的快捷键从了Left Shift更改为Control+Space
```



## 配置DrClient开机自动启动

1. DrClientLinux下载

   需要连接JLU.PC的wifi然后必须**关闭梯子**，这样随便打开一个网站，便可以跳转到下载页面，点击下载ubuntu版的即可


2. DrClientLinux的开机自动启动

   因为DrClientLinux需要依赖他当前文件夹下的动态库，所以直接用DrClientLinux的autostart是没有用的，因为似乎是在全局下的，默认的搜索路径不包括DrClientLinux的目录，故可以写一个脚本，cd到DrClientLinux所在的目录后再在当前的目录下执行DrClientLinux，这样便可以成功。

3. DrClient的脚本

   ```bash
   (base) liaohy@liaohyHP:~$ cat /opt/DrClient/DrClient.sh 
   #!/bin/bash
   
   APP_DIR="/opt/DrClient"
   PROGRAM="./DrClientLinux"
   LOG_FILE="/opt/DrClient/DrClient.log"
   
   
   # 检查目录
   if [ ! -d "$APP_DIR" ]; then
   	echo "Error: Directory $APP_DIR does no exist" | tee -a "$LOG_FILE"
   	exit 1
   fi
   
   # 切换到指定目录
   cd "$APP_DIR" || {
   	echo "Error: Failed to change to directory $APP_DIR" | tee -a "$LOG_FILE"
   	exit 1
   }
   
   # 检查程序是否可执行
   if [ ! -x "$PROGRAM" ]; then
   	echo "Error: $PROGRAM is not executable or does not exist" | tee -a "$LOG_FILE"
   	exit 1
   fi
   
   # 执行程序并记录日志
   echo "Starting $PROGRAM at $(date)" >> "$LOG_FILE"
   "$PROGRAM" >> "$LOG_FILE" 2>&1
   
   # 检查程序执行结果
   if [ $? -eq 0 ]; then
   	echo "Program $PROGRAM executed successfully" >> "$LOG_FILE"
   else
   	echo "Error: Program $PROGRAM failed" >> "$LOG_FILE"
   	exit 1
   fi
   
   ```

4. 设置开机自动启动

   > 本来我是想通过systemd设置开机自动启动的，但是不知为何使用ai生成的自动启动service总是出现问题，必须我restart一遍才可以启动。

   这是service内容

   ```bash
   (base) liaohy@liaohyHP:~$ cat /etc/systemd/system/DrClient.service.bak 
   # Currently in a problem
   [Unit]
   Description=DrClient JLU WIFI
   After=network.target graphical.target
   
   [Service]
   Type=simple
   ExecStart=/opt/DrClient/DrClient.sh
   Restart=always
   User=liaohy
   WorkingDirectory=/opt/DrClient/
   StandardOutput=journal
   StandardError=journal
   Environment="DISPLAY=:0"  # 替换为实际的 DISPLAY 值
   ExecStartPre=/bin/sleep 10  # 延迟 10 秒
   [Install]
   WantedBy=multi-user.target
   
   
   ```

   目前的自动启动依赖于系统的**Startup Applications**这个应用。设置DrClient.sh的脚本开机自动启。



## 安装lazyvim

> 要保证nvim的版本在0.10.0以上。否则某些lazyvim的功能会支持不完全。

### 安装nvim

使用apt安装nvim的二进制包

如果直接安装二进制包，nvim的版本是比0.10.0低的（linux-mint22.1下），要获得高版本的nvim二进制包，可以加入nvim的unstable源

```bash
sudo add-apt-repository ppa:neovim-ppa/unstable
```

这样便可以下载高版本的nvim了，便可以安装lazyvim了。

### 下载lazyvim

```bash
# required
mv ~/.config/nvim{,.bak}

# optional but recommended
mv ~/.local/share/nvim{,.bak}
mv ~/.local/state/nvim{,.bak}
mv ~/.cache/nvim{,.bak}
```

```bash
git clone https://github.com/LazyVim/starter ~/.config/nvim
```

```bash
rm -rf ~/.config/nvim/.git
```

### 安装`nerd fonts`

下载`Nerd Fonts`，然后选择`JetBrainsMono Nerd Font`安装。解压后

````bash
# cd到解压后的目录中
sudo cp ./*.ttf /usr/share/fonts/
# 更新缓存
sudo fc-cache -fv
````

然后在终端的设置中选择`jetBrainsMono Nerd Font`，选择完后字体间距异常的话，重启终端即可。

## ros2的安装

> ros2在linux-mint的安装参考ubuntu一样
>
> 但是ros2无法像ubuntu一样直接使用小鱼的脚本执行，原因似乎是因为小鱼脚本会提前识别系统的发行版，mint发行版无法直接使用，虽然我感觉mint发行版和ubuntu一样，但是小鱼的脚本就是会拒绝执行，也许可以欺骗一下脚本让他识别为ubuntu，这样或许能行，不过目前没有尝试过

### 设置locale

先检查locale是否支持UTF-8的编码

不论是en_US.UTF-8还是zh_CN.UTF-8都是没有问题的，这两种只是输出形式上有差异，比如zh_CN的优先输出的是中文形式的（日期，地址等），而en_US的优先输出的是英文形式的（日期，地址）

输入`locale`便可以查看自己的编码格式了

这是grok给出的差异

#### 1. **语言（Language）**

- zh_CN.UTF-8：
  - 语言：中文（简体）。
  - 系统消息、程序界面（如错误提示、帮助文本）会优先显示中文（如果软件支持）。
  - 例如，命令行工具的输出可能是中文（如 ls 的错误消息）。
- en_US.UTF-8：
  - 语言：英语（美国）。
  - 系统消息和程序界面使用英语。
  - 例如，命令行工具的输出会是英语。

#### 2. **区域设置（Region）**

区域设置决定了与文化和地理相关的格式化规则，即使编码都是 UTF-8：

- 日期和时间格式（LC_TIME）：
  - zh_CN.UTF-8：日期格式通常为 YYYY-MM-DD（如 2025-06-22），时间可能为 HH:MM:SS。月份和星期名称用中文（如 “六月”、“星期日”）。
  - en_US.UTF-8：日期格式通常为 MM/DD/YYYY（如 06/22/2025），时间为 HH:MM:SS AM/PM。月份和星期名称用英语（如 “June”、“Sunday”）。
- 数字格式（LC_NUMERIC）：
  - zh_CN.UTF-8：小数点为 .，通常不使用千位分隔符（或使用逗号 ,，视软件而定）。
  - en_US.UTF-8：小数点为 .，千位分隔符为 ,（如 1,234.56）。
- 货币格式（LC_MONETARY）：
  - zh_CN.UTF-8：货币符号为 ¥（人民币），格式如 ¥1234.56。
  - en_US.UTF-8：货币符号为 $（美元），格式如 $1,234.56。
- 排序规则（LC_COLLATE）：
  - zh_CN.UTF-8：字符串排序基于中文拼音或笔画顺序（视实现而定）。
  - en_US.UTF-8：字符串排序基于英语字母顺序（a, b, c, ..., z）。
- 纸张尺寸（LC_PAPER）：
  - zh_CN.UTF-8：默认纸张为 A4（210x297mm，公制）。
  - en_US.UTF-8：默认纸张为 Letter（8.5x11英寸，英制）。
- 地址格式（LC_ADDRESS）：
  - zh_CN.UTF-8：地址顺序为国家-省-市-详细地址（如“中国北京市朝阳区”）。
  - en_US.UTF-8：地址顺序为详细地址-市-州-国家（如“123 Main St, New York, NY, USA”）。
- 电话格式（LC_TELEPHONE）：
  - zh_CN.UTF-8：国家代码为 +86，电话号码格式如 +86 123 4567 8901。
  - en_US.UTF-8：国家代码为 +1，电话号码格式如 +1 (123) 456-7890。
- 度量单位（LC_MEASUREMENT）：
  - zh_CN.UTF-8：使用公制单位（如米、千克）。
  - en_US.UTF-8：使用英制单位（如英里、磅），但在技术环境中也可能默认公制。
- 人名格式（LC_NAME）：
  - zh_CN.UTF-8：姓在前，名在后（如“张伟”）。
  - en_US.UTF-8：名在前，姓在后（如“John Smith”）。

#### 3. **字符编码（UTF-8）**

- 相同点：

  - 两者都使用 UTF-8 编码，支持多语言字符（包括中文、英文、特殊符号等）。
  - UTF-8 确保字符存储和显示一致，避免编码问题（如乱码）。
  - 对于 ROS 2 来说，UTF-8 编码是关键要求，因此两者在这方面对 ROS 2 都是兼容的。

- 不同点：

  - 编码本身没有区别，但语言和区域设置会影响字符的

    处理方式

    。例如：

    - zh_CN.UTF-8 可能优先处理中文字符的输入和显示（如拼音输入法）。
    - en_US.UTF-8 更适合英语输入和显示。

### 添加ros2的源

像Ubuntu这类的发行版的软件源分为：Main、Restricted、Universe 和 Multiverse。

要打开Universe源

> 但是似乎又不需要执行，好像20及以上版本的Ubuntu都默认打开了

```bash
sudo apt install software-properties-common
sudo add-apt-repository universe
```

添加ros2 GPG key

````bash
# 先下载必要依赖
sudo apt update && sudo apt install curl gnupg2 -y
# 添加GPG key
# sudo curl -sSL https://gitee.com/tyx6/rosdistro/raw/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg 该密钥似乎已经过期了，使用下面这个
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
````

选择一个源进行添加

```bash
# 官方源
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# 清华源
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] https://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

```

> 添加完源后记得更新源 

```bash
sudo apt update && sudo apt upgrade
```

### 安装ros2

```bash
# 我用的是Linux Mint 22.1 x86_64 ,是基于ubuntu24.04的发行版，其对应的是ros2版本是jazzy
sudo apt install ros-jazzy-desktop
# 也可以直接使用$ROS_DISTRO来自动下载对应的ros2版本
# sudo apt install ros-$ROS_DISTRO-desktop
```

把ros2的环境自动添加到环境变量里

```bash
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 使用miniconda的虚拟环境

````bash
# 如果是conda环境先退出conda环境
conda deactiave # 直到前面没有出现(base)或者其他环境字样
which python3 # 使用which判断python3是否指向系统
python3 --version # 判断系统的python3的版本型号
# 创建一个和系统python3一样的虚拟环境，因为ros2安装在系统里的故创建的虚拟环境最好和系统的python版本保持一致是最好的
conda create -n ros2_work python=3.12.3 # 使用conda创建一个名为ros2_work的python=3.12.3（这是我系统的python3环境）的虚拟环境
````

在conda的虚拟环境中引入ros2的库。

先查看在系统python下的搜索路径

```bash
liaohy@liaohyHP:~$ python3
Python 3.12.3 (main, Jun 18 2025, 17:59:45) [GCC 13.3.0] on linux
Type "help", "copyright", "credits" or "license" for more information.
>>> import sys
>>> print(sys.path)
['', '/opt/ros/jazzy/lib/python3.12/site-packages', '/usr/lib/python312.zip', '/usr/lib/python3.12', '/usr/lib/python3.12/lib-dynload', '/usr/local/lib/python3.12/dist-packages', '/usr/lib/python3/dist-packages', '/usr/lib/python3.12/dist-packages']
```

再查看conda的虚拟环境下的搜索路径

````bash
(ros2_work) liaohy@liaohyHP:~$ python3
Python 3.12.3 | packaged by Anaconda, Inc. | (main, May  6 2024, 19:46:43) [GCC 11.2.0] on linux
Type "help", "copyright", "credits" or "license" for more information.
>>> import sys
>>> print(sys.path)
['', '/opt/ros/jazzy/lib/python3.12/site-packages', '/home/liaohy/User/Env/miniconda3/envs/ros2_work/lib/python312.zip', '/home/liaohy/User/Env/miniconda3/envs/ros2_work/lib/python3.12', '/home/liaohy/User/Env/miniconda3/envs/ros2_work/lib/python3.12/lib-dynload', '/home/liaohy/User/Env/miniconda3/envs/ros2_work/lib/python3.12/site-packages']
````

可以发现少了

````bash
'/usr/lib/python312.zip', '/usr/lib/python3.12', '/usr/lib/python3.12/lib-dynload', '/usr/local/lib/python3.12/dist-packages', '/usr/lib/python3/dist-packages', '/usr/lib/python3.12/dist-packages'
````

而这其中只需要加入`/user/local/lib/python3.12/dist-packages`,`/user/lib/python3/dist-packages`这两个，

然后加上ros2的库`/opt/ros/jazzy/lib/python3.12/site-packages`，即可。

 而给python添加自定义的路径有很多种方法，我使用其中一种是，在对应的python目录下的`./lib/python3.12/site-packages`下添加一个sitecustomize.py文件，来实现自定义路径。

```python
import sys
sys.path.extend([
    '/opt/ros/jazzy/lib/python3.12/site-packages',
    '/usr/local/lib/python3.12/dist-packages',
    '/usr/lib/python3/dist-packages'
])

```

这样便可以直接在虚拟环境里使用ros2了，要下载的库都可以下载到虚拟环境里，这样就可以避免把python库下载到全局。同时可以直接使用ros2的库和系统的python库。无需担心其他版本或者库问题。

### 安装colcon

````bash
# 激活conda的虚拟环境，在其中下载colcon
conda activate ros2_work
pip install colcon-common-extensions
````

## 安装matlab2025a

> 来自官方安装方法

> 本方法只测试过matlab2025a（截至我写的时候最高的），其他更低版本在linux上非xfce的桌面环境下都有点问题。还需更多操作。

To install MATLAB on Linux:

1. From [MathWorks Downloads](https://www.mathworks.com/downloads/), select a MATLAB release and download the installer.

2. Unzip the downloaded installer files and navigate to the unzipped folder. For example, use these commands to unzip the installer for release `R2025a` to a folder of the same name, and then navigate to the folder.

   ```shell
   unzip matlab_R2025a_Linux.zip -d ./matlab_R2025a_Linux
   cd ./matlab_R2025a_Linux
   ```

3. In the installation folder, run the `install` script and follow the prompts to complete the installation.

   > `xhost +SI:localuser:root`整体含义
   >
   > - 该命令允许本地的 root 用户通过 X 服务器运行图形界面的应用程序。例如，当以 root 用户身份运行需要显示图形界面的程序（如 gksu 或某些图形化配置工具）时，X 服务器需要明确授权 root 用户访问当前的显示服务器。
   > - 默认情况下，X 服务器可能只允许当前登录的用户访问图形界面，root 用户可能被拒绝访问（出于安全考虑）。运行 xhost +SI:localuser:root 可以临时授予 root 用户访问权限。
   >
   > `sudo -H`具体含义
   >
   > - **默认行为**：当你运行 sudo command 时，sudo 会保留当前用户的 HOME 环境变量（例如，普通用户的 /home/username）。这可能导致问题，因为某些程序会尝试在当前用户的家目录中读取或写入配置文件，而 root 用户可能没有权限访问普通用户的家目录，或者你不希望 root 操作普通用户的配置文件。
   > - **-H 选项的作用**：sudo -H 明确地将 HOME 环境变量设置为目标用户的家目录（例如，/root 对于 root 用户），确保命令以目标用户的家目录为上下文运行。

   ```shell
   xhost +SI:localuser:root
   sudo -H ./install
   xhost -SI:localuser:root
   ```

   `sudo` is required only when you install products to a folder where you do not have write permissions, which might include the default installation folder. The `xhost` commands are required only when you install products as the root user with `sudo`. These commands temporarily give the root user access to the graphical display required to run the installer.

   Default installation folder: `/usr/local/MATLAB/R20XXy`

To start MATLAB after the installation is complete, see [Start MATLAB on Linux Platforms](https://www.mathworks.com/help/matlab/matlab_env/start-matlab-on-linux-platforms.html) (MATLAB).

安装完后如果发现有matlab打开有异常，重启电脑可能能解决（本人是这样的）

目前似乎有点bug是`matlab2025a`无法使用`fcitx5`，不清楚是`matlab2025a`的bug还是，`fcitx5`的bug，还是`linux-mint22.1`的bug。
