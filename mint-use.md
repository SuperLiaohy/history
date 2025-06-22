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















