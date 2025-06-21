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

















