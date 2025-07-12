# Qt6

> 本文档记录本人学习Qt6的记录

> 本人环境`linux-mint22.1`，同时安装了`qt5`和`qt6`，都是使用`apt`软件源进行的二进制包安装的

## 安装Qt6

使用`apt`源进行`qt6`的二进制包安装

```bash
# 对qt6的安装
sudo apt install qt6-base-dev
# 安装qtcreator
sudo apt install qtcreator
```

这样就安装完了`qt6`的基本组件

```bash
# 检测qt6是否正确安装
qmake6 --version
```

## Qt主事件循环

`QCoreApplication`:为非GUI应用提供主事件循环

`QGuiApplication`:为GUI应用提供主事件循环

`QApplication`:为`Qt Widgets`模块的应用程序提供主事件循环

继承关系

`QApplication` $\rightarrow$ `QGuiApplication` $\rightarrow$ `QCoreApplication` $\rightarrow$ `QObject`

```c++
#include "mainwindow.h"

#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    MainWindow w;
    w.show();
    return a.exec(); // 进入主事件循环并等待，直到调用exit()。返回传递给exit()的值
}

```

## moc与QObject

> MOC为元对象编译器是一个预处理器

Qt对标准C++进行了扩展，引入了一些新的概念和功能。

- 信号槽机制
- 属性(动态添加成员)
- 内存管理
- ...

QObject类是所有使用元对象系统的类的基类，但是并不是所有Q开头的类的都是Object的派生类，例如QString

在一个类的`private`部分声明`Q_OBJECT`宏。

Qt为C++添加的特性在Qt Core中实现，由Qt的元对象系统实现，包括：信号与槽机制，属性系统，动态类型转化等。

元对象系统是一个C++扩展，使该语言更适合真正的组件话GUI编程。

处理流程

Qt源代码 $\rightarrow$ 预处理 $\rightarrow$ moc $\rightarrow$ 编译器 $\rightarrow$ 链接器 $\rightarrow$ 应用程序

QObject不支持拷贝

QObject的拷贝构造函数和赋值运算符都是私有的，并且使用了Q_DISABLE_COPY()宏。原因有很多，比如拷贝时的唯一标识符该如何处理，拷贝时的原对象和其他的关系如何处理，等等。

树状的内存管理系统。父级销毁，子级都会被销毁。

使用如下建立父子关系

```c++
class A:public QObject {
public:
    A(QObject *parent=nullptr):QObject(parent) {
        qInfo() << this << "被构造";
    };
    ~A(){qInfo()<< this << "被销毁";}
};


int main(int argc, char *argv[])
{

    A objA;
    objA.setObjectName("A");
    A *PA2=new A(&objA);
    PA2->setObjectName("A2");
    A *PA3=new A(PA2);
    PA3->setObjectName("A3");

    objA.dumpObjectTree();
}
```

```bash
QObject(0x7fff89fe0000) 被构造
QObject(0x61c4c39b6f30) 被构造
QObject(0x61c4c39b7960) 被构造
QObject::A 
    QObject::A2 
        QObject::A3 
QObject(0x7fff89fe0000, name = "A") 被销毁
QObject(0x61c4c39b6f30, name = "A2") 被销毁
QObject(0x61c4c39b7960, name = "A3") 被销毁
```

