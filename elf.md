# ELF文件解析

> 本文侧重介绍dwarf标准，也许称为dwarf标准解析更好

## ELF格式

1. ELF(文件)头
2. 段头表：仅限可执行的目标文件
3. 节头表：包含一些列大小固定的记录，每个节section对应其中的一条记录，包括每个节在文中的偏移，大小等。 

## dwarf标准

### 什么是DWARF?

- DWARF 第一版发布于 1992 年, 主要是为UNIX下的调试器提供必要的调试信息，例如PC地址对应的文件名及行号等信息，以方便**源码级调试**
- 其包含足够的信息以供调试器完成特定的一些功能, 例如显示当前栈帧(Stack Frame)下的局部变量, 尝试修改一些变量, 直接跳到函数末尾等
- 有足够的可扩展性，可为多种语言提供调试信息: 如: Ada, **C**, **C++**, Fortran, Java, **Objective C**, **Go**, **Python**, Haskell ...
- 除了编译/调试器外，还可用于从运行时地址还原源码对应的符号|行号的工具(如: atos)

以下以文件`foo.c`程序代码作为案例来分析。

```bash
(base) liaohy@liaohyHP:~/User/Tmp/dwarf_test$ cat foo.c
int foo(int a, int b) {
  int c;
  static double d = 5.0;
  c = a + b;
  return c;
}

int main(int argc, char *argv[])
{
  int r;
  r = foo(2, 3);
  return 0;
}
```

可以使用`size`工具查看`elf`文件的`Segment`和`Section`。例如：

```bash
(base) liaohy@liaohyHP:~/User/Tmp/dwarf_test$ size -A foo
foo  :
section              size    addr
.interp                28     792
.note.gnu.property     48     824
.note.gnu.build-id     36     872
.note.ABI-tag          32     908
.gnu.hash              36     944
.dynsym               144     984
.dynstr               136    1128
.gnu.version           12    1264
.gnu.version_r         48    1280
.rela.dyn             192    1328
.init                  27    4096
.plt                   16    4128
.plt.got               16    4144
.text                 307    4160
.fini                  13    4468
.rodata                 4    8192
.eh_frame_hdr          52    8196
.eh_frame             180    8248
.init_array             8   15856
.fini_array             8   15864
.dynamic              448   15872
.got                   64   16320
.data                  24   16384
.bss                    8   16408
.comment               43       0
.debug_aranges         48       0
.debug_info           250       0
.debug_abbrev         179       0
.debug_line            79       0
.debug_str            219       0
Total                2705

```



