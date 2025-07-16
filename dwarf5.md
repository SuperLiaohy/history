# DWARF5

> 本文为阅读DWARF5官方文档的记录

## Introduction

dwarf在dwarf1时仅支持c语言，在dwarf2及其之后，便不再局限于特定语言了。

截至目前，dwarf5协议具有以下独立性

- 语言独立性

对于不同语言，既有他们共有属性，也有他们自己的特有属性

- 架构独立性
- 操作系统独立性
- 紧凑的数据表示
- 高效的处理
- 实现的独立性
- 避免重复的数据
- 利用其他的标准

比如会使用语言自己的标准。

- 有限的工具依赖

可以通过汇编器，链接器等支持

- 产商的可扩展性

## General Description

### The Debugging Information Entry (DIE)

DWARF使用了一系列的`DIE`来定义了源程序的`low-level representation`，每个`DIE`有一个定义的`TAG`和一系列的`attributes`

> The tag specifies the class to which an entry belongs and the attributes
> define the specific characteristics of the entry.