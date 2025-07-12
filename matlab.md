# Matlab杂记

> 本文为记录我使用matlab中碰到的一些常用用法。主要以例子通过引导。

## 删除数组中的某个值

如果是删除特定的值的话，比如删除某个数组中的所有为3的值。

```matlab
A = [1, 2, 3, 4, 3, 5];
A = A(A ~= 3); % 删除所有值为 3 的元素
disp(A); % 输出: [1, 2, 4, 5]
```

使用索引删除指定位置的值

删除单个索引

```matlab
A = [1, 2, 3, 4, 5];
A(2) = []; % 删除第 2 个元素
disp(A); % 输出: [1, 3, 4, 5]
```

删除多个索引

```matlab
A = [1, 2, 3, 4, 5];
A([2, 4]) = []; % 删除第 2 和第 4 个元素
disp(A); % 输出: [1, 3, 5]
```

删除满足条件的值

```matlab
A = [1, 2, 3, 4, 5];
A(A > 3) = []; % 删除大于 3 的元素
disp(A); % 输出: [1, 2, 3]
```

删除二维数组的整行或整列

```matlab
A = [1, 2, 3; 4, 5, 6; 7, 8, 9];
A(2, :) = []; % 删除第 2 行
disp(A); % 输出: [1, 2, 3; 7, 8, 9]

A = [1, 2, 3; 4, 5, 6; 7, 8, 9];
A(:, 2) = []; % 删除第 2 列
disp(A); % 输出: [1, 3; 4, 6; 7, 9]
```

## 对符号表达式的化简

使用`simplify`对表达式进行化简

```matlab
simplified_expr = simplify(expr, 'Steps', 50); % 增加化简步数
```

## 为符号表达式添加约束

使用`assume`提供约束

```matlab
syms x
% 添加范围约束
assume(x > 0); % 约束 x 为正数
% 添加等式约束
assume(x + y == 1); % 设置约束 x + y = 1
expr = x^2 + 2*x + 1;
simplified_expr = simplify(expr); % 化简
disp(simplified_expr); % 输出: (x + 1)^2
```

可以使用`syms <符号变量> clear`来清除之前`assume`提供的约束。

```matlab
syms x clear % 清除 x 的所有约束
```

可以使用`assumption`函数来查看当前变量的约束

```matlab
syms kx ky kz; 
assume(kx^2+ky^2+kz^2==1)
assumptions(kx) % 查看有关 kx 的所有约束
% 输出
%ans =
%
%kx^2 + ky^2 + kz^2 == 1
```



## 使用具体的约束代入符号表达式

使用`subs`函数来将约束带入表达式中

```matlab
syms x y
expr = x^2 + 2*x*y + y^2;
constrained_expr = subs(expr, y, 1-x); % 代入约束 y = 1-x
simplified_expr = simplify(constrained_expr); % 化简
disp(simplified_expr); % 输出: 1
```

## 解方程

面对更为复杂的约束时，我们就可以考虑使用方程组来解决

使用`slove`可以用来求解系统

```matlab
syms x y
eqns = [x^2 + y^2 == 5, x - y == 1];
vars = [x y];
[x, y] = solve(eqns, vars) 
% 输出
%x =
% 
%-1
% 2
% 
% 
%y =
% 
%-2
% 1
```

## 提取公因式

对于简单的表达式，可以直接使用`factor`来对多项式进行分解

```matlab
syms x y
expr = x^2 + x*y + x; % 表达式
factored_expr = factor(expr); % 因式分解
disp(factored_expr); % 输出: [x, x + y + 1]
```

使用`collect`函数可以按变量整理公因式。

````matlab
syms x y
expr = x^2 + x*y + 2*x + y; % 表达式
collected_expr = collect(expr, x); % 按 x 提取公因式
disp(collected_expr); % 输出: x*(x + y + 2) + y
% 可以指定多个变量，例如 collect(expr, [x, y])，但通常按单一变量更清晰。
% 还可以使用 collect(expr, sin(x)) 提取 sin(x)的多项式
````

## 批量定义一系列的符号变量

定义符号数组

```matlab
n = 5; % 定义数组长度
x = sym('x', [1, n]); % 创建符号数组 x1, x2, ..., x5
disp(x); % 输出: [x1, x2, x3, x4, x5]
```

定义二维符号数组

```matlab
m = 2; n = 3; % 定义矩阵维度
A = sym('a', [m, n]); % 创建 2x3 符号矩阵 a
disp(A); % 输出: [a1_1, a1_2, a1_3; a2_1, a2_2, a2_3]
```

自定义批量符号变量，可不使用默认的名称

```matlab
x = sym('x_%d', [1, 4]); % 创建 x_1, x_2, x_3, x_4
disp(x); % 输出: [x_1, x_2, x_3, x_4]
```

