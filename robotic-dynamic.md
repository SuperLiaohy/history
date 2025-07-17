# robotic dynamic

> 本文为回顾机械臂动力学建模的文档记录

> 建议再学动力学之前有比较好的运动学基础，这样有便于理解相关概念。

> 本文的资料来源基本为b站视频`台大机器人学之动力学--林沛群`                                             

## Time-varying position and orientation

从速度的定义出发

${}^{B}V_Q = \frac{\mathrm{d}}{\mathrm{d}t} {}^{B}P_Q = \lim_{\Delta{t} \to 0} \frac{{}^{B}P_Q(t+\Delta{t})-{}^{B}P_Q(t)}{\Delta{t}}$ 

${}^{A}({}^{B}V_Q)={}^{A}(\frac{\mathrm{d}}{\mathrm{d}t}{}^{B}P_Q)$

${}^{A}_B{R}({}^{B}V_Q)={}^{A}({}^{B}V_Q)$  
原点处的，速度可以简写成如下，其中U表示大地坐标系  
$v_c = {}^{U}V_{CORG}$  
同理角速度也可以简写成如下  
$w_c = {}^{U}\Omega_C$  

## Rigid body motion  


定义$\hat{I}$和$\hat{J}$这种大写带头的为大地坐标系的单位向量。  
而$\hat{i}$和$\hat{j}$这种小写带头的可以理解为固连在关节上的坐标系，总之就是会边的坐标系。  

对于大地坐标系来说  
大地坐标系是不动的，故对其单位向量求导，其为0。  
故$\dot{\hat{I}}=0$,

对于被固连的坐标系
其$|\Delta\hat{i}|=\mathrm{d}\theta\sin\phi|\hat{i}|$  
$|\dot{\hat{i}}|=\frac{\mathrm{d}\hat{\theta}}{\mathrm{d}t}\sin\phi|\hat{i}|$
而方向又是垂直$\vec{w}$和$\hat{i}$且符合右手螺旋定则  
故$\dot{\hat{i}}=\vec{w} \times \hat{i}$

$$
\begin{align*}
\vec{r_A} &= \vec{r}_{B} + \vec{r}_{A/B}  \\
          &= (X_B\hat{I} + Y_B\hat{J}) + (x_{A/B}\hat{i} + y_{A/B}\hat{j})
\end{align*}
$$
对两边求导

$$
\begin{align*}
\dot{\vec{r_A}} &= \dot{\vec{r}}_{B} + \dot{\vec{r}}_{{A/B}}  \\
        \vec{v_A}  &= (\dot{X_B}\hat{I} + X_B\dot{\hat{I}} + \dot{Y_B}\hat{J} + Y_B\dot{\hat{J}}) + (\dot{x}_{A/B}\hat{i} + x_{A/B}\dot{\hat{i}} + \dot{y}_{A/B}\hat{j} + y_{A/B}\dot{\hat{j}}) \\
        &= (\dot{X_B}\hat{I} + \dot{Y_B}\hat{J}) + (\dot{x}_{A/B}\hat{i} + x_{A/B}(\vec{\omega} \times \hat{i}) + \dot{y}_{A/B}\hat{j} + y_{A/B}(\vec{\omega} \times \hat{j})) \\
        &= (\dot{X_B}\hat{I} + \dot{Y_B}\hat{J}) + (\dot{x}_{A/B}\hat{i}  + \dot{y}_{A/B}\hat{j}) + \vec{\omega} \times (x_{A/B}\hat{i} + y_{A/B}\hat{j}) \\
        &= (\dot{X_B}\hat{I} + \dot{Y_B}\hat{J}) + (\dot{x}_{A/B}\hat{i}  + \dot{y}_{A/B}\hat{j}) + \vec{\omega} \times (X_{A/B}\hat{I} + Y_{A/B}\hat{J})   \\
        &= \vec{v_B} + \vec{v}_{{A/B}_{rel}} + \vec{\omega} \times \vec{r}_{A/B}
\end{align*}
$$
因此可得出结论   
${}^{A}V_Q={}^{A}V_{BORG}+{}^{A}_BR{}^{B}V_Q+{}^{A}\Omega_B \times {}^{B}P_Q$  
> 注意这里为向量的坐标相加，故要将所有向量的坐标都变换到统一坐标系下，才能进行相加。特别要注意这一点，之后的公式都是基于此。此外还要区分向量的坐标和向量之间的区别。

## Velocity propagation  
前后两个连杆固连的坐标系间速度和角速度的关系
既${}^{i}\omega_{i}$,${}^{i}v_{i}$和${}^{i+1}\omega_{i+1}$,${}^{i+1}v_{i+1}$之间的关系。  
定义：  
$\dot{\theta}_{i+1}$为第$i+1$个关节的关节旋转速度  
$\dot{\theta}_{i}$为第$i$个关节的关节旋转速度  
故对于旋转关节有如下关系  
角速度  
$$
\begin{align*}
{}^{i}\omega_{i+1}&={}^{i}\omega_i+{}^{i}_{i+1}R\dot{\theta}_{i+1}{}^{i+1}\hat{Z}_{i+1}  \\
{}^{i+1}_{i}R{}^{i}\omega_{i+1}&={}^{i+1}_{i}R{}^{i}\omega_i+{}^{i+1}_{i}R{}^{i}_{i+1}R\dot{\theta}_{i+1}{}^{i+1}\hat{Z}_{i+1}  \\
{}^{i+1}\omega_{i+1}&={}^{i+1}_{i}R{}^{i}\omega_i+\dot{\theta}_{i+1}{}^{i+1}\hat{Z}_{i+1}  \\
\end{align*}
$$  
速度  
$$
\begin{align*}
{}^{i}v_{i+1}&={}^{i}v_i + {}^{i}\omega_{i} \times {}^{i}P_{i+1}  \\
{}^{i+1}_{i}R{}^{i}v_{i+1}&={}^{i+1}_{i}R{}^{i}v_i+{}^{i+1}_{i}R({}^{i}\omega_i \times {}^{i}P_{i+1})  \\
{}^{i+1}v_{i+1}&={}^{i+1}_{i}R{}^{i}v_i+{}^{i+1}_{i}R({}^{i}\omega_i \times {}^{i}P_{i+1})  \\
% {}^{i+1}v_{i+1}&={}^{i+1}_{i}R{}^{i}v_i +   \\
\end{align*}
$$  
这些所给的公式都是从$i+1$的物理量是由$i$的物理量传导过来然后再加上$i+1$坐标原点自有的物理量得出的。  
由此可见在递推中速度和角速度都是外推的，由内向外递推。因为我们获取的是每个关节本身的旋转速度，这导致我们只能由内向外推。因为依据公式最内部的关节物理量耦合最少只和自身有关。而最外部的关节的物理量耦合最多需要一层一层递推下来。  
这样的向外传导决定了速度和角速度这两个物理量在机械臂的递推中属于外推的。  

对于平移关节而言，亦可根据其物理量在外推过程中传导的特点列出。这里自给出最后关系，推理同上，故不写了。  
> 提醒，这里的速度外推这里使用Z轴是因为，我们在MDH建模的时候会把驱动方向设为Z轴，故上文旋转关节也是用的是Z轴。故这里i+1的本身坐标系原点速度方向也用的是Z轴方向。
$$
\begin{align}
{}^{i+1}\omega_{i+1}&={}^{i+1}_{i}R{}^{i}\omega_{i}  \\
{}^{i+1}v_{i+1}&={}^{i+1}_{i}R({}^{i}v_i + {}^{i}\omega_{i} \times {}^{i}P_{i+1}) + \dot{d}_{i+1}{}^{i+1}\hat{Z}_{i+1}  \\
\end{align}
$$

## Jacobians
> 夹克比矩阵这一块。。。  

夹克比矩阵用于将一个空间的量转化为另一个空间的量  
比如可以将角度转化为力矩的夹克比矩阵，角度转化为速度的夹克比矩阵

## Static forces

对于力的传导来说  
$$
\begin{align*}
    {}^{i}f_i &= {}^{i}f_{i+1}  \\
    {}^{i}f_i &= {}^{i}_{i+1}R{}^{i+1}f_{i+1}   \\
\end{align*}
$$
对于力矩的传导来说
$$
\begin{align*}
    {}^{i}n_i &= {}^{i}n_{i+1} + {}^{i}P_{i+1} \times {}^{i}f_{i+1} \\
    {}^{i}n_i &= {}^{i}_{i+1}R{}^{i+1}n_{i+1} +  {}^{i}P_{i+1} \times {}^{i}f_{i+1} \\
\end{align*}
$$

这些所给的公式都是从$i$的物理量是由$i+1$的物理量传导过来然后再加上$i$坐标原点自有的物理量得出的。  
由此可见在递推中力和力矩都是内推的，由外向内推。因为我们获取的是每个关节本身的力矩，这导致我们只能由外向内推。因为依据公式最外部的关节物理量耦合最少只和自身有关。而最内部的关节的物理量耦合最多需要一层一层递推下来。  
这样的向外传导决定了力和力矩这两个物理量在机械臂的递推中属于内推的。  

而对于旋转关节要输出的力矩则为
$\tau_i={}^{i}n_i^T{}^{i}\hat{Z}_i$

