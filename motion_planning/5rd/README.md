### BIVP
代码位置[here](lec5_hw/src/click_gen.cpp#L74)  
步骤:  
* 1.初始化F0,及初始位置，速度，加速度
* 2.循环初始化Ei,Fi。确保必须过某个航点，且在航点上速度，加速度，jerk,sanp连续性，即求导为0    
* 3.初始化Em点,及末尾的位置，速度，加速度
* 4.求逆
  
图片效果  
![01.png](images/01.png)
视频效果  
![01.gif](images/01.gif)
