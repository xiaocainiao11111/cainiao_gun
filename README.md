# cainiao_gun
>传统FPS（✗）

>电子FPS（✓）

主控：esp32-woroom-32E

平台：vscode-platformio


自制体感手柄，赛博FPS打枪神器，取名cainiao_gun。

## 文件夹介绍
- examples：开发过程中的例程，大部分已验证。

- software：源码，能用，但屎山。

- hardware：硬件原理图及pcb（空）

- images：图

# 已完成功能

### **2023/3/30**

- 代码实现:
        
        RGB

- 新增例程：
        
        RGB_test1//完整版
        RGB_test2//使用部分
        PWM_test1//电磁铁接mos导通测试

### **2023/3/27**

- 代码实现：
        
        1、摇杆 / 体感双模式（对应速度和加速度）

        2、基本键位的映射（模拟键盘和鼠标）

        3、蓝牙连接（低功耗ble）

        4、mpu6050_DMP滤波

### **2023/3/24**

- 新增例程

## 后续更新

- 代码优化

- 屏幕

- 陀螺仪误差补偿

- 更多键位映射

- 电磁后坐（手柄有后坐，打枪更快乐）

- freertos（彻底解决delay与ble速率的矛盾）

- oop

- pcb

- 模型

。。。。。


## 说明

由于ble_keyboard和ble_mouse共存会出bug，因此需要导入第三方库combo。

*****
*****

## 参考：

https://github.com/ServAlex/ESP32-BLE-Combo

https://github.com/T-vK/ESP32-BLE-Mouse

https://mc.dfrobot.com.cn/thread-308719-1-1.html

https://www.yiboard.com/forum.php?mod=viewthread&tid=1195&highlight=%E5%8A%A0%E9%80%9F%E5%BA%A6

# *make by xiaocainiao* 
