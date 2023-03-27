# ele_gun
传统FPS（✗）

电子FPS（✓）

自制esp32体感手柄，FPS赛博打枪神器，取名ele_gun。

## 文件夹介绍
examples：开发过程中的例程，最喜欢的一环。

software：源码，能用，但屎山。

hardware：硬件原理图及pcb（空）

images：图

## 已完成功能

#2023/3/27

摇杆 / 体感双模式（对应速度和加速度）

基本键位的映射（模拟键盘和鼠标）

蓝牙连接

mpu6050_DMP滤波

#2023/3/24

例程全验证

## 预备更新

代码优化

RGB

屏幕

陀螺仪误差补偿

更多键位映射

电磁后坐（手柄有后坐，打枪更快乐）

freertos（彻底解决delay与ble速率的问题）

oop（开放api，有手就能开发）

pcb

模型

。。。。。


## 说明

由于ble_keyboard和ble_mouse共存会出bug，因此需要导入第三方库combo。

## 参考：

https://github.com/ServAlex/ESP32-BLE-Combo

https://github.com/T-vK/ESP32-BLE-Mouse

https://mc.dfrobot.com.cn/thread-308719-1-1.html

https://www.yiboard.com/forum.php?mod=viewthread&tid=1195&highlight=%E5%8A%A0%E9%80%9F%E5%BA%A6

# *make by xiaocainiao* 
