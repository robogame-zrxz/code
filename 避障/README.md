# 避障方案

1. 到达定位点开启避障模块
2. 使用树莓派外接的摄像头捕捉图像
3. 调用函数，使用canny边缘检测并计算箱子的距离
4. 判断箱子和陨石区的间隙，决定通过的路线，并将横移距离保存
5. 通过两次陨石后，利用横移距离回到终点进入采矿区
6. 返回时使用两次记录的横移距离原路返回