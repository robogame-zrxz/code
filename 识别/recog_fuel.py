import cv2
import serial
import numpy as np

def find_fuel(frame):
    # 转换为灰度图像
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # 运行Canny边缘检测
    edges = cv2.Canny(gray, threshold1=50, threshold2=150)

    # 寻找轮廓
    contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # 遍历轮廓，找到正方形
    for contour in contours:
        approx = cv2.approxPolyDP(contour, 0.04 * cv2.arcLength(contour, True), True)
    if len(approx) == 4:
        x, y, w, h = cv2.boundingRect(approx)
    if abs(w - h) < 10:  # 确保宽高相近，以避免误判矩形为正方形
        center_x = x + w // 2
        center_y = y + h // 2
    # 输出燃料矿物的中心位置
    return [center_x, center_y]



# 创建一个串行通信对象
ser = serial.Serial("/dev/ttyS0", 9600)
# 创建一个视频捕获对象
cap = cv2.VideoCapture(0)
# 读取图像
while True:
    ret, frame = cap.read()  # 读取一帧图像

    if not ret:
        break

# 识别燃料矿
result = find_fuel(frame)

cap.release()
cv2.destroyAllWindows()
