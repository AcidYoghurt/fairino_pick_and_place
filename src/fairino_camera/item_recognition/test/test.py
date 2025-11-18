import cv2
import numpy as np

# 模拟光照下的正方体图像（假设左侧光）
image = cv2.imread("cube.jpg")
gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

# 使用Canny检测边缘
edges = cv2.Canny(gray, threshold1=50, threshold2=150)

# 提取直线（可选）
lines = cv2.HoughLinesP(edges, 1, np.pi/180, threshold=50, minLineLength=10, maxLineGap=10)
for line in lines:
    x1, y1, x2, y2 = line[0]
    cv2.line(image, (x1, y1), (x2, y2), (0, 255, 0), 2)

cv2.imshow("Edges", edges)
cv2.waitKey(0)