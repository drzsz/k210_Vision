import sensor
import image
import lcd
import math

# 初始化LCD显示屏
lcd.init()

# 初始化摄像头
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.set_vflip(True)
sensor.run(1)

# 缩放图像
def scale_image(img, scale):
    width = img.width() // scale
    height = img.height() // scale
    img_scaled = img.resize(width, height)
    return img_scaled

while True:
    # 获取摄像头图像
    img = sensor.snapshot()

    # 缩放图像
    img_scaled = scale_image(img, 2) # 将图像缩小为原来的一半

    # 在缩放后的图像中查找矩形
    rects = img_scaled.find_rects(threshold=50000)

    # 将矩形坐标放大到原始图像尺寸
    for r in rects:
        r_scaled = [r.x() * 2, r.y() * 2, r.w() * 2, r.h() * 2]

        # 估计矩形的旋转角度
        width = r_scaled[2]
        height = r_scaled[3]
        rotation = math.degrees(math.atan(height) + math.atan(width))

        # 绘制矩形
        img.draw_rectangle(r_scaled, color=(255, 0, 0), thickness=2)

        # 输出矩形的四个点的坐标
        x = r_scaled[0]
        y = r_scaled[1]
        w = r_scaled[2]
        h = r_scaled[3]
        cx = x + w // 2
        cy = y + h // 2

        # 计算矩形的四个顶点
        angle = math.radians(rotation)
        cos_angle = math.cos(angle)
        sin_angle = math.sin(angle)
        x1 = int(cx + w / 2 * cos_angle - h / 2 * sin_angle)
        y1 = int(cy + w / 2 * sin_angle + h / 2 * cos_angle)
        x2 = int(cx - w / 2 * cos_angle - h / 2 * sin_angle)
        y2 = int(cy - w / 2 * sin_angle + h / 2 * cos_angle)
        x3 = int(2 * cx - x1)
        y3 = int(2 * cy - y1)
        x4 = int(2 * cx - x2)
        y4 = int(2 * cy - y2)

        # 绘制矩形的四个顶点
        img.draw_circle(x1, y1, 5, color=(0, 255, 0), thickness=2)
        img.draw_circle(x2, y2, 5, color=(0, 255, 0), thickness=2)
        img.draw_circle(x3, y3, 5, color=(0, 255, 0), thickness=2)
        img.draw_circle(x4, y4, 5, color=(0, 255, 0), thickness=2)

        # 在图像上显示矩形的四个顶点坐标
        img.draw_string(x1, y1, "({}, {})".format(x1, y1), color=(255, 255, 255), scale=2)
        img.draw_string(x2, y2, "({}, {})".format(x2, y2), color=(255, 255, 255), scale=2)
        img.draw_string(x3, y3, "({}, {})".format(x3, y3), color=(255, 255, 255), scale=2)
        img.draw_string(x4, y4, "({}, {})".format(x4, y4), color=(255, 255, 255), scale=2)

        print("Rectangle points: ({}, {}), ({}, {}), ({}, {}), ({}, {})".format(x1, y1, x2, y2, x3, y3, x4, y4))
        print("Rotation angle:", rotation)

    # 在LCD显示屏上显示图像
    lcd.display(img)