import sensor
import image
import lcd

# 初始化LCD显示屏
lcd.init()

# 初始化摄像头
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.set_vflip(False)
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
    rects = img_scaled.find_rects(threshold=10000)

    # 将矩形坐标放大到原始图像尺寸
    for r in rects:
        r_scaled = [r.x() * 2, r.y() * 2, r.w() * 2, r.h() * 2]

        # 绘制矩形
        img.draw_rectangle(r_scaled, color=(255, 0, 0), thickness=2)

        # 输出矩形的四个点的坐标
        x1, y1, x2, y2 = r_scaled[0], r_scaled[1], r_scaled[0] + r_scaled[2], r_scaled[1] + r_scaled[3]
        x3, y3, x4, y4 = x2, y1, x1, y2
        print("Rectangle points: ({}, {}), ({}, {}), ({}, {}), ({}, {})".format(x1, y1, x2, y2, x3, y3, x4, y4))

    # 在LCD显示屏上显示图像
    lcd.display(img)
