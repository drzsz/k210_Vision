'''
import sensor, image, time, lcd

sensor.reset()
sensor.set_pixformat(sensor.RGB565)  # 设置为RGB彩色模式
sensor.set_framesize(sensor.QVGA)     # 分辨率设为QVGA (320x240)
sensor.skip_frames(time=2000)
clock = time.clock()

while True:
    clock.tick()
    original_img = sensor.snapshot().copy()  # 捕获并复制原始彩色图像

    # 创建用于边缘检测的灰度图副本
    grayscale_img = original_img.to_grayscale()

    # 在灰度图上执行边缘检测
    grayscale_img.find_edges(image.EDGE_SIMPLE, threshold=(100, 255))

    # 遍历所有像素点，在原图上用红线标记检测到的边缘
    for x in range(grayscale_img.width()):
        for y in range(grayscale_img.height()):
            # 当灰度图的像素值>127时视为边缘（检测结果为白色边缘）
            if grayscale_img.get_pixel(x, y) > 127:
                original_img.set_pixel(x, y, (255, 0, 0))  # 设置红色像素

    lcd.display(original_img)  # 显示带边缘标记的彩色图
    print(clock.fps())
'''
import sensor, image, time, lcd

sensor.reset()
sensor.set_pixformat(sensor.RGB565)  # 使用彩色模式
sensor.set_framesize(sensor.QVGA)     # 分辨率设为QVGA (320x240)
sensor.skip_frames(time=2000)
lcd.rotation(0)                       # 确保显示方向正确
clock = time.clock()

while True:
    clock.tick()
    # 获取彩色图像
    img = sensor.snapshot()

    # 创建灰度副本进行边缘检测
    gray = img.copy().to_grayscale()
    gray.find_edges(image.EDGE_SIMPLE, threshold=(100, 255))

    # 在所有检测到的边缘上绘制红线
    for x in range(img.width()):
        for y in range(img.height()):
            if gray.get_pixel(x, y) > 127:  # 如果检测到边缘
                # 在原始彩色图像上绘制红线 (RGB565格式)
                img.set_pixel(x, y, (255, 0, 0))

    lcd.display(img)  # 显示彩色图像
    print(clock.fps())  # 打印帧率

