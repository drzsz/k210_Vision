import sensor, image, time, lcd, machine
from machine import UART
from fpioa_manager import fm

# 初始化LCD显示
lcd.init()
lcd.clear(lcd.GREEN)
print("LCD initialized")

# 初始化摄像头
sensor.reset()
sensor.set_pixformat(sensor.GRAYSCALE)  # 使用灰度图像简化处理
sensor.set_framesize(sensor.QVGA)       # 320x240
sensor.skip_frames(time=500)
sensor.set_vflip(0)
sensor.set_auto_gain(False)
sensor.set_auto_whitebal(False)
sensor.set_contrast(3)                 # 增加对比度
sensor.set_brightness(-3)              # 降低亮度
print("Camera initialized")

# 配置串口通信
fm.register(6, fm.fpioa.UART2_RX)
fm.register(8, fm.fpioa.UART2_TX)
uart = UART(UART.UART2, 115200, 8, 0, 0, timeout=1000, read_buf_len=4096)

# 激光检测参数
MIN_BRIGHTNESS = 200       # 降低阈值以便检测
MAX_BLOB_SIZE = 30         # 最大斑点尺寸

# 位置跟踪
last_x, last_y = 160, 120

def find_laser_point(img):
    """寻找最亮的点"""
    # 1. 查找整个图像中最亮的像素
    max_brightness = 0
    max_x, max_y = 0, 0

    for y in range(0, img.height(), 2):  # 每隔2行扫描
        for x in range(0, img.width(), 2):  # 每隔2列扫描
            pix = img.get_pixel(x, y)
            if pix > max_brightness:
                max_brightness = pix
                max_x, max_y = x, y

    # 2. 检查是否达到最小亮度
    if max_brightness < MIN_BRIGHTNESS:
        return None, None, max_brightness

    # 3. 检查周围区域确认是激光点
    count = 0
    for dy in range(-1, 2):
        for dx in range(-1, 2):
            nx, ny = max_x + dx, max_y + dy
            if 0 <= nx < img.width() and 0 <= ny < img.height():
                if img.get_pixel(nx, ny) > MIN_BRIGHTNESS - 20:
                    count += 1

    # 4. 如果是孤立的亮点，则认为是激光点
    if count < 3:  # 周围亮点太少，可能是噪声
        return None, None, max_brightness

    return max_x, max_y, max_brightness

def sending_data(x, y):
    """发送坐标到串口"""
    global last_x, last_y
    data_str = "X{:03d}Y{:03d}\n".format(x, y)
    uart.write(data_str.encode())
    print("Sent:", data_str.strip())
    last_x, last_y = x, y

# ===== 主循环 =====
print("Starting laser tracking...")
clock = time.clock()

while True:
    img = sensor.snapshot()

    # 查找激光点
    laser_x, laser_y, brightness = find_laser_point(img)
    found = laser_x is not None

    # 如果没有找到激光点，使用上一次的位置
    if not found:
        laser_x, laser_y = last_x, last_y

    # 发送坐标到串口
    sending_data(laser_x, laser_y)

    # 在顶部绘制状态栏
    img.draw_rectangle(0, 0, 320, 25, color=(255, 255, 255), fill=True)

    # 显示坐标值
    coord_text = "X:%d Y:%d" % (laser_x, laser_y)
    img.draw_string(10, 5, coord_text, color=(0, 0, 0), scale=2)

    # 显示激光点状态
    if found:
        status_text = "Laser Detected!"
        status_color = (0, 255, 0)

        # 绘制标记
        img.draw_cross(laser_x, laser_y, color=0, size=10, thickness=2)
        img.draw_circle(laser_x, laser_y, 5, color=0, thickness=2)
    else:
        status_text = "No Laser (Bright:%d)" % brightness
        status_color = (255, 0, 0)

    img.draw_string(150, 5, status_text, color=status_color, scale=1.5)

    # 显示帧率
    fps_text = "FPS:%.1f" % (clock.fps())
    img.draw_string(250, 5, fps_text, color=(0, 0, 0), scale=1.5)

    # 更新LCD显示
    lcd.display(img)

    # 控制循环速度
    clock.tick()
    time.sleep_ms(10)  # 约100FPS
