import sensor, image, time, lcd, machine
from machine import UART
from fpioa_manager import fm
import math

# 初始化LCD显示
lcd.init()
lcd.clear(lcd.GREEN)
print("LCD initialized")

# 初始化摄像头
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)  # 320x240
sensor.skip_frames(time=500)
sensor.set_vflip(0)
sensor.set_auto_gain(False)
sensor.set_auto_whitebal(False)
print("Camera initialized")

# 配置串口通信
fm.register(6, fm.fpioa.UART2_RX)
fm.register(8, fm.fpioa.UART2_TX)
uart = UART(UART.UART2, 115200, 8, 0, 0, timeout=1000, read_buf_len=4096)

# 激光检测参数
RED_LASER_THRESHOLD = (80, 100,    # L亮度 (高亮度区域)
                       55, 100,    # a红色分量
                       -30, 30)    # b黄色分量 (接近中性)

MIN_PIXELS = 3        # 最小像素点数量
MAX_LASER_SIZE = 20   # 激光点最大尺寸
LASER_BRIGHTNESS = 80 # 最小亮度值

# 调试信息
last_x, last_y = 160, 120
detection_log = []

def find_laser_point(img):
    """寻找最亮的红色区域作为激光点"""
    # 1. 获取高亮度区域
    binary = img.binary([(LASER_BRIGHTNESS, 100)], invert=False)

    # 2. 查找所有亮点斑块
    blobs = binary.find_blobs(
        [RED_LASER_THRESHOLD],
        pixels_threshold=MIN_PIXELS,
        area_threshold=MIN_PIXELS,
        merge=False
    )

    if not blobs:
        return None, None

    # 3. 筛选和排序斑块
    valid_blobs = []
    for blob in blobs:
        # 尺寸过滤
        if blob.w() > MAX_LASER_SIZE or blob.h() > MAX_LASER_SIZE:
            continue

        # 形状过滤 (圆形度)
        circularity = 4 * math.pi * blob.area() / (blob.perimeter()**2)
        if circularity < 0.5:  # 排除非圆形斑点
            continue

        # 颜色过滤
        roi = (blob.x(), blob.y(), blob.w(), blob.h())
        stats = img.get_statistics(roi=roi)

        # 检查红色通道是否足够强
        if stats.l_mean() < LASER_BRIGHTNESS or stats.a_mean() < 60:
            continue

        valid_blobs.append({
            "blob": blob,
            "score": stats.l_mean() + stats.a_mean() * 2,  # 亮度+红色分量加权
            "x": blob.cx(),
            "y": blob.cy()
        })

    if not valid_blobs:
        return None, None

    # 4. 选择得分最高的斑点
    best_blob = max(valid_blobs, key=lambda b: b["score"])

    # 记录调试信息
    global detection_log
    detection_log.append({
        "x": best_blob["x"],
        "y": best_blob["y"],
        "l": stats.l_mean(),
        "a": stats.a_mean(),
        "size": best_blob["blob"].area()
    })
    if len(detection_log) > 20:
        detection_log.pop(0)

    return best_blob["x"], best_blob["blob"]

def sending_data(x, y):
    """发送坐标到串口"""
    global last_x, last_y
    last_x, last_y = x, y

    data_str = "X{:03d}Y{:03d}\n".format(x, y)
    uart.write(data_str.encode())
    print("Sent:", data_str.strip())

# ===== 主循环 =====
print("Starting laser tracking...")
frame_count = 0

while True:
    frame_count += 1
    img = sensor.snapshot()

    # 查找激光点
    laser_x, laser_blob = find_laser_point(img)
    found = laser_blob is not None

    # 如果没有找到激光点，使用上一次的位置
    if not found:
        laser_x, laser_y = last_x, last_y
    else:
        laser_y = laser_blob.cy()
        # 绘制标记
        img.draw_rectangle(laser_blob.rect(), color=(0, 255, 0))
        img.draw_cross(laser_x, laser_y, color=(255, 0, 0), size=5)

        # 添加尺寸指示器
        img.draw_string(laser_x+5, laser_y-15, "{laser_blob.w()}x{laser_blob.h()}",
                        color=(200, 200, 0), scale=1)

    # 发送坐标到串口
    sending_data(laser_x, laser_y)

    # 在顶部绘制状态栏
    img.draw_rectangle(0, 0, 320, 25, color=(0, 0, 0), fill=True)

    # 显示坐标值
    coord_text = "X:%d Y:%d" % (laser_x, laser_y)
    img.draw_string(10, 5, coord_text, color=(255, 255, 255), scale=2)

    # 显示激光点状态
    if found:
        status_text = "Laser Detected"
        status_color = (0, 255, 0)
    else:
        status_text = "No Laser"
        status_color = (255, 0, 0)

    img.draw_string(150, 5, status_text, color=status_color, scale=1.5)

    # 显示帧率和检测信息
    #fps_text = "FPS:%.1f" % (clock.fps())
   # img.draw_string(250, 5, fps_text, color=(255, 255, 255), scale=1.5)

    # 显示最近检测值
    if detection_log:
        last = detection_log[-1]
        info_text = "L:%d A:%d" % (last["l"], last["a"])
        img.draw_string(10, 220, info_text, color=(200, 200, 0), scale=1)
        size_text = "Size:%d" % last["size"]
        img.draw_string(120, 220, size_text, color=(200, 200, 0), scale=1)

    # 在图像上标记位置
    img.draw_circle(laser_x, laser_y, 3, color=(0, 0, 255), thickness=2)

    # 更新LCD显示
    lcd.display(img)

    # 控制循环速度
    time.sleep_ms(20)  # 约50FPS
    #clock.tick()
