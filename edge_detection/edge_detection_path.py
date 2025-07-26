import sensor, image, time, lcd, math, gc, heapq

# 初始化摄像头
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time=2000)
lcd.rotation(0)
clock = time.clock()

# 系统参数
GRID_SIZE = 8  # 网格大小（降低分辨率）
THRESHOLD = 30  # 污染区二值化阈值
RED_COLOR = (255, 0, 0)
BLUE_COLOR = (0, 0, 255)
GREEN_COLOR = (0, 255, 0)

# 定义入口和出口坐标（根据实际位置调整）
ENTRANCE = (20, 20)
EXIT = (300, 220)

# 路径规划数据结构
class Node:
    __slots__ = ('x', 'y', 'g', 'h', 'f', 'parent')
    def __init__(self, x, y):
        self.x, self.y = x, y
        self.g = self.h = self.f = 0
        self.parent = None

    def __lt__(self, other):
        return self.f < other.f

# A*路径规划算法
def astar(grid, start, end):
    grid_w, grid_h = len(grid[0]), len(grid)
    open_set = []
    closed_set = set()

    start_node = Node(start[0], start[1])
    end_node = Node(end[0], end[1])
    heapq.heappush(open_set, start_node)

    while open_set:
        current = heapq.heappop(open_set)
        closed_set.add((current.x, current.y))

        # 找到路径
        if (current.x, current.y) == (end_node.x, end_node.y):
            path = []
            while current:
                path.append((current.x, current.y))
                current = current.parent
            return path[::-1]

        # 检查8个邻居方向
        for dx, dy in [(0,1),(1,0),(0,-1),(-1,0),(1,1),(1,-1),(-1,1),(-1,-1)]:
            nx, ny = current.x + dx, current.y + dy

            # 边界检查
            if not (0 <= nx < grid_w and 0 <= ny < grid_h):
                continue

            # 障碍物检查
            if grid[ny][nx] == 1:
                continue

            # 跳过已处理的节点
            if (nx, ny) in closed_set:
                continue

            # 计算移动成本（对角线成本更高）
            move_cost = 14 if dx and dy else 10

            new_node = Node(nx, ny)
            new_node.g = current.g + move_cost
            new_node.h = 10 * (abs(nx - end_node.x) + abs(ny - end_node.y))  # 曼哈顿距离
            new_node.f = new_node.g + new_node.h
            new_node.parent = current

            # 添加到开放列表
            heapq.heappush(open_set, new_node)

    return []  # 无路径

# 主循环
path = []
last_grid = None

while True:
    clock.tick()
    gc.collect()  # 垃圾回收防止内存溢出

    img = sensor.snapshot()
    gray = img.copy().to_grayscale()

    # 创建二值污染区地图
    grid_width = gray.width() // GRID_SIZE
    grid_height = gray.height() // GRID_SIZE
    grid_map = [[0]*grid_width for _ in range(grid_height)]

    # 生成网格地图（1=障碍物）
    for gy in range(grid_height):
        for gx in range(grid_width):
            sx, sy = gx * GRID_SIZE, gy * GRID_SIZE

            # 检查网格内是否有污染像素
            has_pollution = False
            for y in range(sy, sy + GRID_SIZE):
                for x in range(sx, sx + GRID_SIZE):
                    # 确保坐标在图像范围内
                    if x < gray.width() and y < gray.height():
                        if gray.get_pixel(x, y) < THRESHOLD:
                            has_pollution = True
                            break
                if has_pollution:
                    break

            if has_pollution:
                grid_map[gy][gx] = 1
                # 在原图标记污染区（红色）
                img.draw_rectangle(sx, sy, GRID_SIZE, GRID_SIZE, RED_COLOR, fill=True)

    # 转换入口/出口到网格坐标
    grid_start = (ENTRANCE[0]//GRID_SIZE, ENTRANCE[1]//GRID_SIZE)
    grid_end = (EXIT[0]//GRID_SIZE, EXIT[1]//GRID_SIZE)

    # 当污染区变化时重新规划路径
    if grid_map != last_grid:
        last_grid = [row[:] for row in grid_map]  # 深拷贝
        path = astar(grid_map, grid_start, grid_end)

    # 绘制路径（蓝色）
    if path:
        total_length = 0
        prev_pixel = None

        for i, (gx, gy) in enumerate(path):
            # 转换网格坐标回像素坐标（网格中心）
            px = gx * GRID_SIZE + GRID_SIZE//2
            py = gy * GRID_SIZE + GRID_SIZE//2

            # 绘制路径点
            img.draw_circle(px, py, 3, BLUE_COLOR, fill=True)

            # 连接路径点
            if prev_pixel:
                img.draw_line(prev_pixel[0], prev_pixel[1], px, py, BLUE_COLOR, 2)
                # 计算线段长度
                dx = (px - prev_pixel[0]) * 0.1  # 假设1像素=0.1cm
                dy = (py - prev_pixel[1]) * 0.1
                total_length += math.sqrt(dx*dx + dy*dy)

            prev_pixel = (px, py)

        # 显示路径长度（使用传统字符串格式化）
        text = "Path: %.1f cm" % total_length
        # 添加背景矩形提高文字可读性
        img.draw_rectangle(10, 10, len(text)*8, 16, (255, 255, 255), fill=True)
        img.draw_string(10, 10, text, color=BLUE_COLOR, scale=1)

    # 标记入口（绿色）和出口（绿色）
    img.draw_circle(ENTRANCE[0], ENTRANCE[1], 5, GREEN_COLOR, fill=True)
    img.draw_circle(EXIT[0], EXIT[1], 5, GREEN_COLOR, fill=True)

    lcd.display(img)
    print("FPS: %.1f, Mem: %.1fKB" % (clock.fps(), gc.mem_free()/1024))
