import sensor
import time
import math
from machine import UART

# UART(1) を使用。ボーレート 115200
uart = UART(1, 115200, timeout=10)

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.VGA)

# --- カメラの明るさ・色設定 ---
target_exposure = 40000
sensor.set_auto_exposure(False, exposure_us=target_exposure)
sensor.set_auto_gain(False, gain_db=14)
sensor.set_auto_whitebal(False, rgb_gain_db=(60, 58, 61))

# --- 設定エリア ---
# 閾値（必ず閾値エディタで自分の環境に合わせて再取得してください）
yellow_threshold = (86, 56, -3, 54, 62, 3)
blue_threshold = (35, 15, -20, 0, -18, -4)

center_x = 290
center_y = 250

# 角度計算用の関数（コードをスッキリさせるため）
def get_angle(blob):
    dx = blob.cx() - center_x
    dy = blob.cy() - center_y
    angle_rad = math.atan2(dy, dx)
    angle_deg = math.degrees(angle_rad)

    # 下を正面(0度)にする計算
    robot_angle = 90 - angle_deg

    # 0-360度に正規化
    if robot_angle < 0: robot_angle += 360
    if robot_angle >= 360: robot_angle -= 360
    return int(robot_angle)

# -----------------

sensor.skip_frames(time=2000)
clock = time.clock()

while True:
    clock.tick()
    img = sensor.snapshot()

    # 1. 黄色ゴールの検知
    yellow_blobs = img.find_blobs([yellow_threshold], pixels_threshold=100, area_threshold=100, merge=True)
    if yellow_blobs:
        y_target = max(yellow_blobs, key=lambda b: b.pixels())
        y_angle = get_angle(y_target)
        y_flag = 1

        # 描画（黄色）
        img.draw_rectangle(y_target.rect(), color=(255, 255, 0)) # 黄色の枠
        img.draw_line(center_x, center_y, y_target.cx(), y_target.cy(), color=(255, 255, 0))
        img.draw_string(y_target.x(), y_target.y()-20, f"Y:{y_angle}", color=(255, 255, 0), scale=2)
        print(f"Yellow: {y_angle} deg")

    # 2. 青色ゴールの検知
    blue_blobs = img.find_blobs([blue_threshold], pixels_threshold=100, area_threshold=100, merge=True)
    if blue_blobs:
        b_target = max(blue_blobs, key=lambda b: b.pixels())
        b_flag = 1
        b_angle = get_angle(b_target)

        # 描画（青色）
        img.draw_rectangle(b_target.rect(), color=(0, 0, 255)) # 青の枠
        img.draw_line(center_x, center_y, b_target.cx(), b_target.cy(), color=(0, 0, 255))
        img.draw_string(b_target.x(), b_target.y()-20, f"B:{b_angle}", color=(0, 0, 255), scale=2)
        print(f"Blue: {b_angle} deg")

    # 中心点の描画
    img.draw_cross(center_x, center_y, color=(255, 255, 255))

    # --- マイコンからのリクエスト待機 ---
    if uart.any():
        request = uart.read(1)
        if request and request[0] == 253:
            # 送信パケット作成
            out_data = bytearray([
                253,
                y_flag,
                y_angle & 0x7F,
                (y_angle >> 7) & 0x7F,
                b_flag,
                b_angle & 0x7F,
                (b_angle >> 7) & 0x7F
            ])
            uart.write(out_data)
            print("Sent Data to MCU!") # リクエストが来たときだけ表示
        else:
                print("Unknown Request: %s" % request)

