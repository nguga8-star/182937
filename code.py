# ==============================================================================
# HỆ THỐNG XE TỰ HÀNH & ĐIỀU KHIỂN QUA WEB (ESP32 - MICROPYTHON) - BẢN FIX LỖI
# Kích thước xe: 25 x 15 cm
# Đã khắc phục:
#   1. Chống mất Wi-Fi khi đề-pa/đảo chiều motor + Fail-safe dừng xe khi rớt mạng.
#   2. Phanh tức thì khi gặp vật cản (không chạy rướn gây tông vật cản).
#   3. Quét so sánh 2 bên Trái/Phải công bằng, lùi xe rồi đánh lái về hướng thoáng nhất.
# ==============================================================================

from machine import Pin, PWM, SoftI2C, time_pulse_us
import network
import socket
import time
import gc

# ==================== 1. KHAI BÁO CHÂN KẾT NỐI ====================

LED_ONBOARD_PIN = 2

# L298N
ENA_PIN = 25
IN1_PIN = 26
IN2_PIN = 27
ENB_PIN = 33
IN3_PIN = 14
IN4_PIN = 13

# Siêu âm HC-SR04 / SRF05
TRIG_PIN = 18
ECHO_PIN = 19

# Servo MG90S / SG90
SERVO_PIN = 5

# MPU6050
I2C_SDA_PIN = 21
I2C_SCL_PIN = 22
MPU6050_ADDR = 0x68

# ==================== 2. CẤU HÌNH & BIẾN TOÀN CỤC ====================

# Khoảng cách an toàn cho xe 25x15cm
STOP_DIST_CM = 35.0          # Tăng lên 35cm để trừ hao quán tính thân xe 25cm
EMERGENCY_STOP_CM = 18.0     # Dưới 18cm là phanh gấp ngay lập tức, không cần đọc lại
MIN_VALID_DIST_CM = 2.0
CLEAR_THRESHOLD_CM = 40.0    # Ngưỡng coi là hướng thông thoáng

# PWM điều khiển (Giảm xung đề-pa đột ngột để tránh sụt áp gây mất Wi-Fi)
BASE_PWM_8BIT = 95
KICKSTART_PWM_8BIT = 135     # Hạ từ 165 -> 135 để chống sụt nguồn ESP32
REVERSE_PWM_8BIT = 110
TURN_PWM_8BIT = 140          # Đủ lực cho cả 2 bánh quay tại chỗ trên mặt sàn
MIN_PWM_8BIT = 80
KP_STRAIGHT = 2.5

SPEED_AT_PWM100_CMS = 60.0
DIST_SLIP_FACTOR = 1.0
REVERSE_DIST_CM = 18.0       # Lùi ~18cm (vừa đẹp cho thân xe 25cm lấy đà đánh lái)

# Đảo chiều quay Trái/Phải (Do xe bạn đang bị ngược chiều quay thực tế)
INVERT_TURN_DIRECTION = True

# Cấu hình Servo
SERVO_CENTER_DEG = 90
SERVO_LEFT_IS_NEGATIVE_ANGLE = True
SERVO_STEP_DEG = 4
SERVO_STEP_DELAY_MS = 12
SERVO_SETTLE_MARGIN_SEC = 0.15

# Các cặp góc quét đối xứng: (+ là Trái, - là Phải)
SCAN_ANGLES_LEFT = [35, 65, 90]
SCAN_ANGLES_RIGHT = [-35, -65, -90]

# Trạng thái hoạt động
current_mode = "manual"
manual_cmd = "S"
last_manual_cmd_ms = 0
MANUAL_TIMEOUT_MS = 1500     # Tự động dừng nếu mất mạng quá 1.5 giây khi đang ấn nút

server_socket = None
ap_if = None

gyro_z_offset = 0.0
current_yaw = 0.0
target_yaw = 0.0
last_gyro_us = time.ticks_us()
current_servo_phys_angle = SERVO_CENTER_DEG
motors_are_moving_forward = False
mpu_available = False

# ==================== 3. KHỞI TẠO PHẦN CỨNG ====================

in1 = Pin(IN1_PIN, Pin.OUT, value=0)
in2 = Pin(IN2_PIN, Pin.OUT, value=0)
in3 = Pin(IN3_PIN, Pin.OUT, value=0)
in4 = Pin(IN4_PIN, Pin.OUT, value=0)

ena_pwm = PWM(Pin(ENA_PIN), freq=1000, duty=0)
enb_pwm = PWM(Pin(ENB_PIN), freq=1000, duty=0)

try:
    led = Pin(LED_ONBOARD_PIN, Pin.OUT, value=0)
except Exception:
    led = None

trig = Pin(TRIG_PIN, Pin.OUT, value=0)
echo = Pin(ECHO_PIN, Pin.IN)

servo_pwm = PWM(Pin(SERVO_PIN), freq=50, duty=0)
i2c = SoftI2C(scl=Pin(I2C_SCL_PIN), sda=Pin(I2C_SDA_PIN), freq=100000)


def led_set(state):
    if led is not None:
        led.value(1 if state else 0)


def pwm_8bit_to_duty10(val_8bit):
    val_8bit = max(0, min(255, int(val_8bit)))
    return int((val_8bit * 1023) // 255)


def check_abort_auto():
    """Lắng nghe Web liên tục và kiểm tra xem người dùng có ngắt chế độ Auto không."""
    if server_socket is not None:
        handle_web_requests(server_socket)
    return current_mode != "auto"


# ==================== 4. HÀM SERVO & MPU6050 ====================

def _apply_servo_pulse(phys_angle):
    phys_angle = max(0, min(180, int(phys_angle)))
    pulse_us = 500 + int((phys_angle * 1900) / 180)
    try:
        servo_pwm.duty_ns(pulse_us * 1000)
    except AttributeError:
        duty_10bit = int((pulse_us * 1023) // 20000)
        servo_pwm.duty(duty_10bit)


def servo_relax():
    try:
        servo_pwm.duty_ns(0)
    except AttributeError:
        servo_pwm.duty(0)


def servo_write_physical_angle(target_phys_angle, relax_after=False):
    global current_servo_phys_angle
    target_phys_angle = max(10, min(170, int(target_phys_angle))) # Giới hạn chống kẹt cơ khí gây sụt áp
    step = SERVO_STEP_DEG if target_phys_angle >= current_servo_phys_angle else -SERVO_STEP_DEG
    angle = current_servo_phys_angle

    if angle == target_phys_angle:
        _apply_servo_pulse(target_phys_angle)
        time.sleep(SERVO_SETTLE_MARGIN_SEC)
    else:
        while (step > 0 and angle < target_phys_angle) or (step < 0 and angle > target_phys_angle):
            angle += step
            if (step > 0 and angle > target_phys_angle) or (step < 0 and angle < target_phys_angle):
                angle = target_phys_angle
            _apply_servo_pulse(angle)
            time.sleep_ms(SERVO_STEP_DELAY_MS)
        current_servo_phys_angle = target_phys_angle
        time.sleep(SERVO_SETTLE_MARGIN_SEC)

    if relax_after:
        servo_relax()


def servo_set_offset(offset_deg, relax_after=False):
    if SERVO_LEFT_IS_NEGATIVE_ANGLE:
        phys_angle = SERVO_CENTER_DEG - offset_deg
    else:
        phys_angle = SERVO_CENTER_DEG + offset_deg
    servo_write_physical_angle(phys_angle, relax_after=relax_after)


def mpu_write_reg(reg, data):
    i2c.writeto_mem(MPU6050_ADDR, reg, bytes([data]))


def mpu_read_gyro_z_raw():
    if not mpu_available:
        return 0
    try:
        data = i2c.readfrom_mem(MPU6050_ADDR, 0x47, 2)
        val = (data[0] << 8) | data[1]
        if val & 0x8000:
            val -= 65536
        return val
    except Exception:
        return 0


def mpu_init():
    global gyro_z_offset, last_gyro_us, mpu_available
    try:
        mpu_write_reg(0x6B, 0x00)
        time.sleep_ms(50)
        mpu_write_reg(0x1B, 0x00)
        mpu_write_reg(0x1A, 0x03)
        mpu_available = True
    except Exception:
        print("MPU6050 khong kha dung. Chay che do thuong.")
        mpu_available = False
        last_gyro_us = time.ticks_us()
        return

    total = 0
    samples = 120
    for _ in range(samples):
        total += mpu_read_gyro_z_raw()
        time.sleep_ms(2)
    gyro_z_offset = float(total) / float(samples)
    last_gyro_us = time.ticks_us()


def mpu_update_yaw():
    global current_yaw, last_gyro_us
    now = time.ticks_us()
    dt = time.ticks_diff(now, last_gyro_us) / 1000000.0
    last_gyro_us = now

    if not mpu_available or dt > 0.5:
        return

    raw_gz = mpu_read_gyro_z_raw()
    gz_deg_per_sec = (float(raw_gz) - gyro_z_offset) / 131.0
    if abs(gz_deg_per_sec) > 0.45:
        current_yaw += gz_deg_per_sec * dt


# ==================== 5. ĐIỀU KHIỂN ĐỘNG CƠ ====================

def motors_stop(brake=False):
    global motors_are_moving_forward
    if brake and motors_are_moving_forward:
        # Phanh điện tử ngắn để triệt tiêu quán tính, chống trôi xe tông vào vật cản
        in1.value(0); in2.value(1)
        in3.value(0); in4.value(1)
        brake_duty = pwm_8bit_to_duty10(110)
        ena_pwm.duty(brake_duty)
        enb_pwm.duty(brake_duty)
        time.sleep_ms(35)

    motors_are_moving_forward = False
    ena_pwm.duty(0)
    enb_pwm.duty(0)
    in1.value(0); in2.value(0)
    in3.value(0); in4.value(0)


def _set_turn_pins(turn_left):
    """Đặt chân IN1..IN4 để xoay tại chỗ, có hỗ trợ đảo chiều nếu đấu dây ngược."""
    actual_left = (not turn_left) if INVERT_TURN_DIRECTION else turn_left
    if actual_left:
        in1.value(0); in2.value(1)
        in3.value(1); in4.value(0)
    else:
        in1.value(1); in2.value(0)
        in3.value(0); in4.value(1)


def motors_drive_straight(base_pwm_8bit=BASE_PWM_8BIT):
    global motors_are_moving_forward, last_gyro_us
    mpu_update_yaw()

    in1.value(1); in2.value(0)
    in3.value(1); in4.value(0)

    if not motors_are_moving_forward:
        # Khởi động mềm 2 bậc để tránh sụt áp nguồn gây mất Wi-Fi
        ena_pwm.duty(pwm_8bit_to_duty10(90))
        enb_pwm.duty(pwm_8bit_to_duty10(90))
        time.sleep_ms(20)
        kick_duty = pwm_8bit_to_duty10(KICKSTART_PWM_8BIT)
        ena_pwm.duty(kick_duty)
        enb_pwm.duty(kick_duty)
        time.sleep_ms(30)
        motors_are_moving_forward = True
        last_gyro_us = time.ticks_us()

    error = target_yaw - current_yaw
    correction = int(error * KP_STRAIGHT) if mpu_available else 0
    if INVERT_TURN_DIRECTION:
        correction = -correction

    left_pwm = max(MIN_PWM_8BIT, min(255, base_pwm_8bit - correction))
    right_pwm = max(MIN_PWM_8BIT, min(255, base_pwm_8bit + correction))

    ena_pwm.duty(pwm_8bit_to_duty10(left_pwm))
    enb_pwm.duty(pwm_8bit_to_duty10(right_pwm))


def motors_manual_control(cmd):
    """Điều khiển thủ công 4 hướng qua giao diện Web."""
    duty = pwm_8bit_to_duty10(TURN_PWM_8BIT)

    if cmd == 'F':  # Tiến
        in1.value(1); in2.value(0)
        in3.value(1); in4.value(0)
        ena_pwm.duty(duty); enb_pwm.duty(duty)
    elif cmd == 'B':  # Lùi
        in1.value(0); in2.value(1)
        in3.value(0); in4.value(1)
        ena_pwm.duty(duty); enb_pwm.duty(duty)
    elif cmd == 'L':  # Quay Trái tại chỗ
        _set_turn_pins(True)
        ena_pwm.duty(duty); enb_pwm.duty(duty)
    elif cmd == 'R':  # Quay Phải tại chỗ
        _set_turn_pins(False)
        ena_pwm.duty(duty); enb_pwm.duty(duty)
    else:  # 'S' hoặc khác -> Dừng
        motors_stop(brake=False)


def motors_move_distance_cm(distance_cm, forward=True, check_obstacle=False):
    global last_gyro_us, motors_are_moving_forward
    if check_abort_auto():
        return

    motors_stop(brake=False)
    time.sleep_ms(80) # Nghỉ ngắn trước khi đảo chiều để bảo vệ nguồn Wi-Fi

    duration_ms = int((abs(distance_cm) / SPEED_AT_PWM100_CMS) * 1000.0 * DIST_SLIP_FACTOR)

    if forward:
        in1.value(1); in2.value(0)
        in3.value(1); in4.value(0)
    else:
        in1.value(0); in2.value(1)
        in3.value(0); in4.value(1)

    # Đề-pa êm
    kick_duty = pwm_8bit_to_duty10(KICKSTART_PWM_8BIT)
    ena_pwm.duty(kick_duty); enb_pwm.duty(kick_duty)
    time.sleep_ms(35)

    last_gyro_us = time.ticks_us()
    start_ms = time.ticks_ms()
    last_sonar_check_ms = start_ms

    while time.ticks_diff(time.ticks_ms(), start_ms) < duration_ms:
        if check_abort_auto():
            motors_stop()
            return

        now_ms = time.ticks_ms()
        if forward and check_obstacle and time.ticks_diff(now_ms, last_sonar_check_ms) >= 40:
            last_sonar_check_ms = now_ms
            if sonar_read_once() <= STOP_DIST_CM:
                motors_stop(brake=True)
                return

        mpu_update_yaw()
        error = target_yaw - current_yaw
        correction = int(error * KP_STRAIGHT) if mpu_available else 0
        if INVERT_TURN_DIRECTION:
            correction = -correction

        if forward:
            left_pwm = max(MIN_PWM_8BIT, min(255, BASE_PWM_8BIT - correction))
            right_pwm = max(MIN_PWM_8BIT, min(255, BASE_PWM_8BIT + correction))
        else:
            left_pwm = max(MIN_PWM_8BIT, min(255, REVERSE_PWM_8BIT + correction))
            right_pwm = max(MIN_PWM_8BIT, min(255, REVERSE_PWM_8BIT - correction))

        ena_pwm.duty(pwm_8bit_to_duty10(left_pwm))
        enb_pwm.duty(pwm_8bit_to_duty10(right_pwm))
        time.sleep_ms(5)

    motors_stop(brake=False)
    time.sleep_ms(120)
    mpu_update_yaw()


def motors_turn_by_offset(offset_deg):
    """
    offset_deg > 0: Đánh lái sang Trái
    offset_deg < 0: Đánh lái sang Phải
    """
    global target_yaw, last_gyro_us, motors_are_moving_forward
    if check_abort_auto():
        return

    motors_stop(brake=False)
    time.sleep_ms(60)

    angle_deg = abs(float(offset_deg))
    if angle_deg < 2.0:
        return

    turn_left = (offset_deg > 0)
    _set_turn_pins(turn_left)

    start_yaw = current_yaw
    last_gyro_us = time.ticks_us()

    turn_duty = pwm_8bit_to_duty10(TURN_PWM_8BIT)
    ena_pwm.duty(turn_duty)
    enb_pwm.duty(turn_duty)

    start_ms = time.ticks_ms()
    # Thời gian dự phòng theo góc (nếu MPU6050 bị treo hoặc xoay không đủ tốc độ)
    expected_ms = int(angle_deg * 6.8) + 120

    if mpu_available:
        target_turn = max(2.0, angle_deg - 3.0)
        while abs(current_yaw - start_yaw) < target_turn:
            if check_abort_auto():
                motors_stop()
                return
            mpu_update_yaw()
            # Nếu MPU không thay đổi góc (do lỗi đọc) hoặc quá thời gian dự kiến -> ngắt an toàn
            if time.ticks_diff(time.ticks_ms(), start_ms) > max(expected_ms + 400, 1800):
                break
            time.sleep_ms(5)
    else:
        while time.ticks_diff(time.ticks_ms(), start_ms) < expected_ms:
            if check_abort_auto():
                motors_stop()
                return
            time.sleep_ms(10)

    motors_stop(brake=False)
    time.sleep_ms(150)
    mpu_update_yaw()
    target_yaw = current_yaw


# ==================== 6. SIÊU ÂM & QUÉT NÉ VẬT CẢN ====================

def sonar_read_once():
    trig.value(0)
    time.sleep_us(2)
    trig.value(1)
    time.sleep_us(10)
    trig.value(0)

    duration = time_pulse_us(echo, 1, 20000) # Timeout 20ms (~3.4m) giúp vòng lặp phản xạ nhanh hơn
    if duration <= 0:
        return 999.0

    dist = (float(duration) * 0.0343) / 2.0
    if dist < MIN_VALID_DIST_CM:
        return 999.0
    return dist


def confirm_obstacle_when_stopped(first_reading):
    """
    Khi phát hiện vật cản:
    - Nếu <= EMERGENCY_STOP_CM (18cm): Phanh gấp và xác nhận luôn!
    - Nếu từ 18..35cm: Dừng xe ngay lập tức (KHÔNG chạy tiếp như code cũ) rồi đo kiểm tra 2 lần.
    """
    motors_stop(brake=True)
    if first_reading <= EMERGENCY_STOP_CM:
        return True

    valid_hits = 1
    for _ in range(2):
        if check_abort_auto():
            return False
        time.sleep_ms(15)
        d = sonar_read_once()
        if MIN_VALID_DIST_CM <= d <= STOP_DIST_CM + 3.0:
            valid_hits += 1
    return valid_hits >= 2


def sonar_read_scan_median():
    v = []
    for _ in range(3):
        v.append(sonar_read_once())
        time.sleep_ms(15)
    v.sort()
    return v[1]


def scan_best_direction():
    """
    Quét đối xứng cả bên Trái và bên Phải để tìm hướng có không gian trống lớn nhất.
    Trả về góc cần đánh lái (dương = Trái, âm = Phải) hoặc None nếu bị kẹt cả 2 bên.
    """
    best_offset = None
    best_dist = -1.0

    # Đan xen cặp góc Trái - Phải từ nhỏ đến lớn: 35, -35, 65, -65, 90, -90
    paired_offsets = [35, -35, 65, -65, 90, -90]
    results = {}

    for offset in paired_offsets:
        if check_abort_auto():
            break
        servo_set_offset(offset, relax_after=False)
        dist = sonar_read_scan_median()
        results[offset] = dist
        if dist > best_dist:
            best_dist = dist
            best_offset = offset

    servo_set_offset(0, relax_after=True)

    # Tính trung bình độ thoáng bên Trái và bên Phải để quyết định bên ưu tiên
    left_avg = sum(results.get(a, 0) for a in SCAN_ANGLES_LEFT) / 3.0
    right_avg = sum(results.get(a, 0) for a in SCAN_ANGLES_RIGHT) / 3.0

    if best_dist <= STOP_DIST_CM:
        return None

    # Chọn bên có tổng khoảng trống rộng hơn, lấy góc vừa đủ thoát (55 độ) để xe 25x15cm rẽ mượt
    if left_avg >= right_avg + 5.0:
        # Ưu tiên bên Trái (+): chọn góc đầu tiên bên trái vượt ngưỡng thoáng
        for a in SCAN_ANGLES_LEFT:
            if results.get(a, 0) > CLEAR_THRESHOLD_CM:
                return a
        return 60
    elif right_avg > left_avg + 5.0:
        # Ưu tiên bên Phải (-): chọn góc đầu tiên bên phải vượt ngưỡng thoáng
        for a in SCAN_ANGLES_RIGHT:
            if results.get(a, 0) > CLEAR_THRESHOLD_CM:
                return a
        return -60
    else:
        # Hai bên tương đương nhau -> chọn góc có khoảng cách xa nhất
        return best_offset


def avoid_obstacle():
    """
    Quy trình chuẩn cho xe 25x15cm:
    1. Xe đã dừng ngay khi thấy vật cản.
    2. Lùi xuống 18cm trước để đầu xe thoát khỏi góc hẹp.
    3. Quét Servo tìm bên không có vật cản (Trái hoặc Phải).
    4. Đánh lái trực tiếp về phía trống đó rồi để vòng lặp chính chạy thẳng tiếp.
    """
    global target_yaw
    if check_abort_auto():
        return

    # Bước 1: Lùi xe xuống ngay khi thấy vật cản
    motors_move_distance_cm(REVERSE_DIST_CM, forward=False)
    if check_abort_auto():
        return

    # Bước 2: Quét tìm hướng không có vật cản
    clear_offset = scan_best_direction()
    if check_abort_auto():
        return

    # Bước 3: Nếu cả 2 bên đều tắc -> Lùi thêm chút nữa rồi quay đầu 180 độ
    if clear_offset is None:
        motors_move_distance_cm(12.0, forward=False)
        motors_turn_by_offset(180)
        return

    # Bước 4: Đánh hướng xe về phía không thấy vật cản và cập nhật hướng đi mới
    motors_turn_by_offset(clear_offset)
    target_yaw = current_yaw


# ==================== 7. KHỞI TẠO MẠNG WI-FI & WEB SERVER ====================

HTML_PAGE = """<!DOCTYPE html><html><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1,user-scalable=no"><title>Robot Control</title><style>body{font-family:Arial,sans-serif;text-align:center;background:#1e1e2f;color:#fff;margin:0;padding:15px;touch-action:manipulation}h2{color:#4db8ff;margin-bottom:5px}.status{font-size:14px;margin-bottom:15px;color:#aaa}.mode-box{display:flex;justify-content:center;gap:15px;margin-bottom:20px}.btn-m{padding:12px 20px;font-size:16px;border:none;border-radius:8px;font-weight:bold;cursor:pointer}.btn-auto{background:#28a745;color:#fff}.btn-man{background:#ffc107;color:#000}.active{outline:3px solid #fff;box-shadow:0 0 10px #fff}.d-pad{display:grid;grid-template-columns:80px 80px 80px;grid-template-rows:80px 80px 80px;gap:10px;justify-content:center}.btn-d{background:#2c2c44;border:2px solid #4db8ff;color:#4db8ff;font-size:26px;border-radius:12px;display:flex;align-items:center;justify-content:center;-webkit-user-select:none;user-select:none;touch-action:none}.btn-d:active{background:#4db8ff;color:#1e1e2f}.disabled{opacity:0.25;pointer-events:none}</style></head><body><h2>ROBOT ĐIỀU KHIỂN</h2><div class="status" id="st">Chế độ: THỦ CÔNG</div><div class="mode-box"><button class="btn-m btn-man active" id="bM" onclick="setM('manual')">Thủ Công</button><button class="btn-m btn-auto" id="bA" onclick="setM('auto')">Tự Động</button></div><div class="d-pad" id="pad"><div></div><button class="btn-d" onpointerdown="hold('F')" onpointerup="rel()" onpointerleave="rel()" onpointercancel="rel()">▲</button><div></div><button class="btn-d" onpointerdown="hold('L')" onpointerup="rel()" onpointerleave="rel()" onpointercancel="rel()">◀</button><button class="btn-d" onpointerdown="rel()" style="background:#dc3545;color:#fff;border-color:#dc3545">■</button><button class="btn-d" onpointerdown="hold('R')" onpointerup="rel()" onpointerleave="rel()" onpointercancel="rel()">▶</button><div></div><button class="btn-d" onpointerdown="hold('B')" onpointerup="rel()" onpointerleave="rel()" onpointercancel="rel()">▼</button><div></div></div><script>let tmr=null;function snd(c){fetch('/cmd?val='+c,{ signal: AbortSignal.timeout(800) }).catch(()=>{})}function hold(c){if(tmr)clearInterval(tmr);snd(c);tmr=setInterval(()=>snd(c),400)}function rel(){if(tmr){clearInterval(tmr);tmr=null}snd('S')}function setM(m){if(tmr){clearInterval(tmr);tmr=null}fetch('/mode?val='+m).catch(()=>{});if(m==='auto'){document.getElementById('bA').classList.add('active');document.getElementById('bM').classList.remove('active');document.getElementById('pad').classList.add('disabled');document.getElementById('st').innerText='Chế độ: TỰ ĐỘNG NÉ VẬT CẢN'}else{document.getElementById('bM').classList.add('active');document.getElementById('bA').classList.remove('active');document.getElementById('pad').classList.remove('disabled');document.getElementById('st').innerText='Chế độ: THỦ CÔNG'}}</script></body></html>"""


def setup_wifi_ap(ssid="Robot_ESP32", password=""):
    global ap_if
    # Tắt STA_IF để ESP32 không quét ngầm gây rớt mạng AP
    sta = network.WLAN(network.STA_IF)
    sta.active(False)

    ap_if = network.WLAN(network.AP_IF)
    ap_if.active(True)
    try:
        # Cố định kênh 1 giúp sóng ổn định hơn khi motor hoạt động
        if password:
            ap_if.config(essid=ssid, password=password, authmode=network.AUTH_WPA_WPA2_PSK, channel=1)
        else:
            ap_if.config(essid=ssid, authmode=network.AUTH_OPEN, channel=1)
    except Exception:
        if password:
            ap_if.config(essid=ssid, password=password, authmode=network.AUTH_WPA_WPA2_PSK)
        else:
            ap_if.config(essid=ssid, authmode=network.AUTH_OPEN)

    while not ap_if.active():
        time.sleep_ms(100)
    print("Wi-Fi AP san sang:", ap_if.ifconfig()[0])
    return ap_if


def check_wifi_health():
    """Đảm bảo AP luôn bật nếu chẳng may bị nhiễu."""
    global ap_if
    if ap_if is not None and not ap_if.active():
        try:
            ap_if.active(True)
        except Exception:
            pass


def handle_web_requests(s_sock):
    """Xử lý tối đa 3 request mỗi lượt để tránh dồn ứ hàng đợi socket làm đơ mạng."""
    global current_mode, manual_cmd, last_manual_cmd_ms, last_gyro_us, target_yaw
    for _ in range(3):
        conn = None
        try:
            conn, addr = s_sock.accept()
            conn.settimeout(0.08)
            req = conn.recv(512).decode('utf-8', 'ignore')

            if "GET /mode?val=" in req:
                if "val=auto" in req:
                    motors_stop(brake=False)
                    servo_set_offset(0, relax_after=True)
                    mpu_update_yaw()
                    last_gyro_us = time.ticks_us()
                    target_yaw = current_yaw
                    current_mode = "auto"
                elif "val=manual" in req:
                    current_mode = "manual"
                    manual_cmd = "S"
                    motors_stop(brake=False)
                    servo_relax()
                conn.sendall(b'HTTP/1.1 200 OK\r\nContent-Type: text/plain\r\nConnection: close\r\n\r\nOK')

            elif "GET /cmd?val=" in req:
                start_idx = req.find("val=") + 4
                cmd = req[start_idx:start_idx + 1]
                last_manual_cmd_ms = time.ticks_ms()
                if cmd == 'S':
                    manual_cmd = 'S'
                    motors_stop(brake=False)
                elif current_mode == "manual":
                    manual_cmd = cmd
                    motors_manual_control(cmd)
                conn.sendall(b'HTTP/1.1 200 OK\r\nContent-Type: text/plain\r\nConnection: close\r\n\r\nOK')

            else:
                conn.sendall(b'HTTP/1.1 200 OK\r\nContent-Type: text/html\r\nConnection: close\r\n\r\n' + HTML_PAGE.encode('utf-8'))

        except OSError:
            break
        finally:
            if conn is not None:
                try:
                    conn.close()
                except Exception:
                    pass


# ==================== 8. VÒNG LẶP CHÍNH (MAIN LOOP) ====================

def main():
    global server_socket, current_yaw, target_yaw, manual_cmd
    motors_stop(brake=False)
    servo_relax()
    led_set(False)

    # 1. Phát Wi-Fi Access Point
    setup_wifi_ap("Robot_ESP32")

    # 2. Khởi tạo Web Server (Non-blocking)
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_socket.bind(('0.0.0.0', 80))
    server_socket.listen(5)
    server_socket.setblocking(False)

    mpu_init()
    servo_set_offset(0, relax_after=True)
    current_yaw = 0.0
    target_yaw = 0.0

    led_set(True)
    print("HE THONG DA KHOI DONG! Ket noi Wi-Fi 'Robot_ESP32' va vao 192.168.4.1")

    loop_cnt = 0
    while True:
        handle_web_requests(server_socket)

        if current_mode == "auto":
            front_dist = sonar_read_once()
            if front_dist <= STOP_DIST_CM:
                if confirm_obstacle_when_stopped(front_dist):
                    avoid_obstacle()
            else:
                motors_drive_straight(BASE_PWM_8BIT)
        else:
            # Chế độ Manual: Nếu đang chạy mà mất mạng / không nhận được gói tin heartbeat quá 1.5s -> Tự dừng xe!
            if manual_cmd != "S" and time.ticks_diff(time.ticks_ms(), last_manual_cmd_ms) > MANUAL_TIMEOUT_MS:
                manual_cmd = "S"
                motors_stop(brake=False)

        loop_cnt += 1
        if loop_cnt >= 150:
            loop_cnt = 0
            check_wifi_health()
            gc.collect()

        time.sleep_ms(12)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        motors_stop(brake=False)
        servo_relax()
        led_set(False)
        print("Da tat chuong trinh.")
