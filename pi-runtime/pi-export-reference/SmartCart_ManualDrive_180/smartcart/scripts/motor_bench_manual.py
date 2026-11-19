import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# 왼쪽 모터
L_REN = 23
L_LEN = 24
L_RPWM = 12
L_LPWM = 13

# 오른쪽 모터
R_REN = 16
R_LEN = 26
R_RPWM = 18
R_LPWM = 19

# GPIO 초기 설정
pins = [23, 24, 12, 13, 16, 26, 18, 19]

for pin in pins:
    GPIO.setup(pin, GPIO.OUT, initial=GPIO.LOW)

# PWM 설정
pwm = {}

for pin in [12, 13, 18, 19]:
    pwm[pin] = GPIO.PWM(pin, 1000)
    pwm[pin].start(0)

# 목표 PWM
SPEED = 17

# 정지
def stop():
    for p in pwm.values():
        p.ChangeDutyCycle(0)

# 천천히 가속
def move(direction):
    stop()

    if direction == "forward":
        motors = [L_RPWM, R_RPWM]
    else:
        motors = [L_LPWM, R_LPWM]

    for speed in range(5, SPEED + 1):
        for pin in motors:
            pwm[pin].ChangeDutyCycle(speed)
        time.sleep(0.1)

try:
    # 드라이버 활성화
    for pin in [L_REN, L_LEN, R_REN, R_LEN]:
        GPIO.output(pin, GPIO.HIGH)

    print("전진")
    move("forward")
    time.sleep(3)

    print("정지")
    stop()
    time.sleep(2)

    print("후진")
    move("backward")
    time.sleep(3)

    print("정지")
    stop()

except KeyboardInterrupt:
    print("긴급 정지")

finally:
    stop()

    for pin in [L_REN, L_LEN, R_REN, R_LEN]:
        GPIO.output(pin, GPIO.LOW)

    for p in pwm.values():
        p.stop()

    GPIO.cleanup()
