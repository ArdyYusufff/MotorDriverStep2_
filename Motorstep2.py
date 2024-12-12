import RPi.GPIO as GPIO
import time

# Konfigurasi pin GPIO untuk masing-masing motor
# Motor 1
RPWM1 = 2  # Right PWM
LPWM1 = 3  # Left PWM
L_EN1 = 14 # Left Enable
R_EN1 = 15 # Right Enable

# Motor 2
RPWM2 = 4
LPWM2 = 17
L_EN2 = 18
R_EN2 = 23

# Motor 3
RPWM3 = 27
LPWM3 = 22
L_EN3 = 24
R_EN3 = 25

# Setup GPIO mode
GPIO.setmode(GPIO.BCM)

# Fungsi untuk inisialisasi motor
def init_motor():
    motors = [RPWM1, LPWM1, L_EN1, R_EN1, RPWM2, LPWM2, L_EN2, R_EN2, RPWM3, LPWM3, L_EN3, R_EN3]
    for pin in motors:
        GPIO.setup(pin, GPIO.OUT)
        GPIO.output(pin, GPIO.LOW)

# Fungsi untuk mengontrol motor dengan RPWM dan LPWM
def motor_control_pwm(r_pwm, l_pwm, speed_r, speed_l):
    pwm_r = GPIO.PWM(r_pwm, 100)  # 100Hz
    pwm_l = GPIO.PWM(l_pwm, 100)  # 100Hz

    pwm_r.start(speed_r)
    pwm_l.start(speed_l)

    return pwm_r, pwm_l

# Fungsi untuk mengontrol motor dengan L_EN dan R_EN
def motor_control_enable(l_en, r_en, left_on, right_on):
    GPIO.output(l_en, GPIO.HIGH if left_on else GPIO.LOW)
    GPIO.output(r_en, GPIO.HIGH if right_on else GPIO.LOW)

# Fungsi untuk menghentikan motor
def stop_motor(pwms=[]):
    for pwm in pwms:
        pwm.stop()

    all_enable_pins = [L_EN1, R_EN1, L_EN2, R_EN2, L_EN3, R_EN3]
    for pin in all_enable_pins:
        GPIO.output(pin, GPIO.LOW)

# Fungsi untuk aksi robot
def move_forward():
    print("Moving forward")
    pwm1 = motor_control_pwm(RPWM1, LPWM1, 80, 0)
    pwm2 = motor_control_pwm(RPWM2, LPWM2, 80, 0)
    pwm3 = motor_control_pwm(RPWM3, LPWM3, 80, 0)
    motor_control_enable(L_EN1, R_EN1, True, True)
    motor_control_enable(L_EN2, R_EN2, True, True)
    motor_control_enable(L_EN3, R_EN3, True, True)
    time.sleep(2)
    stop_motor(pwm1 + pwm2 + pwm3)

def move_backward():
    print("Moving backward")
    pwm1 = motor_control_pwm(RPWM1, LPWM1, 0, 80)
    pwm2 = motor_control_pwm(RPWM2, LPWM2, 0, 80)
    pwm3 = motor_control_pwm(RPWM3, LPWM3, 0, 80)
    motor_control_enable(L_EN1, R_EN1, True, True)
    motor_control_enable(L_EN2, R_EN2, True, True)
    motor_control_enable(L_EN3, R_EN3, True, True)
    time.sleep(2)
    stop_motor(pwm1 + pwm2 + pwm3)

def turn_left():
    print("Turning left")
    pwm1 = motor_control_pwm(RPWM1, LPWM1, 0, 80)
    pwm2 = motor_control_pwm(RPWM2, LPWM2, 80, 0)
    pwm3 = motor_control_pwm(RPWM3, LPWM3, 80, 0)
    motor_control_enable(L_EN1, R_EN1, True, True)
    motor_control_enable(L_EN2, R_EN2, True, True)
    motor_control_enable(L_EN3, R_EN3, True, True)
    time.sleep(2)
    stop_motor(pwm1 + pwm2 + pwm3)

def turn_right():
    print("Turning right")
    pwm1 = motor_control_pwm(RPWM1, LPWM1, 80, 0)
    pwm2 = motor_control_pwm(RPWM2, LPWM2, 0, 80)
    pwm3 = motor_control_pwm(RPWM3, LPWM3, 0, 80)
    motor_control_enable(L_EN1, R_EN1, True, True)
    motor_control_enable(L_EN2, R_EN2, True, True)
    motor_control_enable(L_EN3, R_EN3, True, True)
    time.sleep(2)
    stop_motor(pwm1 + pwm2 + pwm3)

# Main Program
if __name__ == "__main__":
    try:
        init_motor()

        while True:
            command = input("Enter command (w: forward, s: backward, a: left, d: right, q: quit): ").lower()
            if command == 'w':
                move_forward()
            elif command == 's':
                move_backward()
            elif command == 'a':
                turn_left()
            elif command == 'd':
                turn_right()
            elif command == 'q':
                print("Exiting program")
                break
            else:
                print("Invalid command")

    except KeyboardInterrupt:
        print("Program interrupted")

    finally:
        GPIO.cleanup()
