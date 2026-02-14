import time
from adafruit_servokit import ServoKit

# 16채널 PCA9685 초기화
kit = ServoKit(channels=16)

# 서보 채널 할당 정의 (사용자 배선에 맞게 채널 번호를 수정하세요)
# 예: FL(Front Left), FR(Front Right), BL(Back Left), BR(Back Right)
servos = {
    "FL_Coxa": 0, "FL_Femur": 1, "FL_Tibia": 2,
    "FR_Coxa": 4, "FR_Femur": 5, "FR_Tibia": 6,
    "BL_Coxa": 8, "BL_Femur": 9, "BL_Tibia": 10,
    "BR_Coxa": 12, "BR_Femur": 13, "BR_Tibia": 14
}

def set_neutral_position():
    print("모든 서보를 중립 위치(1500ms / 90도)로 이동합니다...")
    
    for name, channel in servos.items():
        # 일반적으로 ServoKit의 angle 90도가 1500ms에 해당합니다.
        # 사용 중인 서보의 사양에 따라 90도 전후를 미세 조정해야 할 수 있습니다.
        kit.servo[channel].angle = 90
        print(f"[{name}] 채널 {channel}번 정렬 완료")

if __name__ == "__main__":
    try:
        set_neutral_position()
        print("\n정렬이 완료되었습니다. 이제 전원을 끄지 말고 조립을 진행하세요!")
        
        # 전원이 유지되어야 서보가 위치를 고정하므로 무한 루프로 대기합니다.
        while True:
            time.sleep(1)
            
    except KeyboardInterrupt:
        print("\n프로그램을 종료합니다.")