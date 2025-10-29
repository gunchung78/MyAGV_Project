import time
from pynput.keyboard import Controller as KeyboardController, Key
from pynput.mouse import Controller as MouseController, Button

# ====================================================================
# 설정 변수
# ====================================================================
# 반복 주기 (초 단위)
INTERVAL = 0.5 
# 매크로 시작 후 첫 번째 동작까지의 지연 시간 (매크로를 활성화할 창으로 이동할 시간을 벌기 위함)
START_DELAY = 3 
# 매크로를 중지할 키 (ESC 키)
STOP_KEY = Key.esc 

# 컨트롤러 초기화
keyboard = KeyboardController()
mouse = MouseController()

print(f"매크로가 {START_DELAY}초 후에 시작됩니다.")
print(f"매크로를 멈추려면 {STOP_KEY.name.upper()} 키를 누르세요.")

time.sleep(START_DELAY)

# ====================================================================
# 반복 실행 루프
# ====================================================================
try:
    for i in range(0, 997, 1):
        # 1. 키보드 '1' 누르기
        keyboard.press('2')
        keyboard.release('2')
        print(f"[{time.strftime('%H:%M:%S')}] 1번 키 입력")
        
        # 0.5초 대기
        time.sleep(INTERVAL) 
        
        # 2. 마우스 왼쪽 버튼 클릭 (현재 마우스 커서 위치)
        mouse.click(Button.left)
        print(f"[{time.strftime('%H:%M:%S')}] 마우스 클릭")
        
        # 0.5초 대기
        time.sleep(INTERVAL) 

        # 🚨 비상 정지: ESC 키가 눌렸는지 확인 (매우 중요)
        # pynput Listener를 사용하지 않고 간단히 구현하기 위해 여기에 직접 중단 로직을 포함하지는 않습니다.
        # 실행 중인 터미널 창을 닫거나, Ctrl+C를 눌러 중지하세요. 
        # (혹은 아래 '고급 중지 로직' 참고)

except KeyboardInterrupt:
    print("\n매크로가 Ctrl+C에 의해 강제 종료되었습니다.")
except Exception as e:
    print(f"\n오류 발생: {e}")
finally:
    print("매크로 실행이 종료되었습니다.")