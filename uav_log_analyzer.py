import tkinter as tk
from tkinter import filedialog
from pymavlink import mavutil
import matplotlib.pyplot as plt


# 파일 선택 대화상자 열기
def select_file():
    root = tk.Tk()
    root.withdraw()  # Tkinter 창 숨기기
    file_path = filedialog.askopenfilename(
        title="Select log file",
        filetypes=[("BIN files", "*.bin"), ("LOG files", "*.log"), ("All files", "*.*")]
    )
    return file_path

# 파일 선택
log_file_path = select_file()
if not log_file_path:
    print("No file selected. Exiting...")
    exit()

# 로그 파일을 연다
mavlog = mavutil.mavlink_connection(log_file_path)

# 초기화 변수
max_speed = 0
max_altitude = 0
start_time = None
end_time = None

plt_time = []
plt_ahr_alt = []

# 로그 데이터 읽기
while True:
    msg = mavlog.recv_match(blocking=False)
    
    if not msg:
        break

    # 시간 측정 (로그 시작 시간)
    if start_time is None:
        start_time = msg._timestamp

    # 비행 데이터가 담긴 메시지 유형을 확인
    if msg.get_type() == 'GPS':
        gps_speed = msg.Spd
        max_speed = max(max_speed, gps_speed)
        
    # AHRS2 메시지에서 고도 확인
    if msg.get_type() == 'AHR2':
        # print(msg)
        ahrs_altitude = msg.Alt
        max_altitude = max(max_altitude, ahrs_altitude)
        
        # Create a plot
        plt_time.append(msg._timestamp)
        plt_ahr_alt.append(msg.Alt)

    # 마지막 메시지 시간으로 비행 종료 시간 설정
    end_time = msg._timestamp

# 비행 시간 계산
flight_duration = end_time - start_time

# 결과 출력
print(f"비행 시간: {flight_duration:.2f} 초")
print(f"최고 속도: {max_speed:.2f} m/s")
print(f"최고 고도: {max_altitude:.2f} m")

# Make a plot
plt.figure(figsize=(10, 6))
plt.plot(plt_time, plt_ahr_alt, label='Altitude (AHR2)')
plt.xlabel('Time (s)')
plt.ylabel('Altitude (m)')
plt.title('Altitude over Time from AHR2 Messages')
plt.legend()
plt.grid(True)
plt.show()