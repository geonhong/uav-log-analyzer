import tkinter as tk
from tkinter import filedialog
from pymavlink import mavutil
import matplotlib.pyplot as plt
from fpdf import FPDF
from tqdm import tqdm
from geopy.distance import geodesic
from datetime import datetime

# 파일 선택 대화상자 열기
def select_file():
    root = tk.Tk()
    root.withdraw()  # Tkinter 창 숨기기
    file_path = filedialog.askopenfilename(
        title="Select log file",
        filetypes=[("BIN files", "*.bin"), ("LOG files", "*.log"), ("All files", "*.*")]
    )
    return file_path

# PDF 보고서 생성 함수
def create_pdf_report(date, uav_name, duration, max_speed, max_altitude, total_distance, max_distance, altitude_graph, roll_pitch_graph, speed_graph, rcou_graph_1_4, rcou_graph_5_8):
    pdf = FPDF()
    pdf.add_page()

    # 보고서 제목
    pdf.set_font("Arial", "B", 16)
    pdf.cell(200, 10, txt="Flight Test Report", ln=True, align='C')
    
    pdf.ln(10)

    # 비행 시험 날짜와 기체 종류 추가
    pdf.set_font("Arial", size=12)
    pdf.cell(200, 10, txt=f"Flight Date: {date}", ln=True, align='L')
    pdf.cell(200, 10, txt=f"UAV Name: {uav_name}", ln=True, align='L')

    pdf.ln(10)

    # 표 헤더 설정
    pdf.set_font("Arial", "B", 12)
    pdf.cell(70, 10, "Item", 1, 0, 'C')
    pdf.cell(100, 10, "Value", 1, 1, 'C')

    # 표 내용 작성
    pdf.set_font("Arial", size=12)
    pdf.cell(70, 10, "Flight Duration (s)", 1)
    pdf.cell(100, 10, f"{duration:.2f}", 1, 1)
    
    pdf.cell(70, 10, "Max Speed (m/s)", 1)
    pdf.cell(100, 10, f"{max_speed:.2f}", 1, 1)

    pdf.cell(70, 10, "Max Altitude (m)", 1)
    pdf.cell(100, 10, f"{max_altitude:.2f}", 1, 1)

    pdf.cell(70, 10, "Total Distance (m)", 1)
    pdf.cell(100, 10, f"{total_distance:.2f}", 1, 1)

    pdf.cell(70, 10, "Max Distance from Home (m)", 1)
    pdf.cell(100, 10, f"{max_distance:.2f}", 1, 1)

    # 줄바꿈
    pdf.ln(10)

    # 그래프 이미지 삽입
    pdf.set_font("Arial", "B", 12)
    pdf.cell(200, 10, txt="Altitude over Time", ln=True)
    pdf.image(altitude_graph, x=10, y=None, w=180)

    pdf.add_page()
    pdf.cell(200, 10, txt="Roll and Pitch over Time (AHR2)", ln=True)
    pdf.image(roll_pitch_graph, x=10, y=None, w=180)

    pdf.ln(10)
    pdf.cell(200, 10, txt="Speed over Time", ln=True)
    pdf.image(speed_graph, x=10, y=None, w=180)

    pdf.add_page()
    pdf.cell(200, 10, txt="Servo 1-4 over Time", ln=True)
    pdf.image(rcou_graph_1_4, x=10, y=None, w=180)

    pdf.ln(10)
    pdf.cell(200, 10, txt="Servo 5-8 over Time", ln=True)
    pdf.image(rcou_graph_5_8, x=10, y=None, w=180)

    # 파일명 설정
    filename = f"(NFTR-0{date[2:4]}-{date[5:7]}{date[8:10]}) {uav_name}.pdf"

    # PDF 저장
    pdf.output(filename)
    print(f"Report saved as '{filename}'")

# 그래프 생성 함수
def create_graph(x_data, y_data_list, labels, title, y_label, file_name, y_range=None):
    plt.figure(figsize=(10, 6))
    for y_data, label in zip(y_data_list, labels):
        plt.plot(x_data, y_data, label=label)
    plt.xlabel('Time (s)')
    plt.ylabel(y_label)
    plt.title(title)
    if y_range:
        plt.ylim(y_range)
    plt.legend()
    plt.grid(True)
    plt.savefig(file_name)

if __name__ == '__main__':
    # 사용자 입력 받기
    flight_date = input("Enter flight date (YYYY-MM-DD): ")
    uav_name = input("Enter UAV Name: ")

    # 파일 선택
    log_file_path = select_file()
    if not log_file_path:
        print("No file selected. Exiting...")
        exit()

    # 로그 파일을 연다
    mavlog = mavutil.mavlink_connection(log_file_path)

    # 전체 메시지 수를 미리 계산하기 위해 로그 파일의 메시지 수를 카운트
    total_msgs = 0
    first_timestamp = None
    while True:
        msg = mavlog.recv_match(blocking=False)
        if not msg:
            break
        # 첫 번째 메시지의 타임스탬프를 기록
        if first_timestamp is None:
            first_timestamp = msg._timestamp
        total_msgs += 1

    # 로그 파일 다시 열기 (메시지 수 계산 후)
    mavlog = mavutil.mavlink_connection(log_file_path)

    # 초기화 변수
    max_speed = 0
    max_altitude = 0
    start_time = first_timestamp  # start_time을 첫 번째 타임스탬프로 설정
    end_time = None

    plt_time_gps = []
    plt_time_ahr2 = []
    plt_ahr_alt = []
    plt_roll = []
    plt_pitch = []
    plt_speed = []

    plt_time_rcou = []
    plt_rcou_c1 = []
    plt_rcou_c2 = []
    plt_rcou_c3 = []
    plt_rcou_c4 = []
    plt_rcou_c5 = []
    plt_rcou_c6 = []
    plt_rcou_c7 = []
    plt_rcou_c8 = []

    gps_coords = []
    total_distance = 0
    max_distance_from_home = 0
    home_position = None

    # 로그 데이터 읽기
    with tqdm(total=total_msgs, desc="Loading log file", unit="msg") as pbar:
        while True:
            msg = mavlog.recv_match(blocking=False)
            
            if not msg:
                break

            # 비행 데이터가 담긴 메시지 유형을 확인
            if msg.get_type() == 'GPS':
                gps_speed = msg.Spd
                max_speed = max(max_speed, gps_speed)
                plt_speed.append(gps_speed)
                plt_time_gps.append(msg._timestamp - start_time)  # GPS 시간 0부터 시작
                
                # GPS 좌표 저장 및 거리 계산
                current_coord = (msg.Lat, msg.Lng)
                gps_coords.append(current_coord)

                # 홈 위치 설정 (처음 받는 GPS 메시지의 위치를 홈 위치로 설정)
                if home_position is None:
                    home_position = current_coord

                # 현재 위치와 이전 위치 사이의 거리 계산
                if len(gps_coords) > 1:
                    previous_coord = gps_coords[-2]
                    distance = geodesic(previous_coord, current_coord).meters
                    total_distance += distance

                # 현재 위치와 홈 위치 사이의 거리 계산
                distance_from_home = geodesic(home_position, current_coord).meters
                max_distance_from_home = max(max_distance_from_home, distance_from_home)
                
            # AHRS2 메시지에서 고도, 롤, 피치 데이터 확인
            if msg.get_type() == 'AHR2':
                ahrs_altitude = msg.Alt
                max_altitude = max(max_altitude, ahrs_altitude)
                
                # Create a plot for altitude
                plt_time_ahr2.append(msg._timestamp - start_time)  # AHRS2 시간 0부터 시작
                plt_ahr_alt.append(ahrs_altitude)

                # Roll and Pitch data
                plt_roll.append(msg.Roll)
                plt_pitch.append(msg.Pitch)

            # RCOU 메시지에서 C1-C8 데이터 확인
            if msg.get_type() == 'RCOU':
                plt_time_rcou.append(msg._timestamp - start_time)
                plt_rcou_c1.append(msg.C1)
                plt_rcou_c2.append(msg.C2)
                plt_rcou_c3.append(msg.C3)
                plt_rcou_c4.append(msg.C4)
                plt_rcou_c5.append(msg.C5)
                plt_rcou_c6.append(msg.C6)
                plt_rcou_c7.append(msg.C7)
                plt_rcou_c8.append(msg.C8)

            # 마지막 메시지 시간으로 비행 종료 시간 설정
            end_time = msg._timestamp
            
            # Progress bar update
            pbar.update(1)

    # 비행 시간 계산
    flight_duration = end_time - start_time

    # 결과 출력
    print(f"비행 시간: {flight_duration:.2f} 초")
    print(f"최고 속도: {max_speed:.2f} m/s")
    print(f"최고 고도: {max_altitude:.2f} m")
    print(f"총 비행 거리: {total_distance:.2f} m")
    print(f"홈으로부터 최대 비행 거리: {max_distance_from_home:.2f} m")

    # 고도 그래프 생성 및 저장 (AHR2 데이터를 사용)
    altitude_graph_path = 'altitude_plot.png'
    create_graph(plt_time_ahr2, [plt_ahr_alt], ['Altitude'], 'Altitude over Time from AHR2 Messages', 'Altitude (m)', altitude_graph_path)

    # 롤 및 피치 그래프 생성 및 저장 (AHR2 데이터를 사용)
    roll_pitch_graph_path = 'roll_pitch_plot.png'
    create_graph(plt_time_ahr2, [plt_roll, plt_pitch], ['Roll', 'Pitch'], 'Roll and Pitch over Time from AHR2 Messages', 'Angle (degrees)', roll_pitch_graph_path)

    # 스피드 그래프 생성 및 저장 (GPS 데이터를 사용)
    speed_graph_path = 'speed_plot.png'
    create_graph(plt_time_gps, [plt_speed], ['Speed'], 'Speed over Time from GPS Messages', 'Speed (m/s)', speed_graph_path)

    # RCOU C1-C4 그래프 생성 및 저장
    rcou_graph_1_4_path = 'rcou_c1_c4_plot.png'
    create_graph(plt_time_rcou, [plt_rcou_c1, plt_rcou_c2, plt_rcou_c3, plt_rcou_c4], ['C1', 'C2', 'C3', 'C4'], 'Servo 1-4 over Time', 'PWM Output', rcou_graph_1_4_path, y_range=[1000, 2000])

    # RCOU C5-C8 그래프 생성 및 저장
    rcou_graph_5_8_path = 'rcou_c5_c8_plot.png'
    create_graph(plt_time_rcou, [plt_rcou_c5, plt_rcou_c6, plt_rcou_c7, plt_rcou_c8], ['C5', 'C6', 'C7', 'C8'], 'Servo 5-8 over Time', 'PWM Output', rcou_graph_5_8_path, y_range=[1000, 2000])

    # PDF 보고서 생성
    create_pdf_report(flight_date, uav_name, flight_duration, max_speed, max_altitude, total_distance, max_distance_from_home, altitude_graph_path, roll_pitch_graph_path, speed_graph_path, rcou_graph_1_4_path, rcou_graph_5_8_path)