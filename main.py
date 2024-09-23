import os
from uav_log_analyzer import select_file
from APLogAnalyzer import APLogAnalyzer

file_path = select_file()
log = APLogAnalyzer(file_path)

case_name_w_ext = os.path.basename(file_path)
case_name, _ = os.path.splitext(case_name_w_ext)

log.extractData(
    message_type='AHR2',
    fields = ['TimeUS', 'Roll', 'Pitch', 'Yaw', 'Alt'],
    output_file= case_name + '_AHR2_data.csv',
    plot=False
)

log.extractData(
    message_type='GPS',
    fields = ['TimeUS', 'Spd'],
    output_file= case_name + '_GPS_data.csv',
    plot=False
)

log.extractData(
    message_type='RCOU',
    fields = ['TimeUS', 'C5', 'C6', 'C7', 'C8'],
    output_file= case_name + '_RCOU_data.csv',
    plot=False
)

log.extractData(
    message_type='BAT',
    fields = ['TimeUS', 'Volt'],
    output_file= case_name + '_BAT_data.csv',
    plot=False
)
# log.save_as_ascii(case_name + '_ascii.dat')