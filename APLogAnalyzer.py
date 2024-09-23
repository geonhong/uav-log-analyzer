import os
import csv
import matplotlib.pyplot as plt
from pymavlink import mavutil
from enum import Enum

class ModeEnum(Enum):
    AUTO = 10
    FBWB = 17
    QHOVER = 18
    QLOITER = 19

class APLogAnalyzer:
    def __init__(self, log_file):
        """
        Initializes the ArduPilotLog class and loads the log file.
        :param log_file: Path to the ArduPilot log file (.bin or .log)
        """
        self.log_file = log_file
        self.connection = mavutil.mavlink_connection(log_file)
        self.messages = []
        self._get_start_time()
        self._get_mode_change_time()

    def extractData(self, message_type, fields=None, output_file=None, plot=False, rewind=True):
        """
        Extracts specific message data from the log file.
        :param message_type: The MAVLink message type to extract (e.g., 'ATT', 'GPS', etc.)
        :param fields: List of specific fields to extract from the message (e.g., ['TimeUS', 'Lat', 'Lng'])
        :param output_file: Path to save the extracted data as a CSV file (optional)
        :param plot: If True, plots the extracted data
        :return: Extracted data as a list of dictionaries
        """
        extracted_data = []

        if rewind:
            # Rewind the file pointer or reset connection
            self.connection = mavutil.mavlink_connection(self.log_file)

        # Read the log file and extract messages
        while True:
            msg = self.connection.recv_match(type=message_type, blocking=False)
            if msg is None:
                break

            # Extract desired fields if specified, otherwise get all fields
            if fields:
                data = {field: getattr(msg, field) for field in fields if hasattr(msg, field)}

                # Convert TimeUS to seconds based on the first log time
                if 'TimeUS' in data:
                    # Subtract the initial TimeUS and convert to seconds
                    data['Timestamp(s)'] = (data['TimeUS'] - self.start_time) / 1_000_000.0

            else:
                data = msg.to_dict()

            extracted_data.append(data)
            
        # Save the data to a CSV file if output_file is specified
        if output_file:
            self._save_to_csv(extracted_data, output_file)

        # Plot the data if plot is True
        if plot and fields:
            self._plot_data(extracted_data, fields)

        return extracted_data

    def _save_to_csv(self, data, output_file):
        """
        Saves extracted data to a CSV file.
        :param data: List of dictionaries containing extracted data
        :param output_file: Path to save the CSV file
        """
        if not data:
            print("No data to save.")
            return

        with open(output_file, mode='w', newline='') as file:
            writer = csv.DictWriter(file, fieldnames=data[0].keys())
            writer.writeheader()
            for row in data:
                writer.writerow(row)

        print(f"Data saved to {output_file}")

    def _plot_data(self, data, fields):
        """
        Plots the extracted data.
        :param data: List of dictionaries containing extracted data
        :param fields: List of fields to plot (up to two fields)
        """
        if len(fields) < 2:
            print("At least two fields are required for plotting.")
            return

        x_values = [entry[fields[0]] for entry in data]
        y_values = [entry[fields[1]] for entry in data]

        plt.figure()
        plt.plot(x_values, y_values)
        plt.xlabel(fields[0])
        plt.ylabel(fields[1])
        plt.title(f"{fields[0]} vs {fields[1]}")
        plt.show()
    
    def _get_start_time(self):
        start_time = None
        # Read the log file and extract messages
        while True:
            msg = self.connection.recv_match(type='FMTU', blocking=False)
            if msg is None:
                break

            if start_time is None:
                start_time = getattr(msg, 'TimeUS')
            else:
                break
        
        print('Log start time: ', start_time)

        self.start_time = start_time
    
    def _get_mode_change_time(self):
        while True:
            msg = self.connection.recv_match(type='MODE', blocking=False)
            if msg is None:
                break

            timeStamp = (msg.TimeUS - self.start_time) / 1_000_000.0
            mode_name = self._get_mode_name(msg.Mode)
            print('Mode has been changed at ', timeStamp, ' to ', mode_name)
    
    def _get_mode_name(self, mode_value):
        try:
            return ModeEnum(mode_value).name
        except ValueError:
            return f"Unknown Mode: {mode_value}"

    def save_as_ascii(self, output_file):
        """
        Reads the entire log file using the mavlink connection and saves each MAVLink message as an ASCII string.
        :param output_file: The path to save the ASCII-encoded log file.
        """
        try:
            with open(output_file, 'w') as out_f:
                while True:
                    # Read the next MAVLink message from the log file
                    msg = self.connection.recv_match(blocking=False)
                    if msg is None:
                        break  # End of file
                    
                    # Convert message to its ASCII representation
                    ascii_representation = str(msg)  # The message object itself is converted to a string
                    
                    # Write the ASCII message to the output file
                    out_f.write(ascii_representation + '\n')

            print(f"Log file saved as ASCII in {output_file}")

        except FileNotFoundError:
            print(f"Log file '{self.log_file}' not found.")
        except Exception as e:
            print(f"An error occurred: {e}")
