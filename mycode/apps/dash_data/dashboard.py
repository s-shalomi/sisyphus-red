import serial
import requests
import random
import json
import re
import time

# Set up the UART connection
ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
other_chars = re.compile(r'\x1B\[[0-?]*[ -/]*[@-~]')

url = "https://api.eu-w1.tago.io/data"
headers = {
    "Content-Type": "application/json",
    "Device-Token": "2d40a1f9-7dda-4e91-8240-c1bd40693a41",
}

while True:
    line = ser.readline().decode('utf-8').strip()
    line = other_chars.sub('', line)
    if "uart:~$" in line:
        line = line.split("uart:~$")[-1].strip()

    if line.startswith("Path: {"):
        data = line[7:-3].strip()
        print(f"Received path data: {data}")

        payload = [
            {
                "variable": "path",
                "value": data,
            }
        ]

        response = requests.post(url, json=payload, headers=headers)
        print(f"Sent data, Response: {response.status_code} Message: {response.text}")
        
    if line.startswith("Obstacles: {"):
        data = line[12:-1].strip()
        print(f"Received obstacles: {data}")

        payload = [
            {
                "variable": "obstacles",
                "value": data,
            }
        ]

        response = requests.post(url, json=payload, headers=headers)
        print(f"Sent data, Response: {response.status_code} Message: {response.text}")


# params = {
#     "variable": "obstacles",
    
# }

# response = requests.get(url, params=params, headers=headers)
# if response.status_code == 200:
#     data = response.json()
#     if data:
#         # path = data[0].get("value", "")
#         print(f"Received data: {data}")
#     else:
#         print("No data received.")
