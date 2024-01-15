import tkinter as tk
from tkinter import font
from PIL import Image, ImageTk
import serial
import pyautogui
import time

pyautogui.FAILSAFE = False

# Specify the serial port and baud rate
serial_port = 'COM4'  # Replace with your ESP32's serial port
baud_rate = 115200

# Open the serial port

ser = serial.Serial(serial_port, baud_rate, timeout=1)

if ser.isOpen():
    print(f"Serial port {serial_port} opened successfully.")

def calibration():
    try:
        ser.write('C'.encode('utf-8'))
        while True:
            data = ser.readline().decode('utf-8').strip()
            if data == 'Calibrated':
                print('IMU Calibrated')
                return
    except:
        print("Error during calibration")

def scale_adjust():
    try:
        ser.write('S'.encode('utf-8'))
        need_x_scaling = True
        need_y_scaling = True
        need_adjusting = True

        label.config(text="Move glove along x-axis of screen")
        root.update()
        while need_x_scaling == True:
            data = ser.readline().decode('utf-8').split()
            if ' '.join(data) == 'X axis set':
                print('x-axis set')
                need_x_scaling = False

        label.config(text="Move glove along y-axis of screen")
        root.update()
        while need_y_scaling == True:
            data = ser.readline().decode('utf-8').split()
            if ' '.join(data) == 'Y axis set':
                print('y-axis set')
                need_y_scaling = False

        label.config(text="Adjusting")
        root.update()
        while need_adjusting == True:
            data = ser.readline().decode('utf-8').split()
            if ' '.join(data) == 'Adjusted':
                print('Axis adjusted')
                need_adjusting = False
                return     
    except:
        print("Error during scale adjustment")

def run_mouse():
    try:
        ser.write('R'.encode('utf-8'))
        while True:
            data = ser.readline().decode('utf-8').split()
            print(data)
            click_state, long_state, scroll_state, move_state, x, y, a = tuple(map(float, data))
            x *= 1920*0.5
            y *= 1080*0.5

            #click and drag
            if click_state == 1 : 
                pyautogui.click()
            elif click_state == 2 :
                pyautogui.doubleClick()
            
            elif long_state == 1 :
                pyautogui.drag(x,-y,duration=0.1)
            
            #scroll            
            if scroll_state == 1:
                # break
                if 30 <= abs(a) <= 60 :
                    if a>0:pyautogui.scroll(200)
                    else: pyautogui.scroll(-200)
                elif abs(a) > 60 :
                    if a>0:pyautogui.scroll(500)
                    else: pyautogui.scroll(-500)

            #move cursor
            if move_state == 1 :
                curr_x, curr_y = pyautogui.position()
                pyautogui.moveTo(curr_x + x, curr_y - y)

    except KeyboardInterrupt:
        # Close the serial port when the script is interrupted
        ser.close()
        print("Serial port closed.")

# Main window
root = tk.Tk()
root.title("Motion Mouse")

# Custom font for labels
custom_font = font.Font(family="Arial", size=14)

# Function to handle the Start button click
def start_click():
    start_button.destroy()
    label.config(text="Calibrating. Keep IMU level.")
    root.update()
    calibration()
    scale_adjust()
    root.destroy()
    time.sleep(0.2)
    run_mouse()

# Add components to the main window
label = tk.Label(root, text="Motion Mouse", font=custom_font, padx=50)
label.pack(pady=10)

# Load, resize, and display the mouse image
try:
    original_img = Image.open("mouse_image.png")
    resized_img = original_img.resize((115, 100), Image.Resampling.LANCZOS)  # Change (100, 100) to your desired size
    mouse_img = ImageTk.PhotoImage(resized_img)
    img_label = tk.Label(root, image=mouse_img)
    img_label.pack(pady=10)
except Exception as e:
    print("Error loading image:", e)

start_button = tk.Button(root, text="Start", command=start_click)
start_button.pack(pady=10)

root.mainloop()