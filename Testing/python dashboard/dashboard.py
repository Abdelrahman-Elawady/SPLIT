import tkinter as tk
import serial
import time

# === SETUP YOUR USB PORT HERE ===
# Change 'COM5' to whatever port your Transceiver ESP32 is using
# On Mac/Linux, it looks like '/dev/ttyUSB0'
SERIAL_PORT = 'COM7' 
BAUD_RATE = 115200

try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    time.sleep(2) # Wait for serial connection to stabilize
except Exception as e:
    print(f"Error opening serial port: {e}")
    exit()

def send_command(prefix, value):
    command = f"{prefix}:{value}\n"
    ser.write(command.encode('utf-8'))
    print(f"Sent: {command.strip()}")

# --- UI Setup ---
root = tk.Tk()
root.title("Robot Telemetry & Control Dashboard")
root.geometry("400x500")

# --- Sliders ---
def create_slider(label, prefix, min_val, max_val, resolution):
    frame = tk.Frame(root)
    frame.pack(pady=5, fill="x", padx=20)
    tk.Label(frame, text=label, width=15, anchor="w").pack(side="left")
    
    slider = tk.Scale(frame, from_=min_val, to=max_val, resolution=resolution, orient="horizontal", length=200)
    slider.pack(side="left")
    
    btn = tk.Button(frame, text="Send", command=lambda: send_command(prefix, slider.get()))
    btn.pack(side="right", padx=5)

tk.Label(root, text="PID Tuning", font=("Arial", 14, "bold")).pack(pady=10)
create_slider("Angle Kp (P)", "P", 0, 800, 1)
create_slider("Angle Kd (D)", "D", 0, 20, 0.1)
create_slider("Speed Kp (V)", "V", 0, 5, 0.05)
create_slider("Speed Ki (U)", "U", 0, 0.5, 0.01)
create_slider("Yaw Kp (Y)", "Y", 0, 5, 0.1)

# --- Navigation Controls (Hold-to-Drive) ---
tk.Label(root, text="Navigation", font=("Arial", 14, "bold")).pack(pady=10)
nav_frame = tk.Frame(root)
nav_frame.pack()

btn_fwd = tk.Button(nav_frame, text="Forward", width=10)
btn_fwd.grid(row=0, column=1, pady=2)
btn_fwd.bind('<ButtonPress-1>', lambda e: send_command("B", 6.0)) # Lean forward 3 degrees
btn_fwd.bind('<ButtonRelease-1>', lambda e: send_command("B", 0.0)) # Stand up straight

btn_left = tk.Button(nav_frame, text="Left Spin", width=10)
btn_left.grid(row=1, column=0, padx=2)
btn_left.bind('<ButtonPress-1>', lambda e: send_command("T", 60.0))
btn_left.bind('<ButtonRelease-1>', lambda e: send_command("T", 0.0))

btn_right = tk.Button(nav_frame, text="Right Spin", width=10)
btn_right.grid(row=1, column=2, padx=2)
btn_right.bind('<ButtonPress-1>', lambda e: send_command("T", -60.0))
btn_right.bind('<ButtonRelease-1>', lambda e: send_command("T", 0.0))

btn_rev = tk.Button(nav_frame, text="Reverse", width=10)
btn_rev.grid(row=2, column=1, pady=2)
btn_rev.bind('<ButtonPress-1>', lambda e: send_command("B", -9.0))
btn_rev.bind('<ButtonRelease-1>', lambda e: send_command("B", 0.0))


# --- Auto-Spin Logic ---
is_spinning = False

def toggle_spin():
    global is_spinning
    if is_spinning:
        send_command("T", 0.0) 
        btn_autospin.config(text="Auto Spin: OFF", bg="SystemButtonFace")
        is_spinning = False
    else:
        send_command("T", 60.0) # Change 35.0 to adjust spin speed
        btn_autospin.config(text="Auto Spin: ON", bg="lightgreen")
        is_spinning = True

btn_autospin = tk.Button(nav_frame, text="Auto Spin: OFF", width=15, command=toggle_spin)
btn_autospin.grid(row=3, column=1, pady=15)

btn_stand = tk.Button(root, text="Execute Stand Up Sequence", bg="yellow", command=lambda: send_command("S", 1.0))
btn_stand.pack(pady=10)

root.mainloop()