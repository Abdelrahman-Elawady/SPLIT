import tkinter as tk
from tkinter import filedialog
import serial
import time
import csv
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.animation import FuncAnimation

SERIAL_PORT = 'COM7' 
BAUD_RATE = 115200

try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.05)
    time.sleep(2) 
except Exception as e:
    print(f"Error opening serial port: {e}")
    exit()

def send_command(prefix, value):
    command = f"{prefix}:{value}\n"
    ser.write(command.encode('utf-8'))

root = tk.Tk()
root.title("Dual-Graph Balancing Telemetry")
root.geometry("1500x900")

left_panel = tk.Frame(root, width=350, bd=2, relief="groove")
left_panel.pack(side=tk.LEFT, fill=tk.Y, padx=10, pady=10)

right_panel = tk.Frame(root)
right_panel.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True, padx=10, pady=10)

# --- Left Panel: PID & Controls ---
tk.Label(left_panel, text="PID Tuning", font=("Arial", 14, "bold")).pack(pady=5)

def create_slider(parent, label, prefix, min_val, max_val, res):
    frame = tk.Frame(parent)
    frame.pack(fill="x", pady=2, padx=5)
    tk.Label(frame, text=label, width=12, anchor="w").pack(side="left")
    slider = tk.Scale(frame, from_=min_val, to=max_val, resolution=res, orient="horizontal", length=140)
    slider.pack(side="left")
    tk.Button(frame, text="Send", command=lambda: send_command(prefix, slider.get())).pack(side="right")

create_slider(left_panel, "Angle Kp (P)", "P", 0, 800, 1)
create_slider(left_panel, "Angle Kd (D)", "D", 0, 20, 0.1)
create_slider(left_panel, "Speed Kp (V)", "V", 0, 5, 0.05)
create_slider(left_panel, "Speed Ki (U)", "U", 0, 0.5, 0.01)
create_slider(left_panel, "Yaw Kp (Y)", "Y", 0, 5, 0.1) # Added missing Yaw slider

# Gripper
tk.Label(left_panel, text="Servo Gripper", font=("Arial", 14, "bold")).pack(pady=(15, 5))
grip_frame = tk.Frame(left_panel)
grip_frame.pack()
tk.Button(grip_frame, text="Open", bg="lightblue", width=10, command=lambda: send_command("G", 150)).pack(side="left", padx=5)
tk.Button(grip_frame, text="Close", bg="lightgreen", width=10, command=lambda: send_command("G", 60)).pack(side="left", padx=5)

# Navigation
tk.Label(left_panel, text="Directional Control", font=("Arial", 14, "bold")).pack(pady=(15, 5))
nav_frame = tk.Frame(left_panel)
nav_frame.pack()
btn_fwd = tk.Button(nav_frame, text="FORWARD", width=12, bg="#e0e0e0")
btn_fwd.grid(row=0, column=1, pady=5)
btn_fwd.bind('<ButtonPress-1>', lambda e: send_command("B", 6.0))
btn_fwd.bind('<ButtonRelease-1>', lambda e: send_command("B", 0.0))
btn_left = tk.Button(nav_frame, text="LEFT", width=12, bg="#e0e0e0")
btn_left.grid(row=1, column=0, padx=5)
btn_left.bind('<ButtonPress-1>', lambda e: send_command("T", 60.0))
btn_left.bind('<ButtonRelease-1>', lambda e: send_command("T", 0.0))
btn_right = tk.Button(nav_frame, text="RIGHT", width=12, bg="#e0e0e0")
btn_right.grid(row=1, column=2, padx=5)
btn_right.bind('<ButtonPress-1>', lambda e: send_command("T", -60.0))
btn_right.bind('<ButtonRelease-1>', lambda e: send_command("T", 0.0))
btn_rev = tk.Button(nav_frame, text="REVERSE", width=12, bg="#e0e0e0")
btn_rev.grid(row=2, column=1, pady=5)
btn_rev.bind('<ButtonPress-1>', lambda e: send_command("B", -6.0))
btn_rev.bind('<ButtonRelease-1>', lambda e: send_command("B", 0.0))

lbl_battery = tk.Label(left_panel, text="Battery: -- V", font=("Arial", 16, "bold"), fg="green")
lbl_battery.pack(pady=20)

# --- Right Panel: Graph Toggles ---
chk_frame = tk.Frame(right_panel)
chk_frame.pack(pady=5, fill="x")

tk.Label(chk_frame, text="Top Graph (Angles):", font=("Arial", 10, "bold")).grid(row=0, column=0, sticky="e", padx=5)
show_ang = tk.BooleanVar(value=True)
show_targ = tk.BooleanVar(value=True)
show_err = tk.BooleanVar(value=True)
tk.Checkbutton(chk_frame, text="Angle", variable=show_ang, fg="red").grid(row=0, column=1)
tk.Checkbutton(chk_frame, text="Setpoint", variable=show_targ, fg="blue").grid(row=0, column=2)
tk.Checkbutton(chk_frame, text="Error", variable=show_err, fg="purple").grid(row=0, column=3)

tk.Label(chk_frame, text="Bottom Graph (PID/Motors):", font=("Arial", 10, "bold")).grid(row=1, column=0, sticky="e", padx=5)
show_p = tk.BooleanVar(value=True)
show_i = tk.BooleanVar(value=True)
show_d = tk.BooleanVar(value=True)
show_spd = tk.BooleanVar(value=False)
show_pos = tk.BooleanVar(value=False)
show_mL = tk.BooleanVar(value=False)
show_mR = tk.BooleanVar(value=False)
tk.Checkbutton(chk_frame, text="P-Term", variable=show_p, fg="red").grid(row=1, column=1)
tk.Checkbutton(chk_frame, text="I-Term", variable=show_i, fg="green").grid(row=1, column=2)
tk.Checkbutton(chk_frame, text="D-Term", variable=show_d, fg="blue").grid(row=1, column=3)
tk.Checkbutton(chk_frame, text="Speed In", variable=show_spd, fg="orange").grid(row=1, column=4)
tk.Checkbutton(chk_frame, text="Pos Error", variable=show_pos, fg="cyan").grid(row=1, column=5)
tk.Checkbutton(chk_frame, text="Motor L", variable=show_mL, fg="black").grid(row=1, column=6)
tk.Checkbutton(chk_frame, text="Motor R", variable=show_mR, fg="gray").grid(row=1, column=7)

# --- Dual Graph Setup ---
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8), gridspec_kw={'height_ratios': [1, 1.5]})
canvas = FigureCanvasTkAgg(fig, master=right_panel)
canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

MAX_PTS = 150
time_data = [0]*MAX_PTS
d = {k: [0]*MAX_PTS for k in ['ang', 'targ', 'err', 'p', 'i', 'd', 'spd', 'pos', 'mL', 'mR']}

line_ang, = ax1.plot(time_data, d['ang'], label="Angle", color="red")
line_targ, = ax1.plot(time_data, d['targ'], label="Target", color="blue", linestyle="--")
line_err, = ax1.plot(time_data, d['err'], label="Error", color="purple")
ax1.set_ylabel("Degrees / Radians")
ax1.set_ylim(-40, 40)
ax1.legend(loc="upper right")
ax1.grid(True)

line_p, = ax2.plot(time_data, d['p'], label="P-Term", color="red")
line_i, = ax2.plot(time_data, d['i'], label="I-Term", color="green")
line_d, = ax2.plot(time_data, d['d'], label="D-Term", color="blue")
line_spd, = ax2.plot(time_data, d['spd'], label="Speed In", color="orange")
line_pos, = ax2.plot(time_data, d['pos'], label="Pos Error", color="cyan")
line_mL, = ax2.plot(time_data, d['mL'], label="Motor L", color="black", alpha=0.5)
line_mR, = ax2.plot(time_data, d['mR'], label="Motor R", color="gray", alpha=0.5)
ax2.set_ylabel("Raw PWM / Math Output")
ax2.set_ylim(-300, 300)
ax2.legend(loc="upper right", ncol=3)
ax2.grid(True)

log_data = [["Time", "Angle", "Target", "Error", "P", "I", "D", "SpeedIn", "PosErr", "MotL", "MotR", "Battery"]]
start_time = time.time()
logging_active = False

def update_graph(frame):
    ax1.set_xlim(time_data[0], time_data[-1])
    ax2.set_xlim(time_data[0], time_data[-1])
    
    line_ang.set_data(time_data, d['ang']); line_ang.set_visible(show_ang.get())
    line_targ.set_data(time_data, d['targ']); line_targ.set_visible(show_targ.get())
    line_err.set_data(time_data, d['err']); line_err.set_visible(show_err.get())
    
    line_p.set_data(time_data, d['p']); line_p.set_visible(show_p.get())
    line_i.set_data(time_data, d['i']); line_i.set_visible(show_i.get())
    line_d.set_data(time_data, d['d']); line_d.set_visible(show_d.get())
    line_spd.set_data(time_data, d['spd']); line_spd.set_visible(show_spd.get())
    line_pos.set_data(time_data, d['pos']); line_pos.set_visible(show_pos.get())
    line_mL.set_data(time_data, d['mL']); line_mL.set_visible(show_mL.get())
    line_mR.set_data(time_data, d['mR']); line_mR.set_visible(show_mR.get())

ani = FuncAnimation(fig, update_graph, interval=100)

def read_serial():
    global time_data, d, log_data
    while ser.in_waiting > 0:
        try:
            line = ser.readline().decode('utf-8').strip()
            if line.startswith("T:"):
                raw = line[2:].split(',')
                if len(raw) == 11:
                    vals = list(map(float, raw))
                    t_now = round(time.time() - start_time, 2)
                    
                    time_data.pop(0); time_data.append(t_now)
                    keys = ['ang', 'targ', 'err', 'p', 'i', 'd', 'spd', 'pos', 'mL', 'mR']
                    for i, key in enumerate(keys):
                        d[key].pop(0); d[key].append(vals[i])
                    
                    if logging_active:
                        log_data.append([t_now] + vals)
                    
                    bat = vals[10]
                    lbl_battery.config(text=f"Battery: {bat:.2f} V")
                    if bat < 10.5: lbl_battery.config(fg="red")
                    else: lbl_battery.config(fg="green")
        except Exception:
            pass
    root.after(20, read_serial)

def toggle_logging():
    global logging_active
    if logging_active:
        logging_active = False
        btn_log.config(text="Start Recording Data", bg="SystemButtonFace")
    else:
        logging_active = True
        btn_log.config(text="Recording Active...", bg="red")

def export_csv():
    filepath = filedialog.asksaveasfilename(defaultextension=".csv", filetypes=[("CSV Files", "*.csv")])
    if filepath:
        with open(filepath, 'w', newline='') as file:
            writer = csv.writer(file)
            writer.writerows(log_data)

btn_frame = tk.Frame(right_panel)
btn_frame.pack(pady=5)
btn_log = tk.Button(btn_frame, text="Start Recording Data", font=("Arial", 10, "bold"), command=toggle_logging)
btn_log.pack(side="left", padx=10)
tk.Button(btn_frame, text="Export CSV", font=("Arial", 10, "bold"), command=export_csv).pack(side="left", padx=10)

read_serial()
root.mainloop()