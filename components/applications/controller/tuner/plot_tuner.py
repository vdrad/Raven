import customtkinter as ctk
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
from tkinter import filedialog
import io
import re

# ==========================================
# VISUAL IDENTITY
# ==========================================
BG_COLOR = "#1E1E24"      
PANEL_COLOR = "#282931"   
ACCENT_COLOR = "#8A2BE2"  

# Motor 0 (Left) Colors
READING_0_COLOR = "#FF1C42" # Red/Pink
OUTPUT_0_COLOR = "#2EFFD2"  # Cyan
ERROR_0_COLOR = "#FFD700"   # Yellow

# Motor 1 (Right) Colors
READING_1_COLOR = "#742BB8" # Purple
OUTPUT_1_COLOR = "#71F160"  # Neon Green
ERROR_1_COLOR = "#FF4D4D"   # Red

SETPOINT_COLOR = "#FDF5FF" 
DIST_COLOR = OUTPUT_0_COLOR      # Light Blue for Distance

TITLE_FONT = ("SAKURATA", 24) 
SUBTITLE_FONT = ("Segoe UI", 14, "bold")
MONO_FONT = ("Segoe UI", 14, "bold") # Reduced slightly to fit detailed distance text

class DataProcessor:
    """Processes 10-column dual motor logs with Metadata Header for Trapezoidal Profiles."""
    
    @staticmethod
    def parse_log_file(filepath):
        csv_data = []
        metadata_line = ""
        is_capturing = False
        
        with open(filepath, 'r', encoding='utf-8') as file:
            for line in file:
                if "--- CSV START ---" in line:
                    is_capturing = True
                    continue
                if "--- CSV END ---" in line:
                    break
                if is_capturing and "TUNER:" in line:
                    clean_line = line.split("TUNER:")[1].strip()
                    # Capture the new Metadata row for PID Gains
                    if clean_line.startswith("Metadata"):
                        metadata_line = clean_line
                    else:
                        csv_data.append(clean_line)
        
        if not csv_data:
            raise ValueError("No CSV data found.")
            
        df = pd.read_csv(io.StringIO('\n'.join(csv_data)))
        
        # Parse Metadata for Gains using Regex
        gains = {"0": {"p": 0.0, "i": 0.0, "d": 0.0}, "1": {"p": 0.0, "i": 0.0, "d": 0.0}}
        if metadata_line:
            for suffix in ["0", "1"]:
                match = re.search(f"PID_{suffix}\\(P:([\\d.]+) I:([\\d.]+) D:([\\d.]+)\\)", metadata_line)
                if match:
                    gains[suffix]["p"] = float(match.group(1))
                    gains[suffix]["i"] = float(match.group(2))
                    gains[suffix]["d"] = float(match.group(3))
        
        # Store gains inside the dataframe attributes for easy access
        df.attrs['gains'] = gains

        if not df.empty:
            df['Tick'] = df['Tick'] - df['Tick'].iloc[0]
            df['Time_ms'] = (df['DeltaTime_us'].cumsum() - df['DeltaTime_us'].iloc[0]) / 1000.0
            
            # Calculate Errors for both
            if 'Setpoint_0' in df.columns and 'Reading_0' in df.columns:
                df['Error_0'] = df['Setpoint_0'] - df['Reading_0']
            if 'Setpoint_1' in df.columns and 'Reading_1' in df.columns:
                df['Error_1'] = df['Setpoint_1'] - df['Reading_1']
        
        return df

    @staticmethod
    def calculate_metrics(df, suffix="0"):
        if df.empty or f'Setpoint_{suffix}' not in df.columns: 
            return 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
            
        # 1. Max Tracking Error (Worst case deviation)
        max_track_err = df[f'Error_{suffix}'].abs().max()
        
        # 2. Overall RMSE (Root Mean Square Error for the whole profile)
        rmse = np.sqrt((df[f'Error_{suffix}'] ** 2).mean())
        
        # 3. Cruise RMSE (Error only during the flat top of the trapezoid)
        max_setpoint = df[f'Setpoint_{suffix}'].max()
        # Consider "Cruise" as any time the setpoint is above 99% of its maximum
        cruise_mask = df[f'Setpoint_{suffix}'] >= (max_setpoint * 0.99)
        if cruise_mask.any():
            cruise_rmse = np.sqrt((df.loc[cruise_mask, f'Error_{suffix}'] ** 2).mean())
        else:
            cruise_rmse = 0.0
        
        avg_dt = df['DeltaTime_us'].iloc[1:].mean() if len(df) > 1 else 0
        freq = 1000000.0 / avg_dt if avg_dt > 0 else 0
        
        # Retrieve gains parsed from metadata
        gains = df.attrs.get('gains', {}).get(suffix, {"p": 0.0, "i": 0.0, "d": 0.0})
        kp, ki, kd = gains["p"], gains["i"], gains["d"]

        # 4. Distance Integration using Trapezoidal Rule (Velocity * Time)
        # Time in seconds for standard unit integration (mm/s * s = mm)
        time_s = df['Time_ms'] / 1000.0 
        dist_setpoint = np.trapz(df[f'Setpoint_{suffix}'], time_s) if f'Setpoint_{suffix}' in df.columns else 0.0
        dist_reading = np.trapz(df[f'Reading_{suffix}'], time_s) if f'Reading_{suffix}' in df.columns else 0.0
        
        return max_track_err, rmse, cruise_rmse, avg_dt, freq, kp, ki, kd, dist_setpoint, dist_reading

class DarkToolbar(NavigationToolbar2Tk):
    def __init__(self, canvas, window):
        super().__init__(canvas, window, pack_toolbar=False)
        self.config(background=PANEL_COLOR)
        for child in self.winfo_children():
            child.config(background=PANEL_COLOR)

class TelemetryPlotter:
    """Graphics Engine with Synchronized Zoom, Auto-Scaling, and Motor Toggles."""
    
    def __init__(self, parent_frame):
        self.fig, (self.ax1, self.ax2, self.ax3) = plt.subplots(
            3, 1, figsize=(10, 8), sharex=True, gridspec_kw={'height_ratios': [2.5, 1.5, 1.5]}
        )
        self.fig.patch.set_facecolor(PANEL_COLOR)
        
        self.canvas = FigureCanvasTkAgg(self.fig, master=parent_frame)
        self.canvas_widget = self.canvas.get_tk_widget()
        self.canvas_widget.pack(fill=ctk.BOTH, expand=True)

        self.toolbar = DarkToolbar(self.canvas, parent_frame)
        self.toolbar.update()
        self.toolbar.pack(side=ctk.BOTTOM, fill=ctk.X)
        
        self.ax1.callbacks.connect('xlim_changed', self.on_xlims_change)
        
        self.df = None 
        self.show_0 = True
        self.show_1 = True
        self.setup_axes()

    def setup_axes(self):
        for ax in (self.ax1, self.ax2, self.ax3):
            ax.set_facecolor(BG_COLOR)
            ax.tick_params(colors='#888888', labelsize=9)
            ax.grid(True, color='#333338', linestyle='-', alpha=0.4)
            for spine in ax.spines.values():
                spine.set_color('#3A3A45')

        self.ax1.set_title("VELOCITY RESPONSE [mm/s]", color='white', fontsize=10, fontweight='bold', loc='left')
        self.ax2.set_title("MOTOR OUTPUT [Volts]", color='white', fontsize=10, fontweight='bold', loc='left')
        self.ax3.set_title("TRACKING ERROR [mm/s]", color='white', fontsize=10, fontweight='bold', loc='left')
        self.fig.tight_layout()

    def on_xlims_change(self, event_ax):
        if self.df is None or self.df.empty: return

        cur_xlim = event_ax.get_xlim()
        mask = (self.df['Tick'] >= cur_xlim[0]) & (self.df['Tick'] <= cur_xlim[1])
        visible = self.df.loc[mask]

        if not visible.empty:
            y1_min, y1_max = float('inf'), float('-inf')
            y2_min, y2_max = float('inf'), float('-inf')
            y3_min, y3_max = float('inf'), float('-inf')

            # Calculate limits based ONLY on visible motors
            if self.show_0 and 'Reading_0' in visible.columns:
                y1_min = min(y1_min, visible['Reading_0'].min(), visible['Setpoint_0'].min())
                y1_max = max(y1_max, visible['Reading_0'].max(), visible['Setpoint_0'].max())
                y2_min = min(y2_min, visible['Output_0'].min())
                y2_max = max(y2_max, visible['Output_0'].max())
                y3_min = min(y3_min, visible['Error_0'].min())
                y3_max = max(y3_max, visible['Error_0'].max())
                
            if self.show_1 and 'Reading_1' in visible.columns:
                y1_min = min(y1_min, visible['Reading_1'].min(), visible['Setpoint_1'].min())
                y1_max = max(y1_max, visible['Reading_1'].max(), visible['Setpoint_1'].max())
                y2_min = min(y2_min, visible['Output_1'].min())
                y2_max = max(y2_max, visible['Output_1'].max())
                y3_min = min(y3_min, visible['Error_1'].min())
                y3_max = max(y3_max, visible['Error_1'].max())

            if y1_min != float('inf'):
                pad1 = (y1_max - y1_min) * 0.12 or 1.0
                self.ax1.set_ylim(y1_min - pad1, y1_max + pad1)
            if y2_min != float('inf'):
                pad2 = (y2_max - y2_min) * 0.12 or 1.0
                self.ax2.set_ylim(y2_min - pad2, y2_max + pad2)
            if y3_min != float('inf'):
                pad3 = (y3_max - y3_min) * 0.12 or 1.0
                self.ax3.set_ylim(y3_min - pad3, y3_max + pad3)

            self.canvas.draw_idle()

    def update_plots(self, df, show_0=True, show_1=True):
        self.df = df 
        self.show_0 = show_0
        self.show_1 = show_1
        
        for ax in (self.ax1, self.ax2, self.ax3): ax.clear()
        self.setup_axes()

        if df is None or df.empty: return
        x = df['Tick']

        if show_0 and 'Setpoint_0' in df.columns:
            self.ax1.plot(x, df['Setpoint_0'], color=SETPOINT_COLOR, label='Target', linewidth=1.2, linestyle='--', alpha=0.6)
        elif show_1 and 'Setpoint_1' in df.columns:
            self.ax1.plot(x, df['Setpoint_1'], color=SETPOINT_COLOR, label='Target', linewidth=1.2, linestyle='--', alpha=0.6)

        # Plot Motor 0 (Left)
        if show_0 and 'Reading_0' in df.columns:
            self.ax1.plot(x, df['Reading_0'], color=READING_0_COLOR, label='Left (0)', linewidth=2.5, zorder=5)
            self.ax1.fill_between(x, df['Reading_0'], color=READING_0_COLOR, alpha=0.05)
            self.ax2.plot(x, df['Output_0'], color=OUTPUT_0_COLOR, label='Out Left', linewidth=2)
            self.ax2.fill_between(x, df['Output_0'], color=OUTPUT_0_COLOR, alpha=0.1)
            self.ax3.plot(x, df['Error_0'], color=ERROR_0_COLOR, label='Err Left', linewidth=1.5)
            self.ax3.fill_between(x, df['Error_0'], 0, color=ERROR_0_COLOR, alpha=0.1)

        # Plot Motor 1 (Right)
        if show_1 and 'Reading_1' in df.columns:
            self.ax1.plot(x, df['Reading_1'], color=READING_1_COLOR, label='Right (1)', linewidth=2.5, zorder=5)
            self.ax1.fill_between(x, df['Reading_1'], color=READING_1_COLOR, alpha=0.05)
            self.ax2.plot(x, df['Output_1'], color=OUTPUT_1_COLOR, label='Out Right', linewidth=2)
            self.ax2.fill_between(x, df['Output_1'], color=OUTPUT_1_COLOR, alpha=0.1)
            self.ax3.plot(x, df['Error_1'], color=ERROR_1_COLOR, label='Err Right', linewidth=1.5)
            self.ax3.fill_between(x, df['Error_1'], 0, color=ERROR_1_COLOR, alpha=0.1)

        self.ax3.axhline(y=0, color='white', linestyle='-', alpha=0.2)
        
        if self.ax1.get_legend_handles_labels()[0]:
            self.ax1.legend(loc='lower right', facecolor=PANEL_COLOR, labelcolor='white', fontsize=8)
        if self.ax2.get_legend_handles_labels()[0]:
            self.ax2.legend(loc='upper right', facecolor=PANEL_COLOR, labelcolor='white', fontsize=8)

        if len(x) > 0:
            self.ax1.set_xlim(left=0, right=x.max())
        
        self.on_xlims_change(self.ax1)
        self.canvas.draw()


class PIDTunerDashboard(ctk.CTk):
    def __init__(self):
        super().__init__()
        self.title("RAVEN PID TUNER | DUAL MOTOR ANALYSIS")
        self.geometry("1400x940") # Slightly taller to accommodate new KPIs
        self.configure(fg_color=BG_COLOR)
        
        self.processor = DataProcessor()
        self.current_df = None
        self.last_filepath = None  # TRACK THE LAST IMPORTED FILE
        self.build_ui()

    def build_ui(self):
        header = ctk.CTkFrame(self, fg_color=PANEL_COLOR, height=75, corner_radius=0)
        header.pack(side=ctk.TOP, fill=ctk.X)
        
        ctk.CTkLabel(header, text="RAVEN | PID TUNER", font=TITLE_FONT, text_color=ACCENT_COLOR).pack(side=ctk.LEFT, padx=25)
        
        self.gain_info = ctk.StringVar(value="Gains: Awaiting Telemetry")
        ctk.CTkLabel(header, textvariable=self.gain_info, font=("Consolas", 12), text_color="#AAAAAA").pack(side=ctk.LEFT, padx=40)
        
        # IMPORT LOG BUTTON
        ctk.CTkButton(header, text="IMPORT LOG", font=SUBTITLE_FONT, fg_color=ACCENT_COLOR, 
                      hover_color="#6B1EAD", command=self.handle_import).pack(side=ctk.RIGHT, padx=(10, 25))

        # REFRESH BUTTON (Initially Disabled)
        self.refresh_btn = ctk.CTkButton(header, text="REFRESH", font=SUBTITLE_FONT, fg_color="#4A4A55", 
                                         hover_color="#5D5D6A", width=90, state="disabled", command=self.handle_refresh)
        self.refresh_btn.pack(side=ctk.RIGHT, padx=10)

        container = ctk.CTkFrame(self, fg_color=BG_COLOR)
        container.pack(fill=ctk.BOTH, expand=True, padx=20, pady=20)

        self.side_panel = ctk.CTkFrame(container, fg_color=BG_COLOR, width=330)
        self.side_panel.pack(side=ctk.LEFT, fill=ctk.Y, padx=(0, 20))
        
        toggle_frame = ctk.CTkFrame(self.side_panel, fg_color=PANEL_COLOR, corner_radius=8)
        toggle_frame.pack(fill=ctk.X, pady=(0, 15), ipady=5)
        
        self.show_left_var = ctk.BooleanVar(value=True)
        self.show_right_var = ctk.BooleanVar(value=True)
        
        ctk.CTkCheckBox(toggle_frame, text="Left (0)", variable=self.show_left_var, 
                        command=self.refresh_plots, fg_color=READING_0_COLOR, text_color=READING_0_COLOR).pack(side=ctk.LEFT, padx=15, pady=10)
        ctk.CTkCheckBox(toggle_frame, text="Right (1)", variable=self.show_right_var, 
                        command=self.refresh_plots, fg_color=READING_1_COLOR, text_color=READING_1_COLOR).pack(side=ctk.RIGHT, padx=15, pady=10)
        
        # Track all KPI strings
        self.vars = {k: ctk.StringVar(value="--") for k in ["max_err", "rmse", "cruise", "exp_dist", "act_dist", "dt", "freq"]}
        
        self.add_kpi("MAX TRACK ERR [L | R]", self.vars["max_err"], READING_0_COLOR)
        self.add_kpi("OVERALL RMSE [L | R]", self.vars["rmse"], READING_0_COLOR)
        self.add_kpi("CRUISE RMSE [L | R]", self.vars["cruise"], READING_0_COLOR)
        
        # New Odometry Distances
        self.add_kpi("EXPECTED DIST [ROBOT m]", self.vars["exp_dist"], DIST_COLOR)
        self.add_kpi("ACTUAL DIST [ROBOT m]", self.vars["act_dist"], DIST_COLOR)

        self.add_kpi("AVG SAMPLE TIME [μs]", self.vars["dt"], "#AAAAAA")
        self.add_kpi("LOOP FREQUENCY [Hz]", self.vars["freq"], ACCENT_COLOR)

        plot_frame = ctk.CTkFrame(container, fg_color=PANEL_COLOR, corner_radius=12)
        plot_frame.pack(side=ctk.RIGHT, fill=ctk.BOTH, expand=True)
        self.plotter = TelemetryPlotter(plot_frame)

    def add_kpi(self, title, var, color):
        card = ctk.CTkFrame(self.side_panel, fg_color=PANEL_COLOR, corner_radius=8)
        card.pack(fill=ctk.X, pady=(0, 10), ipady=6)
        ctk.CTkLabel(card, text=title, font=("Segoe UI", 10, "bold"), text_color="#777777").pack()
        ctk.CTkLabel(card, textvariable=var, font=MONO_FONT, text_color=color).pack()

    def refresh_plots(self):
        if self.current_df is not None:
            self.plotter.update_plots(self.current_df, show_0=self.show_left_var.get(), show_1=self.show_right_var.get())

    # SHARED LOAD LOGIC
    def load_file(self, path):
        try:
            self.current_df = self.processor.parse_log_file(path)
            self.last_filepath = path
            
            # Enable refresh button if disabled
            self.refresh_btn.configure(state="normal")
            
            # Metrics for Motor 0 (Left)
            max_err0, rmse0, cruise0, dt, freq, kp0, ki0, kd0, exp_d0, act_d0 = self.processor.calculate_metrics(self.current_df, "0")
            # Metrics for Motor 1 (Right)
            max_err1, rmse1, cruise1, _, _, kp1, ki1, kd1, exp_d1, act_d1 = self.processor.calculate_metrics(self.current_df, "1")
            
            # Update KPI texts side-by-side (Left | Right)
            self.vars["max_err"].set(f"{max_err0:.1f} | {max_err1:.1f}")
            self.vars["rmse"].set(f"{rmse0:.1f} | {rmse1:.1f}")
            self.vars["cruise"].set(f"{cruise0:.1f} | {cruise1:.1f}")
            self.vars["dt"].set(f"{dt:.1f}")
            self.vars["freq"].set(f"{int(freq)}")
            
            # Differential Drive Kinematics (Center point distance is the average of both wheels)
            exp_dist_robot = (exp_d0 + exp_d1) / (2.0 * 1000)
            act_dist_robot = (act_d0 + act_d1) / (2.0 * 1000)

            # Format explicitly combining Robot Average with individual wheel breakdowns
            self.vars["exp_dist"].set(f"{exp_dist_robot:.3f}  (L:{exp_d0/1000:.3f}|R:{exp_d1/1000:.3f})")
            self.vars["act_dist"].set(f"{act_dist_robot:.3f}  (L:{act_d0/1000:.3f}|R:{act_d1/1000:.3f})")

            # Update Gains (Left | Right)
            gains_txt = (f"L0: P={kp0:.5f} I={ki0:.5f} D={kd0:.5f}  ||  "
                         f"R1: P={kp1:.5f} I={ki1:.5f} D={kd1:.5f}")
            self.gain_info.set(gains_txt)
            
            self.refresh_plots()
        except Exception as e:
            print(f"Loading Error: {e}")

    def handle_import(self):
        path = filedialog.askopenfilename(filetypes=[("Logs", "*.txt")])
        if path:
            self.load_file(path)

    def handle_refresh(self):
        if self.last_filepath:
            self.load_file(self.last_filepath)


if __name__ == "__main__":
    app = PIDTunerDashboard()
    app.mainloop()