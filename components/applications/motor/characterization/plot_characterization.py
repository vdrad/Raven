import customtkinter as ctk
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
from tkinter import filedialog
import matplotlib.cm as cm
from matplotlib.colors import LinearSegmentedColormap # <-- Import adicionado para a paleta customizada

# ==========================================
# VISUAL IDENTITY & THEME (F1 TELEMETRY STYLE)
# ==========================================
BG_COLOR = "#111111"      # Deep Black for the background
PANEL_COLOR = "#1A1A1E"   # Dark Carbon Grey for panels
BORDER_COLOR = "#33333C"  # Subtle borders for instrument boxes
ACCENT_COLOR = "#8A2BE2"  
TEXT_COLOR = "#E0E0E0"

# Motor Colors
LEFT_COLOR = "#742BB8"  # Purple
RIGHT_COLOR = "#742BB8" # Purple

# Typography
TITLE_FONT = ("SAKURATA", 20, "bold") 
SUBTITLE_FONT = ("JetBrains Mono", 13, "bold")
LBL_FONT = ("Titillium Web", 11, "bold")
VAL_FONT = ("JetBrains Mono", 34, "bold")
MONO_FONT = ("JetBrains Mono", 13) 

class DataProcessor:
    """Processes characterization logs and extracts steady-state & transient metrics."""
    
    @staticmethod
    def parse_log_file(filepath):
        datasets = {}
        current_voltage = None
        data_rows = []
        
        with open(filepath, 'r') as file:
            for line in file:
                if "MCH: " in line:
                    clean_line = line.split("MCH: ")[1].strip()
                else:
                    clean_line = line.strip()
                    
                if not clean_line:
                    continue
                    
                if clean_line.startswith("Metadata"):
                    parts = clean_line.split(',')
                    volt_str = parts[1].split(':')[1].replace('V', '')
                    current_voltage = float(volt_str)
                    data_rows = []
                
                elif clean_line.startswith("--- CSV END ---") and current_voltage is not None:
                    if data_rows:
                        df = pd.DataFrame(data_rows, columns=['Point', 'DeltaTime_us', 'Vel_Left', 'Vel_Right'])
                        df = df.astype({'DeltaTime_us': float, 'Vel_Left': float, 'Vel_Right': float})
                        df['Time_s'] = df['DeltaTime_us'].cumsum() / 1e6
                        datasets[current_voltage] = df
                    current_voltage = None
                    
                elif current_voltage is not None and clean_line[0].isdigit():
                    parts = clean_line.split(',')
                    if len(parts) >= 4:
                        data_rows.append(parts)
                        
        return datasets

    @staticmethod
    def get_steady_state_velocities(datasets):
        steady_states = {'voltage': [], 'left': [], 'right': []}
        
        for voltage, df in datasets.items():
            tail_len = max(1, int(len(df) * 0.2))
            tail_df = df.tail(tail_len)
            
            steady_left = tail_df['Vel_Left'].mean()
            steady_right = tail_df['Vel_Right'].mean()
            
            steady_states['voltage'].append(voltage)
            steady_states['left'].append(steady_left)
            steady_states['right'].append(steady_right)
            
        return pd.DataFrame(steady_states)

    @staticmethod
    def calculate_feedforward(voltages, velocities):
        active_indices = np.where(np.abs(velocities) > 0.05)[0]
        if len(active_indices) < 2:
            return 0.0, 0.0 
            
        active_v = velocities[active_indices]
        active_volts = voltages[active_indices]
        Kv, Ks = np.polyfit(active_v, active_volts, 1)
        return Kv, Ks

    @staticmethod
    def get_transient_metrics(datasets, steady_df, motor_choice):
        col_name = f"Vel_{motor_choice.capitalize()}"
        
        max_volt = steady_df['voltage'].max()
        if pd.isna(max_volt): return 0.0, 0.0, 0.0
        
        df = datasets[max_volt]
        steady_vel = steady_df[steady_df['voltage'] == max_volt][motor_choice].values[0]
        
        if steady_vel < 0.1: 
            return 0.0, 0.0, max_volt
            
        threshold = 0.95 * steady_vel
        crossed_indices = df.index[df[col_name] >= threshold].tolist()
        settling_time = df.loc[crossed_indices[0], 'Time_s'] if crossed_indices else df['Time_s'].iloc[-1]
        
        dt_array = np.gradient(df['Time_s'])
        dt_array[dt_array == 0] = 1e-6 
        dv_array = np.gradient(df[col_name])
        
        accel_array = dv_array / dt_array
        max_accel = np.max(accel_array)
        
        return settling_time, max_accel, max_volt


class CharacterizationDashboard(ctk.CTk):
    def __init__(self):
        super().__init__()
        
        self.title("RAVEN | Motor Characterization")
        self.geometry("1450x880")
        self.configure(fg_color=BG_COLOR)
        
        self.processor = DataProcessor()
        self.datasets = {}
        self.steady_df = None
        self.current_motor = ctk.StringVar(value="Left")
        
        self._build_ui()
        
    def _build_ui(self):
        # --- TOP CONTROL BAR ---
        self.top_frame = ctk.CTkFrame(self, fg_color=PANEL_COLOR, height=70, corner_radius=0, border_width=1, border_color=BORDER_COLOR)
        self.top_frame.pack(side="top", fill="x")
        self.top_frame.pack_propagate(False)
        
        self.title_lbl = ctk.CTkLabel(self.top_frame, text="RAVEN | MOTOR CHARACTERIZATION", font=TITLE_FONT, text_color=ACCENT_COLOR)
        self.title_lbl.pack(side="left", padx=20)
        
        self.import_btn = ctk.CTkButton(self.top_frame, text="IMPORT .TXT LOG", font=SUBTITLE_FONT, 
                                        fg_color=ACCENT_COLOR, hover_color="#6A1B9A",
                                        command=self.handle_import)
        self.import_btn.pack(side="right", padx=20, pady=15)
        
        self.motor_switch = ctk.CTkSegmentedButton(self.top_frame, values=["Left", "Right"],
                                                   variable=self.current_motor,
                                                   command=self.update_plots,
                                                   font=SUBTITLE_FONT,
                                                   selected_color=ACCENT_COLOR,
                                                   selected_hover_color="#6A1B9A")
        self.motor_switch.pack(side="right", padx=40, pady=15)
        
        # --- MAIN CONTENT ---
        self.main_container = ctk.CTkFrame(self, fg_color=BG_COLOR)
        self.main_container.pack(fill="both", expand=True, padx=15, pady=15)
        
        # --- PLOT AREA ---
        self.plot_frame = ctk.CTkFrame(self.main_container, fg_color=PANEL_COLOR, corner_radius=8, border_width=2, border_color=BORDER_COLOR)
        self.plot_frame.pack(side="top", fill="both", expand=True, pady=(0, 10))
        
        self.fig = plt.Figure(figsize=(14, 5), facecolor=PANEL_COLOR)
        self.ax_time = self.fig.add_subplot(121)
        self.ax_fit = self.fig.add_subplot(122)
        
        self.canvas = FigureCanvasTkAgg(self.fig, master=self.plot_frame)
        self.canvas.get_tk_widget().pack(fill="both", expand=True, padx=10, pady=10)
        
        # --- BOTTOM TELEMETRY AREA (3 BOXES) ---
        self.bottom_frame = ctk.CTkFrame(self.main_container, fg_color="transparent", height=150)
        self.bottom_frame.pack(side="bottom", fill="x")
        
        # BOX 1: PEAK DYNAMICS
        self.box_peak = ctk.CTkFrame(self.bottom_frame, fg_color=PANEL_COLOR, corner_radius=8, border_width=2, border_color=BORDER_COLOR)
        self.box_peak.pack(side="left", fill="both", expand=True, padx=(0, 5))
        
        ctk.CTkLabel(self.box_peak, text="PEAK DYNAMICS", font=SUBTITLE_FONT, text_color="#777777").pack(anchor="w", padx=15, pady=(10, 0))
        
        peak_inner = ctk.CTkFrame(self.box_peak, fg_color="transparent")
        peak_inner.pack(fill="both", expand=True, padx=15, pady=5)
        
        # Accel Block
        acc_blk = ctk.CTkFrame(peak_inner, fg_color="transparent")
        acc_blk.pack(side="left", expand=True, anchor="w")
        ctk.CTkLabel(acc_blk, text="MAX ACCEL (m/s²)", font=LBL_FONT, text_color="#555555").pack(anchor="w")
        self.val_max_accel = ctk.CTkLabel(acc_blk, text="--.--", font=VAL_FONT, text_color=TEXT_COLOR)
        self.val_max_accel.pack(anchor="w")

        # Step Block
        step_blk = ctk.CTkFrame(peak_inner, fg_color="transparent")
        step_blk.pack(side="right", expand=True, anchor="e")
        ctk.CTkLabel(step_blk, text="AT VOLTAGE (V)", font=LBL_FONT, text_color="#555555").pack(anchor="e")
        self.val_step = ctk.CTkLabel(step_blk, text="--.-", font=VAL_FONT, text_color=TEXT_COLOR)
        self.val_step.pack(anchor="e")

        # BOX 2: PID TARGET PROFILE
        self.box_target = ctk.CTkFrame(self.bottom_frame, fg_color=PANEL_COLOR, corner_radius=8, border_width=2, border_color=BORDER_COLOR)
        self.box_target.pack(side="left", fill="both", expand=True, padx=(5, 5))
        
        ctk.CTkLabel(self.box_target, text="PID TARGET PROFILE", font=SUBTITLE_FONT, text_color="#777777").pack(anchor="w", padx=15, pady=(10, 0))
        
        tgt_inner = ctk.CTkFrame(self.box_target, fg_color="transparent")
        tgt_inner.pack(fill="both", expand=True, padx=15, pady=5)
        
        # Rec Accel Block
        rec_blk = ctk.CTkFrame(tgt_inner, fg_color="transparent")
        rec_blk.pack(side="left", expand=True, anchor="w")
        ctk.CTkLabel(rec_blk, text="REC. PROFILE (m/s²)", font=LBL_FONT, text_color="#555555").pack(anchor="w")
        self.val_rec_accel = ctk.CTkLabel(rec_blk, text="--.--", font=VAL_FONT, text_color=TEXT_COLOR)
        self.val_rec_accel.pack(anchor="w")
        
        # Settling Block
        set_blk = ctk.CTkFrame(tgt_inner, fg_color="transparent")
        set_blk.pack(side="right", expand=True, anchor="e")
        ctk.CTkLabel(set_blk, text="SETTLING (95% s)", font=LBL_FONT, text_color="#555555").pack(anchor="e")
        self.val_settling = ctk.CTkLabel(set_blk, text="-.---", font=VAL_FONT, text_color=TEXT_COLOR)
        self.val_settling.pack(anchor="e")
        
        # Headroom Visualizer Bar
        bar_frame = ctk.CTkFrame(self.box_target, fg_color="transparent")
        bar_frame.pack(fill="x", padx=15, pady=(0, 15))
        ctk.CTkLabel(bar_frame, text="CONTROL HEADROOM (75%)", font=("Segoe UI", 9, "bold"), text_color="#555").pack(side="left", padx=(0, 10))
        self.headroom_bar = ctk.CTkProgressBar(bar_frame, height=8, corner_radius=2, fg_color="#222222")
        self.headroom_bar.pack(side="right", fill="x", expand=True)
        self.headroom_bar.set(0)

        # BOX 3: EXPORT
        self.box_export = ctk.CTkFrame(self.bottom_frame, fg_color=PANEL_COLOR, corner_radius=8, border_width=2, border_color=BORDER_COLOR)
        self.box_export.pack(side="left", fill="both", expand=True, padx=(5, 0))
        
        ctk.CTkLabel(self.box_export, text="TELEMETRY EXPORT (C/C++)", font=SUBTITLE_FONT, text_color="#777777").pack(anchor="w", padx=15, pady=(10, 5))
        
        self.eq_display = ctk.CTkTextbox(self.box_export, font=MONO_FONT, text_color="#00FF00", fg_color="#0A0A0A", border_width=1, border_color="#333", height=70)
        self.eq_display.pack(fill="both", expand=True, padx=15, pady=(0, 15))
        self.eq_display.insert("1.0", "// Waiting for data...")
        self.eq_display.configure(state="disabled")

    def handle_import(self):
        path = filedialog.askopenfilename(filetypes=[("Text Logs", "*.txt")])
        if path:
            self.datasets = self.processor.parse_log_file(path)
            if not self.datasets:
                return
            self.steady_df = self.processor.get_steady_state_velocities(self.datasets)
            self.update_plots()

    def update_plots(self, *args):
        if not self.datasets or self.steady_df is None:
            return
            
        motor_choice = self.current_motor.get().lower()
        col_name = f"Vel_{motor_choice.capitalize()}"
        steady_col = motor_choice
        base_color = LEFT_COLOR if motor_choice == "left" else RIGHT_COLOR
        
        # 1. Update KPIs
        settling_time, max_accel, max_volt = self.processor.get_transient_metrics(self.datasets, self.steady_df, motor_choice)
        recommended_accel = max_accel * 0.75 
        
        # Telemetry Box Updates (Digital Formatting)
        self.val_max_accel.configure(text=f"{max_accel:05.2f}", text_color=base_color)
        self.val_step.configure(text=f"{max_volt:04.1f}", text_color=TEXT_COLOR)
        
        self.val_rec_accel.configure(text=f"{recommended_accel:05.2f}", text_color=base_color)
        self.val_settling.configure(text=f"{settling_time:05.3f}", text_color=TEXT_COLOR)
        
        self.headroom_bar.set(0.75)
        self.headroom_bar.configure(progress_color=base_color)
        
        # 2. Update Plots
        self.ax_time.clear()
        self.ax_fit.clear()
        
        for ax in [self.ax_time, self.ax_fit]:
            ax.set_facecolor(BG_COLOR)
            ax.tick_params(colors=TEXT_COLOR)
            for spine in ax.spines.values():
                spine.set_color('#333333')
            ax.grid(color='#2A2A2A', linestyle='--', alpha=0.8)

        # -------------------------------------------------------------
        # CUSTOM NEON PASTEL COLORMAP (Frio para Quente Harmonicamente)
        # -------------------------------------------------------------
        voltages = list(self.datasets.keys())
        pastel_neon_anchors = ["#5CFFD2", "#5CFFFF", "#5CB8FF", "#B85CFF", "#FF5CA8", "#FF8A5C", "#FFE65C"]
        custom_cmap = LinearSegmentedColormap.from_list("pastel_neon", pastel_neon_anchors)
        colormap = custom_cmap(np.linspace(0, 1, len(voltages)))
        
        for idx, (volt, df) in enumerate(self.datasets.items()):
            color = colormap[idx]
            self.ax_time.plot(df['Time_s'], df[col_name], color=color, linewidth=2, label=f"{volt}V")
            tail_len = max(1, int(len(df) * 0.2))
            self.ax_time.plot(df['Time_s'].tail(tail_len), df[col_name].tail(tail_len), 
                              color='#FFFFFF', linewidth=3, alpha=0.3)

        self.ax_time.set_title(f"Step Responses ({motor_choice.capitalize()})", color=TEXT_COLOR, fontsize=12, pad=10)
        self.ax_time.set_xlabel("Time (s)", color="#888888")
        self.ax_time.set_ylabel("Velocity (m/s)", color="#888888")
        self.ax_time.legend(facecolor=PANEL_COLOR, edgecolor='#333333', labelcolor=TEXT_COLOR, loc='upper left', fontsize=8)

        vols = self.steady_df['voltage'].values
        vels = self.steady_df[steady_col].values
        
        self.ax_fit.scatter(vels, vols, color=base_color, s=80, edgecolors='#FFFFFF', zorder=5, label='Measured Data')
        
        Kv, Ks = self.processor.calculate_feedforward(vols, vels)
        max_target_voltage = 12.0
        max_vel_extrapolated = (max_target_voltage - Ks) / Kv if Kv != 0 else max(vels)
        
        fit_vels = np.linspace(0, max_vel_extrapolated, 100)
        fit_vols = (Kv * fit_vels) + Ks
        
        self.ax_fit.plot(fit_vels, fit_vols, color='#FF1C42', linestyle='--', linewidth=2, 
                         label=f'Linear Fit (V = {Kv:.3f}w + {Ks:.3f})', zorder=4)

        self.ax_fit.set_title("Feedforward Model", color=TEXT_COLOR, fontsize=12, pad=10)
        self.ax_fit.set_xlabel("Velocity (m/s)", color="#888888")
        self.ax_fit.set_ylabel("Applied Voltage (V)", color="#888888")
        self.ax_fit.legend(facecolor=PANEL_COLOR, edgecolor='#333333', labelcolor=TEXT_COLOR)
        
        self.fig.tight_layout()
        self.canvas.draw()
        
        # 3. Update Code Snippet (Using Textbox for multiline)
        self.eq_display.configure(state="normal")
        self.eq_display.delete("1.0", "end")
        
        motor_prefix = "LEFT" if motor_choice == "left" else "RIGHT"
        code_str = (f"// --- {motor_prefix} MOTOR PROFILE ---\n"
                    f"#define ACCEL_RATE_{motor_prefix}_M_S2 {recommended_accel:.2f}f\n"
                    f"float {motor_choice}_ff = ({Kv:.5f}f * target_vel) + {Ks:.5f}f;")
                    
        self.eq_display.insert("1.0", code_str)
        self.eq_display.configure(state="disabled")

if __name__ == "__main__":
    ctk.set_appearance_mode("dark")
    app = CharacterizationDashboard()
    app.mainloop()