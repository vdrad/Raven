import dash
from dash import dcc, html
from dash.dependencies import Input, Output, State
import plotly.graph_objects as go
import pandas as pd
import numpy as np
import io
import base64

# ==========================================
# 1. CORE DATA PROCESSORS
# ==========================================
def parse_telemetry_text(text):
    clean_csv_lines = []
    headers = "Time_ms,Dist_mm,X_mm,Y_mm,Yaw_mrad,VelL_mmps,VelR_mmps,LinePos,PidLine,PidLSet,PidRSet,PidLOut,PidROut,Bat_dv,Markers,Fan_dv"
    clean_csv_lines.append(headers)
    for line in text.split('\n'):
        if "[TLM]" in line:
            if "TELEMETRY START" in line or "TELEMETRY END" in line or "Time_ms" in line:
                continue
            parts = line.split("[TLM] ")
            if len(parts) > 1:
                csv_payload = parts[1].strip()
                if "," in csv_payload:
                    clean_csv_lines.append(csv_payload)
    csv_text = "\n".join(clean_csv_lines)
    return pd.read_csv(io.StringIO(csv_text))

def process_telemetry_df(df_raw):
    df = df_raw.copy()
    if df.empty: return df
    
    df['Time_s'] = (df['Time_ms'] - df['Time_ms'].min()) / 1000.0
    df['X_m'] = df['X_mm'] / 1000.0
    df['Y_m'] = df['Y_mm'] / 1000.0
    df['Yaw_rad'] = df['Yaw_mrad'] / 1000.0

    df['VelL_mps'] = df['VelL_mmps'] / 1000.0
    df['VelR_mps'] = df['VelR_mmps'] / 1000.0
    df['Robot_Vel'] = (df['VelL_mps'] + df['VelR_mps']) / 2.0
    df['PidLSet_mps'] = df['PidLSet'] / 1000.0
    df['PidRSet_mps'] = df['PidRSet'] / 1000.0
    df['PidLine_mps'] = df['PidLine'] / 1000.0

    df['Bat_v'] = df['Bat_dv'] / 10.0
    df['Fan_v'] = df['Fan_dv'] / 10.0
    df['MotL_v'] = df['PidLOut'] / 1000.0
    df['MotR_v'] = df['PidROut'] / 1000.0

    df['Markers'] = df['Markers'].fillna(0).astype(int)
    df['Left_Markers'] = (df['Markers'] // 128) % 128
    df['Right_Markers'] = (df['Markers'] // 32) % 4
    return df

# ==========================================
# 2. INITIALIZE GLOBAL DATA STORE
# ==========================================
DATA_STORE = {'runs': {}}
try:
    with open("recon.txt", "r") as f: recon_text = f.read()
    DATA_STORE['runs']['recon.txt'] = process_telemetry_df(parse_telemetry_text(recon_text))
except FileNotFoundError:
    print("ERROR: Could not find recon.txt.")
    DATA_STORE['runs']['recon.txt'] = pd.DataFrame()

def calc_marker_coords(row, side):
    x, y, theta = row['X_m'], row['Y_m'], row['Yaw_rad']
    offset = 0.04; length = 0.04
    angle_offset = np.pi/2 if side == 'left' else -np.pi/2
    perp_angle = theta + angle_offset
    return (x + offset * np.cos(perp_angle), y + offset * np.sin(perp_angle), 
            x + (offset+length) * np.cos(perp_angle), y + (offset+length) * np.sin(perp_angle))

# ==========================================
# 3. PHYSICS ENGINE (Optimization)
# ==========================================
def calculate_optimal_profile(df, mass, cof, fan_coeff, max_spd, margin_pct, max_accel, max_fan_v, fan_rate_limit, fan_coast_rate):
    if df is None or df.empty or len(df) < 5:
        return np.array([]), np.array([]), 0.0
    
    dist = df['Dist_mm'].values / 1000.0
    x, y = df['X_m'].values, df['Y_m'].values
    
    x_s = pd.Series(x).rolling(window=5, center=True).mean().bfill().ffill().values
    y_s = pd.Series(y).rolling(window=5, center=True).mean().bfill().ffill().values
    
    dx = np.gradient(x_s, dist)
    dy = np.gradient(y_s, dist)
    ddx = np.gradient(dx, dist)
    ddy = np.gradient(dy, dist)
    
    denom = (dx**2 + dy**2)**1.5
    denom[denom == 0] = 1e-6
    curvature = np.abs(dx * ddy - dy * ddx) / denom
    
    if len(curvature) > 15:
        curvature[-15:] = 0.0
        
    g = 9.81
    safety_factor = 1.0 + (margin_pct / 100.0)
    delta_dist = np.diff(dist)
    
    # --- PHASE 1: Calculate Raw Required Fan Voltage ---
    v_fan_req = np.zeros_like(dist)
    for i in range(len(dist)):
        k = curvature[i]
        F_c_target = mass * (max_spd**2) * k * safety_factor
        F_grip_static = cof * mass * g
        if F_c_target > F_grip_static:
            F_aero_needed = (F_c_target / cof) - (mass * g)
            v_fan_req[i] = np.clip(F_aero_needed / fan_coeff, 0, max_fan_v)
            
    # --- PHASE 2: Apply Fan Slew Rate Limiter (Anticipatory Spooling & Coasting) ---
    v_fan = np.copy(v_fan_req)
    
    # Backward pass: Spool up early for approaching corners
    for i in range(len(dist) - 2, -1, -1):
        max_dv_up = fan_rate_limit * (delta_dist[i] / max_spd) 
        if v_fan[i] < v_fan[i+1] - max_dv_up:
            v_fan[i] = v_fan[i+1] - max_dv_up

    # Forward pass: Physical limit to spool-up AND natural coast-down
    for i in range(1, len(dist)):
        max_dv_up = fan_rate_limit * (delta_dist[i-1] / max_spd)
        max_dv_down = fan_coast_rate * (delta_dist[i-1] / max_spd)
        
        if v_fan[i] > v_fan[i-1] + max_dv_up:
            v_fan[i] = v_fan[i-1] + max_dv_up
        elif v_fan[i] < v_fan[i-1] - max_dv_down:
            v_fan[i] = v_fan[i-1] - max_dv_down

    # --- PHASE 3: Recalculate Safe Velocity based on Achievable Fan Voltage ---
    v_opt = np.zeros_like(dist)
    for i in range(len(dist)):
        k = curvature[i]
        F_aero_actual = v_fan[i] * fan_coeff
        F_grip_total = cof * (mass * g + F_aero_actual)
        if k < 1e-5:
            v_opt[i] = max_spd
        else:
            v_max_c = np.sqrt(F_grip_total / (mass * k * safety_factor))
            v_opt[i] = min(max_spd, v_max_c)

    # --- PHASE 4: Apply Robot Acceleration/Braking Slew Rates ---
    for i in range(len(dist) - 1):
        max_v = np.sqrt(v_opt[i]**2 + 2 * max_accel * delta_dist[i])
        if v_opt[i+1] > max_v: v_opt[i+1] = max_v

    for i in range(len(dist) - 2, -1, -1):
        max_v = np.sqrt(v_opt[i+1]**2 + 2 * max_accel * delta_dist[i])
        if v_opt[i] > max_v: v_opt[i] = max_v

    v_safe = np.clip(v_opt[:-1], 0.1, None)
    dt = delta_dist / v_safe
    predicted_time = np.sum(dt)
    
    return v_opt, v_fan, predicted_time

# ==========================================
# 4. HELPER: GRAPH GENERATORS
# ==========================================
common_layout = dict(plot_bgcolor="#1E1E24", paper_bgcolor="#1E1E24", font=dict(color="white", family="Segoe UI, sans-serif"), margin=dict(l=40, r=20, t=80, b=40), hovermode="x unified", legend=dict(orientation="h", yanchor="bottom", y=1.02, xanchor="right", x=1))

def empty_fig():
    return go.Figure().update_layout(**common_layout)

def build_perf_graphs(df, dataset_id="recon"):
    cl = {**common_layout, 'uirevision': dataset_id}
    cl.update(xaxis=dict(title="Time (s)", showgrid=True, gridcolor="rgba(255,255,255,0.05)", zeroline=False), yaxis=dict(title="Velocity (m/s)", showgrid=True, gridcolor="rgba(255,255,255,0.05)", zeroline=False, autorange=True))
    if df is None or df.empty:
        emp = go.Figure().update_layout(**cl); return emp, emp, emp, emp, emp, emp, emp

    fig_left = go.Figure().add_trace(go.Scatter(x=df['Time_s'], y=df['PidLSet_mps'], name='Setpoint', line=dict(color="#FFE7EB", width=2, dash='dash'))).add_trace(go.Scatter(x=df['Time_s'], y=df['VelL_mps'], name='Reading', line=dict(color='#FF1C42', width=2))).update_layout(**cl)
    fig_right = go.Figure().add_trace(go.Scatter(x=df['Time_s'], y=df['PidRSet_mps'], name='Setpoint', line=dict(color="#C7EAFA", width=2, dash='dash'))).add_trace(go.Scatter(x=df['Time_s'], y=df['VelR_mps'], name='Reading', line=dict(color="#0EB3FF", width=2))).update_layout(**cl)
    fig_macro = go.Figure().add_trace(go.Scatter(x=df['Time_s'], y=df['Robot_Vel'], name='Robot Velocity', line=dict(color='#742BE2', width=2))).update_layout(**cl)
    fig_linepos = go.Figure().add_trace(go.Scatter(x=df['Time_s'], y=df['LinePos'], name='Line Position', line=dict(color='#FFD700', width=2))).update_layout(**cl).update_layout(yaxis=dict(title="Position (mm)"))
    fig_pidline = go.Figure().add_trace(go.Scatter(x=df['Time_s'], y=df['PidLine_mps'], name='PID Line Output', line=dict(color='#FF9F1C', width=2))).update_layout(**cl).update_layout(yaxis=dict(title="Rotational Vel Output (m/s)"))
    fig_battery = go.Figure().add_trace(go.Scatter(x=df['Time_s'], y=df['Bat_v'], name='Battery (V)', line=dict(color="#3AFF64", width=2))).add_trace(go.Scatter(x=df['Time_s'], y=df['Fan_v'], name='Fan (V)', line=dict(color="#FF3939", width=2))).update_layout(**cl).update_layout(yaxis=dict(title="Voltage (V)"))
    
    min_mot, max_mot = min(df['MotL_v'].min(), df['MotR_v'].min()), max(df['MotL_v'].max(), df['MotR_v'].max())
    fig_motor_out = go.Figure().add_trace(go.Scatter(x=df['Time_s'], y=df['MotL_v'], name='Left Motor (V)', line=dict(color='#FF1C42', width=2))).add_trace(go.Scatter(x=df['Time_s'], y=df['MotR_v'], name='Right Motor (V)', line=dict(color='#0EB3FF', width=2))).update_layout(**cl).update_layout(yaxis=dict(title="Output Voltage (V)", range=[min_mot - 1, max_mot + 1]))

    return fig_pidline, fig_left, fig_right, fig_macro, fig_motor_out, fig_battery, fig_linepos

def build_analysis_graphs(df_ref, df_act, kp, ki, kd, dataset_id):
    cl = {**common_layout, 'uirevision': dataset_id}
    fig_pid = go.Figure()
    if df_act is not None and not df_act.empty:
        dt = df_act['Time_s'].diff().replace(0, 0.001)
        dError = df_act['LinePos'].diff() / dt
        iError = (df_act['LinePos'] * dt).cumsum()
        sim_v = (kp * df_act['LinePos']) + (ki * iError.fillna(0)) + (kd * dError.fillna(0))
        fig_pid.add_trace(go.Scatter(x=df_act['Time_s'], y=sim_v, name='Simulated Demand (V)', line=dict(color='#FF9F1C', width=2)))
        fig_pid.add_trace(go.Scatter(x=df_act['Time_s'], y=[12]*len(df_act), name='+12V Limit', line=dict(color='red', width=1, dash='dash')))
        fig_pid.add_trace(go.Scatter(x=df_act['Time_s'], y=[-12]*len(df_act), name='-12V Limit', line=dict(color='red', width=1, dash='dash')))
        fig_pid.update_layout(**cl).update_layout(xaxis=dict(title="Time (s)", showgrid=True, gridcolor="rgba(255,255,255,0.05)", zeroline=False), yaxis=dict(title="Voltage (V)", showgrid=True, gridcolor="rgba(255,255,255,0.05)", zeroline=False))
    else: fig_pid.update_layout(**cl)
        
    fig_delta, fig_dev = go.Figure(), go.Figure()
    if df_act is not None and not df_act.empty and df_ref is not None and not df_ref.empty:
        common_dist = np.linspace(0, min(df_ref['Dist_mm'].max(), df_act['Dist_mm'].max()) / 1000.0, 500)
        t_r = np.interp(common_dist, df_ref['Dist_mm']/1000.0, df_ref['Time_s'])
        t_a = np.interp(common_dist, df_act['Dist_mm']/1000.0, df_act['Time_s'])
        fig_delta.add_trace(go.Scatter(x=common_dist, y=t_r - t_a, name='Time Delta', line=dict(color='#3AFF64', width=2), fill='tozeroy'))
        fig_delta.update_layout(**cl).update_layout(xaxis=dict(title="Distance (m)", showgrid=True, gridcolor="rgba(255,255,255,0.05)", zeroline=False), yaxis=dict(title="Delta (s) [Higher = Faster]", showgrid=True, gridcolor="rgba(255,255,255,0.05)", zeroline=False))
        
        x_r, y_r = np.interp(common_dist, df_ref['Dist_mm']/1000.0, df_ref['X_m']), np.interp(common_dist, df_ref['Dist_mm']/1000.0, df_ref['Y_m'])
        x_a, y_a = np.interp(common_dist, df_act['Dist_mm']/1000.0, df_act['X_m']), np.interp(common_dist, df_act['Dist_mm']/1000.0, df_act['Y_m'])
        fig_dev.add_trace(go.Scatter(x=common_dist, y=np.sqrt((x_a - x_r)**2 + (y_a - y_r)**2) * 1000.0, name='Deviation', line=dict(color='#FF2400', width=2), fill='tozeroy'))
        fig_dev.update_layout(**cl).update_layout(xaxis=dict(title="Distance (m)", showgrid=True, gridcolor="rgba(255,255,255,0.05)", zeroline=False), yaxis=dict(title="Deviation (mm)", showgrid=True, gridcolor="rgba(255,255,255,0.05)", zeroline=False))
    else:
        fig_delta.update_layout(**cl).update_layout(xaxis=dict(title="Distance (m)", showgrid=True, gridcolor="rgba(255,255,255,0.05)", zeroline=False), yaxis=dict(title="Delta (s)", showgrid=True, gridcolor="rgba(255,255,255,0.05)", zeroline=False))
        fig_dev.update_layout(**cl).update_layout(xaxis=dict(title="Distance (m)", showgrid=True, gridcolor="rgba(255,255,255,0.05)", zeroline=False), yaxis=dict(title="Deviation (mm)", showgrid=True, gridcolor="rgba(255,255,255,0.05)", zeroline=False))
        
    return fig_pid, fig_delta, fig_dev

# ==========================================
# 5. DASH APP LAYOUT
# ==========================================
app = dash.Dash(__name__)

stat_style_main = {'color': '#8A2BE2', 'fontFamily': 'Segoe UI', 'fontSize': '16px', 'fontWeight': 'bold', 'marginRight': '25px'}
stat_style_perf = {'color': "#8A2BE2", 'fontFamily': 'Segoe UI', 'fontSize': '14px', 'fontWeight': 'bold', 'marginTop': '4px'}
input_style = {'backgroundColor': '#2A2B36', 'color': 'white', 'border': '1px solid #8A2BE2', 'padding': '6px', 'borderRadius': '6px', 'width': '80px', 'fontFamily': 'Segoe UI', 'fontSize': '12px'}

# UI Helper for perfectly centered inputs
def opt_input(id_val, label, default, step):
    return html.Div(style={'display': 'flex', 'flexDirection': 'column', 'alignItems': 'center', 'gap': '5px'}, children=[
        html.Span(label, style={'color':'#A9A9A9', 'textAlign': 'center', 'fontSize': '12px'}),
        dcc.Input(id=id_val, type='number', value=default, step=step, style=input_style)
    ])

app.layout = html.Div(
    style={'backgroundColor': '#1E1E24', 'minHeight': '100vh', 'width': '100%', 'position': 'absolute', 'top': '0', 'left': '0', 'padding': '30px', 'boxSizing': 'border-box'}, 
    children=[
        dcc.Store(id='active-tab', data='perf'),
        
        # --- HEADER ROW & TABS ---
        html.Div(style={'marginBottom': '20px', 'display': 'flex', 'justifyContent': 'space-between', 'alignItems': 'flex-start'}, children=[
            html.Div(children=[
                html.H1("RAVEN Race Dashboard", style={'color': 'white', 'margin': '0', 'fontFamily': 'SAKURATA, sans-serif', 'fontWeight': 'normal'}),
                html.H3("EQUIPE PARALELA", style={'color': '#8A2BE2', 'margin': '5px 0 0 0', 'letterSpacing': '2px', 'fontFamily': 'SAKURATA, sans-serif'}),
                html.Div(id='global-stats', style={'display': 'flex', 'flexWrap': 'wrap', 'marginTop': '15px'})
            ]),
            html.Div(style={'display': 'flex', 'gap': '10px'}, children=[
                html.Button("RACE PERFORMANCE", id="btn-nav-perf", n_clicks=0),
                html.Button("LAP ANALYSIS", id="btn-nav-analysis", n_clicks=0),
                html.Button("RACE OPTIMIZATION", id="btn-nav-opt", n_clicks=0)
            ])
        ]),
        
        # --- SPLIT SCREEN CONTAINER ---
        html.Div(style={'display': 'flex', 'gap': '2%'}, children=[
            
            # ================= LEFT COLUMN =================
            html.Div(style={'width': '45%', 'position': 'sticky', 'top': '20px', 'height': 'calc(100vh - 40px)', 'overflowY': 'auto', 'display': 'flex', 'flexDirection': 'column', 'gap': '20px', 'paddingRight': '5px'}, children=[
                html.Div(id="graph-wrapper", style={'height': '500px', 'minHeight': '500px', 'width': '100%', 'position': 'relative', 'borderRadius': '12px', 'overflow': 'hidden', 'backgroundColor': '#1E1E24'}, children=[
                    html.Div(style={'position': 'absolute', 'top': '20px', 'left': '20px', 'zIndex': '10', 'display': 'flex', 'alignItems': 'center', 'gap': '15px'}, children=[
                        html.H3("TRACK MAP", style={'color': 'white', 'fontFamily': 'SAKURATA, sans-serif', 'margin': '0', 'fontSize': '22px'}),
                        html.Button("Marker Numbers", id="btn-numbers", n_clicks=0, style={'backgroundColor': 'transparent', 'color': '#666666', 'border': '1px solid #666666', 'padding': '6px 12px', 'borderRadius': '6px', 'cursor': 'pointer', 'fontFamily': 'Segoe UI', 'fontSize': '12px'})
                    ]),
                    html.Div(style={'position': 'absolute', 'top': '20px', 'right': '20px', 'zIndex': '10'}, children=[
                        html.Button("⛶ Fullscreen", id="btn-fullscreen", n_clicks=0, style={'backgroundColor': 'transparent', 'color': 'white', 'border': 'none', 'cursor': 'pointer', 'fontFamily': 'Segoe UI', 'fontSize': '14px', 'textDecoration': 'underline'})
                    ]),
                    dcc.Graph(id="track-map", style={'height': '100%', 'width': '100%'})
                ]),
                html.Div(style={'height': '350px', 'minHeight': '350px', 'width': '100%', 'position': 'relative', 'borderRadius': '12px', 'overflow': 'hidden', 'backgroundColor': '#1E1E24'}, children=[
                    html.Div(style={'position': 'absolute', 'top': '20px', 'left': '20px', 'zIndex': '10'}, children=[html.H3("LINE POSITION", style={'color': 'white', 'fontFamily': 'SAKURATA, sans-serif', 'margin': '0', 'fontSize': '22px'})]),
                    dcc.Graph(id="graph-linepos", figure=empty_fig(), style={'height': '100%', 'width': '100%'})
                ])
            ]),
            
            # ================= RIGHT COLUMN 1: PERFORMANCE =================
            html.Div(id="panel-performance", style={'width': '53%', 'display': 'flex', 'flexDirection': 'column', 'gap': '20px'}, children=[
                html.Div(style={'display': 'flex', 'gap': '15px', 'alignItems': 'center', 'backgroundColor': '#1E1E24', 'padding': '15px', 'borderRadius': '12px', 'border': '1px solid #3A3B46'}, children=[
                    dcc.Upload(id='upload-run', multiple=False, children=html.Button("📥 UPLOAD .TXT", style={'backgroundColor': '#8A2BE2', 'color': 'white', 'border': 'none', 'padding': '10px 18px', 'borderRadius': '6px', 'cursor': 'pointer', 'fontWeight': 'bold'})),
                    html.Div([html.Span("REF RUN:", style={'color': '#A9A9A9', 'fontSize': '12px', 'marginRight': '8px'}), dcc.Dropdown(id='dropdown-reference', options=[], clearable=False, style={'width': '150px'})], style={'display': 'flex', 'alignItems': 'center'}),
                    html.Div([html.Span("ACTIVE RUN:", style={'color': '#A9A9A9', 'fontSize': '12px', 'marginRight': '8px'}), dcc.Dropdown(id='dropdown-active', options=[], clearable=False, style={'width': '150px'})], style={'display': 'flex', 'alignItems': 'center'})
                ]),
                html.Div(children=[html.Div([html.H3("LINE PID OUTPUT", style={'color': 'white', 'margin': '0', 'fontFamily': 'SAKURATA, sans-serif', 'fontSize': '22px'})], style={'position': 'absolute', 'top': '20px', 'left': '20px', 'zIndex': '10'}), dcc.Graph(id="graph-pidline", figure=empty_fig())], style={'height': '400px', 'position': 'relative', 'backgroundColor': '#1E1E24', 'borderRadius': '12px'}),
                html.Div(children=[html.Div([html.H3("LEFT MOTOR PID", style={'color': 'white', 'margin': '0', 'fontFamily': 'SAKURATA, sans-serif', 'fontSize': '22px'}), html.Div(id="stat-rmse-l", style=stat_style_perf)], style={'position': 'absolute', 'top': '20px', 'left': '20px', 'zIndex': '10'}), dcc.Graph(id="graph-left", figure=empty_fig())], style={'height': '400px', 'position': 'relative', 'backgroundColor': '#1E1E24', 'borderRadius': '12px'}),
                html.Div(children=[html.Div([html.H3("RIGHT MOTOR PID", style={'color': 'white', 'margin': '0', 'fontFamily': 'SAKURATA, sans-serif', 'fontSize': '22px'}), html.Div(id="stat-rmse-r", style=stat_style_perf)], style={'position': 'absolute', 'top': '20px', 'left': '20px', 'zIndex': '10'}), dcc.Graph(id="graph-right", figure=empty_fig())], style={'height': '400px', 'position': 'relative', 'backgroundColor': '#1E1E24', 'borderRadius': '12px'}),
                html.Div(children=[html.Div([html.H3("ROBOT VELOCITY", style={'color': 'white', 'margin': '0', 'fontFamily': 'SAKURATA, sans-serif', 'fontSize': '22px'})], style={'position': 'absolute', 'top': '20px', 'left': '20px', 'zIndex': '10'}), dcc.Graph(id="graph-macro", figure=empty_fig())], style={'height': '400px', 'position': 'relative', 'backgroundColor': '#1E1E24', 'borderRadius': '12px'}),
                html.Div(children=[html.Div([html.H3("MOTOR VOLTAGE OUTPUT", style={'color': 'white', 'margin': '0', 'fontFamily': 'SAKURATA, sans-serif', 'fontSize': '22px'})], style={'position': 'absolute', 'top': '20px', 'left': '20px', 'zIndex': '10'}), dcc.Graph(id="graph-motor-out", figure=empty_fig())], style={'height': '400px', 'position': 'relative', 'backgroundColor': '#1E1E24', 'borderRadius': '12px'}),
                html.Div(children=[html.Div([html.H3("BATTERY AND FAN VOLTAGE", style={'color': 'white', 'margin': '0', 'fontFamily': 'SAKURATA, sans-serif', 'fontSize': '22px'})], style={'position': 'absolute', 'top': '20px', 'left': '20px', 'zIndex': '10'}), dcc.Graph(id="graph-battery", figure=empty_fig())], style={'height': '400px', 'position': 'relative', 'backgroundColor': '#1E1E24', 'borderRadius': '12px'})
            ]),

            # ================= RIGHT COLUMN 2: ANALYSIS =================
            html.Div(id="panel-analysis", style={'width': '53%', 'display': 'none', 'flexDirection': 'column', 'gap': '20px'}, children=[
                html.Div(style={'height': '450px', 'position': 'relative', 'borderRadius': '12px', 'backgroundColor': '#1E1E24', 'paddingTop':'10px'}, children=[
                    html.Div(style={'position': 'absolute', 'top': '10px', 'left': '20px', 'zIndex': '10', 'display':'flex', 'flexDirection':'column', 'gap':'5px'}, children=[
                        html.H3("PID SATURATION SIMULATOR", style={'color': 'white', 'margin': '0', 'fontFamily': 'SAKURATA, sans-serif', 'fontSize': '22px'}),
                        html.Div(style={'display': 'flex', 'gap': '15px'}, children=[
                            html.Div([html.Span("Kp ", style={'color':'#8A2BE2'}), dcc.Input(id='input-kp', type='number', value=0.64, step=0.01, style=input_style)]),
                            html.Div([html.Span("Ki ", style={'color':'#8A2BE2'}), dcc.Input(id='input-ki', type='number', value=0.0, step=0.01, style=input_style)]),
                            html.Div([html.Span("Kd ", style={'color':'#8A2BE2'}), dcc.Input(id='input-kd', type='number', value=0.0064, step=0.0001, style=input_style)]),
                        ])
                    ]),
                    dcc.Graph(id="graph-opt-pid", figure=empty_fig(), style={'height': '100%', 'marginTop':'30px'})
                ]),
                html.Div(children=[html.Div([html.H3("DELTA TIME (ACTIVE vs REF)", style={'color': 'white', 'margin': '0', 'fontFamily': 'SAKURATA, sans-serif', 'fontSize': '22px'})], style={'position': 'absolute', 'top': '20px', 'left': '20px', 'zIndex': '10'}), dcc.Graph(id="graph-opt-delta", figure=empty_fig())], style={'height': '400px', 'position': 'relative', 'backgroundColor': '#1E1E24', 'borderRadius': '12px'}),
                html.Div(children=[html.Div([html.H3("TRACK DEVIATION (WHEEL SLIP)", style={'color': 'white', 'margin': '0', 'fontFamily': 'SAKURATA, sans-serif', 'fontSize': '22px'})], style={'position': 'absolute', 'top': '20px', 'left': '20px', 'zIndex': '10'}), dcc.Graph(id="graph-opt-dev", figure=empty_fig())], style={'height': '400px', 'position': 'relative', 'backgroundColor': '#1E1E24', 'borderRadius': '12px'})
            ]),

            # ================= RIGHT COLUMN 3: OPTIMIZATION =================
            html.Div(id="panel-optimization", style={'width': '53%', 'display': 'none', 'flexDirection': 'column', 'gap': '20px'}, children=[
                html.Div(style={'backgroundColor': '#1E1E24', 'padding': '20px', 'borderRadius': '12px', 'display': 'flex', 'flexDirection': 'column', 'gap': '10px'}, children=[
                    html.H3("PHYSICS ENGINE SETUP", style={'color': 'white', 'margin': '0', 'fontFamily': 'SAKURATA, sans-serif'}),
                    html.Div(style={'display': 'flex', 'gap': '15px', 'flexWrap': 'wrap'}, children=[
                        opt_input('opt-mass', 'Mass (g)', 200.0, 10.0),
                        opt_input('opt-cof', 'Tire CoF (μ)', 1.2, 0.1),
                        opt_input('opt-fan', 'Fan (N/V)', 1.5, 0.1),
                        opt_input('opt-vel', 'Max Vel (m/s)', 4.0, 0.1),
                        opt_input('opt-accel', 'Max Accel (m/s²)', 8.0, 0.1),
                        opt_input('opt-max-fan', 'Max Fan (V)', 12.0, 0.1),
                        opt_input('opt-fan-rate', 'Fan Spool (V/s)', 24.0, 1.0),
                        opt_input('opt-fan-coast', 'Fan Coast (V/s)', 5.0, 1.0),
                        opt_input('opt-margin', 'Margin (%)', 15.0, 1.0)
                    ])
                ]),
                html.Div(children=[html.Div([html.H3("PREDICTED OPTIMAL SPEED", style={'color': 'white', 'margin': '0', 'fontFamily': 'SAKURATA, sans-serif', 'fontSize': '22px'})], style={'position': 'absolute', 'top': '20px', 'left': '20px', 'zIndex': '10'}), dcc.Graph(id="graph-opt-speed", figure=empty_fig())], style={'height': '400px', 'position': 'relative', 'backgroundColor': '#1E1E24', 'borderRadius': '12px'}),
                html.Div(children=[html.Div([html.H3("PREDICTED FAN VOLTAGE", style={'color': 'white', 'margin': '0', 'fontFamily': 'SAKURATA, sans-serif', 'fontSize': '22px'})], style={'position': 'absolute', 'top': '20px', 'left': '20px', 'zIndex': '10'}), dcc.Graph(id="graph-opt-fanvolt", figure=empty_fig())], style={'height': '400px', 'position': 'relative', 'backgroundColor': '#1E1E24', 'borderRadius': '12px'})
            ])
        ])
    ]
)

# ==========================================
# 6. TAB NAV & MASTER CALLBACK
# ==========================================
@app.callback(
    [Output("panel-performance", "style"), Output("panel-analysis", "style"), Output("panel-optimization", "style"),
     Output("btn-nav-perf", "style"), Output("btn-nav-analysis", "style"), Output("btn-nav-opt", "style"), Output("active-tab", "data")],
    [Input("btn-nav-perf", "n_clicks"), Input("btn-nav-analysis", "n_clicks"), Input("btn-nav-opt", "n_clicks")]
)
def switch_tabs(n_perf, n_analysis, n_opt):
    ctx = dash.callback_context
    triggered = ctx.triggered[0]['prop_id'].split('.')[0] if ctx.triggered else 'btn-nav-perf'
    ap = {'width': '53%', 'display': 'flex', 'flexDirection': 'column', 'gap': '20px'}
    hp = {'display': 'none'}
    ab = {'backgroundColor': '#8A2BE2', 'color': 'white', 'border': 'none', 'padding': '10px 20px', 'borderRadius': '8px', 'cursor': 'pointer', 'fontWeight': 'bold'}
    ib = {'backgroundColor': 'transparent', 'color': '#8A2BE2', 'border': '1px solid #8A2BE2', 'padding': '10px 20px', 'borderRadius': '8px', 'cursor': 'pointer', 'fontWeight': 'bold'}

    if triggered == 'btn-nav-opt': return hp, hp, ap, ib, ib, ab, 'opt'
    elif triggered == 'btn-nav-analysis': return hp, ap, hp, ib, ab, ib, 'ana'
    else: return ap, hp, hp, ab, ib, ib, 'perf'

@app.callback(
    [Output("track-map", "figure"), Output("graph-left", "figure"), Output("graph-right", "figure"), Output("graph-macro", "figure"), Output("graph-linepos", "figure"), Output("graph-pidline", "figure"), Output("graph-battery", "figure"), Output("graph-motor-out", "figure"),
     Output("graph-opt-pid", "figure"), Output("graph-opt-delta", "figure"), Output("graph-opt-dev", "figure"), Output("graph-opt-speed", "figure"), Output("graph-opt-fanvolt", "figure"),
     Output("dropdown-reference", "options"), Output("dropdown-reference", "value"), Output("dropdown-active", "options"), Output("dropdown-active", "value"),
     Output("global-stats", "children"), Output("stat-rmse-l", "children"), Output("stat-rmse-r", "children"),
     Output("graph-wrapper", "style"), Output("btn-numbers", "style"), Output("btn-fullscreen", "children")],
    [Input("active-tab", "data"), Input("upload-run", "contents"), State("upload-run", "filename"),
     Input("dropdown-reference", "value"), Input("dropdown-active", "value"),
     Input("input-kp", "value"), Input("input-ki", "value"), Input("input-kd", "value"),
     Input("opt-mass", "value"), Input("opt-cof", "value"), Input("opt-fan", "value"), Input("opt-vel", "value"), Input("opt-margin", "value"), Input("opt-accel", "value"), Input("opt-max-fan", "value"), Input("opt-fan-rate", "value"), Input("opt-fan-coast", "value"),
     Input("btn-numbers", "n_clicks"), Input("btn-fullscreen", "n_clicks"),
     Input("graph-left", "hoverData"), Input("graph-right", "hoverData"), Input("graph-macro", "hoverData"), Input("graph-linepos", "hoverData"), Input("graph-pidline", "hoverData"), Input("graph-battery", "hoverData"), Input("graph-motor-out", "hoverData"),
     Input("graph-opt-pid", "hoverData"), Input("graph-opt-delta", "hoverData"), Input("graph-opt-dev", "hoverData"), Input("graph-opt-speed", "hoverData"), Input("graph-opt-fanvolt", "hoverData")]
)
def master_controller(active_tab, upload_contents, upload_filename, ref_val, act_val, kp, ki, kd, m_mass, m_cof, m_fan, m_vel, m_marg, m_accel, m_max_fan, m_fan_rate, m_fan_coast, clicks_num, clicks_fs, hl, hr, hm, hlp, hp, hb, hmo, hop, hod, hodev, hospd, hofv):
    ctx = dash.callback_context
    trigger_id = ctx.triggered[0]['prop_id'].split('.')[0] if ctx.triggered else ""
    is_hover = trigger_id.startswith("graph-")
    
    if not ref_val and 'recon.txt' in DATA_STORE['runs']: ref_val = 'recon.txt'
    if not act_val and 'recon.txt' in DATA_STORE['runs']: act_val = 'recon.txt'
    
    if trigger_id == "upload-run" and upload_contents:
        _, content_string = upload_contents.split(',')
        DATA_STORE['runs'][upload_filename] = process_telemetry_df(parse_telemetry_text(base64.b64decode(content_string).decode('utf-8')))
        act_val = upload_filename
        if not ref_val: ref_val = upload_filename

    run_options = [{'label': k, 'value': k} for k in DATA_STORE['runs'].keys()]
    df_ref = DATA_STORE['runs'].get(ref_val, pd.DataFrame())
    df_act = DATA_STORE['runs'].get(act_val, pd.DataFrame())

    # --- FAST HOVER SYNC ---
    if is_hover:
        h_data = {"graph-left": hl, "graph-right": hr, "graph-macro": hm, "graph-linepos": hlp, "graph-pidline": hp, "graph-battery": hb, "graph-motor-out": hmo, "graph-opt-pid": hop, "graph-opt-delta": hod, "graph-opt-dev": hodev, "graph-opt-speed": hospd, "graph-opt-fanvolt": hofv}.get(trigger_id)
        hover_val = h_data['points'][0]['x'] if h_data and 'points' in h_data and len(h_data['points'])>0 else None
        hover_time, hover_dist = None, None

        if hover_val is not None and not df_act.empty:
            if trigger_id in ["graph-opt-delta", "graph-opt-dev", "graph-opt-speed", "graph-opt-fanvolt"]:
                hover_dist = hover_val
                idx = (df_act['Dist_mm']/1000.0 - hover_dist).abs().idxmin()
                hover_time = df_act['Time_s'].iloc[idx]
            else:
                hover_time = hover_val
                idx = (df_act['Time_s'] - hover_time).abs().idxmin()
                hover_dist = df_act['Dist_mm'].iloc[idx] / 1000.0

        def get_patch(val):
            p = dash.Patch(); p['layout']['shapes'] = [dict(type="line", x0=val, x1=val, y0=0, y1=1, yref="paper", line=dict(color="rgba(255,255,255,0.7)", width=2, dash="dot"))] if val else []; return p

        p_map = dash.Patch()
        if hover_time is not None and not df_act.empty:
            idx = (df_act['Time_s'] - hover_time).abs().idxmin()
            # CRITICAL FIX: Using fixed trace index 5 for the Robot Dot
            p_map['data'][5]['x'] = [df_act['X_m'].iloc[idx]]
            p_map['data'][5]['y'] = [df_act['Y_m'].iloc[idx]]

        return (p_map, get_patch(hover_time), get_patch(hover_time), get_patch(hover_time), get_patch(hover_time), get_patch(hover_time), get_patch(hover_time), get_patch(hover_time), 
                get_patch(hover_time), get_patch(hover_dist), get_patch(hover_dist), get_patch(hover_dist), get_patch(hover_dist), 
                dash.no_update, dash.no_update, dash.no_update, dash.no_update, dash.no_update, dash.no_update, dash.no_update, dash.no_update, dash.no_update, dash.no_update)

    # --- FULL REDRAW ---
    show_numbers = ((clicks_num or 0) % 2 == 1)
    is_fs = ((clicks_fs or 0) % 2 == 1)
    wrap_style = {'position':'fixed','top':'0','left':'0','width':'100vw','height':'100vh','zIndex':'9999','backgroundColor':'#1E1E24'} if is_fs else {'height':'500px','minHeight':'500px','width':'100%','position':'relative','borderRadius':'12px','overflow':'hidden','backgroundColor':'#1E1E24'}
    fs_text = "✖ Exit Fullscreen" if is_fs else "⛶ Fullscreen"
    bn_style = {'backgroundColor':'transparent','color':'#8A2BE2' if show_numbers else '#666666','border':'1px solid #8A2BE2' if show_numbers else '1px solid #666666','padding':'6px 12px','borderRadius':'6px','cursor':'pointer'}

    figs_perf = build_perf_graphs(df_act, act_val)
    figs_opt = build_analysis_graphs(df_ref, df_act, kp or 0, ki or 0, kd or 0, act_val)
    
    v_opt, v_fan, pred_time = calculate_optimal_profile(
        df_act, 
        float(m_mass or 200.0) / 1000.0, 
        float(m_cof or 1.2), 
        float(m_fan or 1.5), 
        float(m_vel or 4.0), 
        float(m_marg or 15.0), 
        float(m_accel or 8.0), 
        float(m_max_fan or 12.0),
        float(m_fan_rate or 24.0),
        float(m_fan_coast or 5.0)
    )
    
    fig_speed, fig_fanvolt = go.Figure().update_layout(**common_layout), go.Figure().update_layout(**common_layout)
    if len(v_opt) > 0:
        dist_m = df_act['Dist_mm']/1000.0
        fig_speed.add_trace(go.Scatter(x=dist_m, y=v_opt, name='Optimal Target', line=dict(color='#8A2BE2', width=3)))
        fig_speed.add_trace(go.Scatter(x=dist_m, y=df_act['Robot_Vel'], name='Actual (Active)', line=dict(color='white', width=1, dash='solid')))
        fig_speed.update_layout(xaxis=dict(title="Distance (m)", showgrid=True, gridcolor="rgba(255,255,255,0.05)", zeroline=False), yaxis=dict(title="Speed (m/s)", showgrid=True, gridcolor="rgba(255,255,255,0.05)", zeroline=False))
        
        fan_limit = float(m_max_fan or 12.0)
        fig_fanvolt.add_trace(go.Scatter(x=dist_m, y=v_fan, name='Required Fan Voltage', line=dict(color='#FF1C42', width=3)))
        fig_fanvolt.add_trace(go.Scatter(x=dist_m, y=[fan_limit]*len(dist_m), name=f'{fan_limit}V Limit', line=dict(color='red', width=1, dash='dash')))
        fig_fanvolt.update_layout(xaxis=dict(title="Distance (m)", showgrid=True, gridcolor="rgba(255,255,255,0.05)", zeroline=False), yaxis=dict(title="Fan Voltage (V)", showgrid=True, gridcolor="rgba(255,255,255,0.05)", zeroline=False))

    # Global Stats
    stat_html, r_l, r_r = [], 0, 0
    if not df_act.empty:
        t_time, t_dist = df_act['Time_s'].max(), (df_act['Dist_mm'].max() - df_act['Dist_mm'].min()) / 1000.0
        stat_html.extend([html.Div(f"⏱ Time: {t_time:.2f} s", style=stat_style_main), html.Div(f"📏 Dist: {t_dist:.2f} m", style=stat_style_main), html.Div(f"⚡ Avg Spd: {(t_dist/t_time if t_time>0 else 0):.2f} m/s", style=stat_style_main)])
        if active_tab == 'opt' and len(v_opt) > 0: stat_html.append(html.Div(f"🚀 PREDICTED OPTIMAL: {pred_time:.2f} s", style={**stat_style_main, 'color': '#3AFF64'}))
        r_l, r_r = np.sqrt(((df_act['VelL_mps']-df_act['PidLSet_mps'])**2).mean()), np.sqrt(((df_act['VelR_mps']-df_act['PidRSet_mps'])**2).mean())

    fig_map = go.Figure()
    x_pad = (df_ref['X_m'].max() - df_ref['X_m'].min()) * 0.05 if not df_ref.empty else 0
    y_pad = (df_ref['Y_m'].max() - df_ref['Y_m'].min()) * 0.05 if not df_ref.empty else 0
    x_r = [df_ref['X_m'].min() - x_pad, df_ref['X_m'].max() + x_pad] if not df_ref.empty else [-1, 1]
    y_r = [df_ref['Y_m'].min() - y_pad, df_ref['Y_m'].max() + y_pad] if not df_ref.empty else [-1, 1]

    # Trace 0: Ref Path
    fig_map.add_trace(go.Scatter(x=df_ref['X_m'] if not df_ref.empty else [], y=df_ref['Y_m'] if not df_ref.empty else [], mode='lines', line=dict(color='white', width=4), visible=(active_tab != 'opt')))

    # Traces 1 & 2: Markers
    lx, ly, rx, ry = [], [], [], []
    if not df_ref.empty:
        left_e = df_ref[df_ref['Left_Markers'].diff() > 0]
        right_e = df_ref[df_ref['Right_Markers'].diff() > 0]
        for _, r in left_e.iterrows(): sx, sy, ex, ey = calc_marker_coords(r, 'left'); lx.extend([sx, ex, None]); ly.extend([sy, ey, None])
        for _, r in right_e.iterrows(): sx, sy, ex, ey = calc_marker_coords(r, 'right'); rx.extend([sx, ex, None]); ry.extend([sy, ey, None])
    fig_map.add_trace(go.Scatter(x=lx, y=ly, mode='lines', line=dict(color='white', width=4), visible=(active_tab != 'opt')))
    fig_map.add_trace(go.Scatter(x=rx, y=ry, mode='lines', line=dict(color='white', width=4), visible=(active_tab != 'opt')))
    
    # Trace 3: Active Path Overlay
    show_active = (active_tab != 'opt') and (act_val != ref_val)
    hx, hy = (df_act['X_m'], df_act['Y_m']) if not df_act.empty else ([], [])
    fig_map.add_trace(go.Scatter(x=hx, y=hy, mode='lines', line=dict(color='#FF2400', width=2), visible=show_active))

    # Trace 4: Opt Gradient Path
    show_opt = (active_tab == 'opt') and not df_act.empty and len(v_opt) > 0
    user_max_vel = float(m_vel or 4.0)
    
    if show_opt:
        valid_speeds = v_opt[v_opt > 0.5]
        cmin_val = np.percentile(valid_speeds, 5) if len(valid_speeds) > 0 else 0
        cmax_val = np.max(v_opt) if len(v_opt) > 0 else 4.0
    else:
        cmin_val, cmax_val = 0, 4.0

    f1_colorscale = [
        [0.0, "#C60CFF"],
        [0.25, "#60CDFF"],
        [0.5, "#56FF56"],
        [0.75, "#FFF021"],
        [1.0, "#FD185D"]
    ]

    fig_map.add_trace(go.Scatter(
        x=df_act['X_m'] if show_opt else [], y=df_act['Y_m'] if show_opt else [], mode='markers',
        marker=dict(
            size=5, 
            symbol='circle',
            line=dict(width=0), 
            color=v_opt if show_opt else [], 
            cmin=cmin_val,
            cmax=cmax_val,
            colorscale=f1_colorscale, 
            showscale=True, 
            colorbar=dict(title="Target Vel (m/s)")
        ),
        visible=show_opt
    ))

    # Trace 5: Robot Dot (Always at index 5)
    dot_x, dot_y = (df_act['X_m'].iloc[0], df_act['Y_m'].iloc[0]) if not df_act.empty else (0,0)
    fig_map.add_trace(go.Scatter(x=[dot_x], y=[dot_y], mode='markers', marker=dict(size=14, color='#8A2BE2', symbol='circle', line=dict(width=2, color='white'))))

    annotations = []
    if show_numbers and active_tab != 'opt':
        left_e = df_ref[df_ref['Left_Markers'].diff() > 0] if not df_ref.empty else []
        for _, r in left_e.iterrows():
            _, _, ex, ey = calc_marker_coords(r, 'left')
            annotations.append(dict(x=ex, y=ey, text=str(int(r['Left_Markers'])), showarrow=False, xshift=12*np.cos(r['Yaw_rad']+np.pi/2), yshift=12*np.sin(r['Yaw_rad']+np.pi/2), font=dict(color='white', size=11)))

    fig_map.update_layout(uirevision='fixed', plot_bgcolor="#1E1E24", paper_bgcolor="#1E1E24", xaxis=dict(range=x_r, showgrid=False, zeroline=False, showticklabels=False), yaxis=dict(range=y_r, showgrid=False, zeroline=False, showticklabels=False, scaleanchor="x"), margin=dict(l=20, r=20, t=100, b=20), showlegend=False, annotations=annotations)

    return (fig_map, figs_perf[1], figs_perf[2], figs_perf[3], figs_perf[6], figs_perf[0], figs_perf[5], figs_perf[4],
            figs_opt[0], figs_opt[1], figs_opt[2], fig_speed, fig_fanvolt,
            run_options, ref_val, run_options, act_val, stat_html, f"RMSE: {r_l:.3f} m/s", f"RMSE: {r_r:.3f} m/s",
            wrap_style, bn_style, fs_text)

if __name__ == '__main__':
    app.run(debug=True)