import dash
from dash import dcc, html
from dash.dependencies import Input, Output
import plotly.graph_objects as go
from plotly.subplots import make_subplots
import pandas as pd
import numpy as np
import io

# ==========================================
# VISUAL IDENTITY
# ==========================================
BG_COLOR = "#1E1E24"      
PANEL_COLOR = "#282931"   
ACCENT_COLOR = "#8A2BE2"  

READING_0_COLOR = "#FF1C42" 
OUTPUT_0_COLOR = "#2EFFD2"  
ERROR_0_COLOR = "#FFD700"   

READING_1_COLOR = "#742BB8" 
OUTPUT_1_COLOR = "#71F160"  
ERROR_1_COLOR = "#FF4D4D"   

SETPOINT_COLOR = "#FDF5FF" 
GRID_COLOR = "#3A3B46"      
FONT_FAMILY = "Segoe UI, sans-serif"

panel_style = {
    'backgroundColor': PANEL_COLOR,
    'borderRadius': '12px',
    'padding': '20px',
    'boxShadow': '0px 4px 15px rgba(0, 0, 0, 0.4)',
    'marginBottom': '20px'
}

# ==========================================
# 1. CUSTOM LOG PARSER
# ==========================================
def load_telemetry_log(filepath):
    clean_csv_lines = []
    headers = "Time_ms,Dist_mm,X_mm,Y_mm,Yaw_mrad,VelL_mmps,VelR_mmps,LinePos,PidLine,PidLSet,PidRSet,PidLOut,PidROut,Bat_dv,Markers,Fan_dv"
    clean_csv_lines.append(headers)
    
    with open(filepath, "r") as f:
        for line in f:
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

# ==========================================
# 2. DATA PROCESSING & MINING
# ==========================================
df = load_telemetry_log("telemetry.txt")

df['Time_s'] = df['Time_ms'] / 1000.0
df['X_m'] = df['X_mm'] / 1000.0
df['Y_m'] = df['Y_mm'] / 1000.0
df['Yaw_rad'] = df['Yaw_mrad'] / 1000.0
df['VelL_mps'] = df['VelL_mmps'] / 1000.0
df['VelR_mps'] = df['VelR_mmps'] / 1000.0
df['Robot_Vel'] = (df['VelL_mps'] + df['VelR_mps']) / 2.0
df['Bat_V'] = df['Bat_dv'] / 10.0
df['Fan_V'] = df['Fan_dv'] / 10.0

df['Markers'] = df['Markers'].fillna(0).astype(int)
df['Left_Markers']  = (df['Markers'] // 128) % 128    
df['Right_Markers'] = (df['Markers'] // 32) % 4       

dt = df['Time_s'].diff().fillna(0.01)
df['Long_Accel_G'] = (df['Robot_Vel'].diff() / dt) / 9.81
df['Long_Accel_G'] = df['Long_Accel_G'].rolling(window=5, center=True).mean().fillna(0)

df['Yaw_Rate'] = df['Yaw_rad'].diff() / dt
df['Lat_Accel_G'] = (df['Robot_Vel'] * df['Yaw_Rate']) / 9.81
df['Lat_Accel_G'] = df['Lat_Accel_G'].rolling(window=5, center=True).mean().fillna(0)


# --- IDEAL RACING LINE ALGORITHM (WITH ACTIVE AERO) ---

# Constraints
V_MAX = 4.0          # Max Top Speed (m/s)
A_LON_MAX = 8.0      # Max Acceleration (m/s^2)
D_BRAKE_MAX = 8.0    # Max Braking (m/s^2)

# Suction Fan Aerodynamic Model Parameters (Estimated from user anecdotes)
A_LAT_BASE = 5.0     # Baseline grip without fan (approx 0.5G)
FAN_MAX_V = 12.0     # Max fan voltage
FAN_K = 0.55         # Aerodynamic constant (Downforce = K * V^2)

# Calculate Track Curvature (Radius = v / yaw_rate)
smoothed_yaw_rate = df['Yaw_Rate'].abs().rolling(window=5, center=True).mean().fillna(0.001)
smoothed_yaw_rate = np.maximum(smoothed_yaw_rate, 0.001)
track_radius = np.clip(df['Robot_Vel'] / smoothed_yaw_rate, 0.01, 100)

# Calculate max possible cornering speed assuming FAN IS AT 100% MAX VOLTAGE
a_lat_max_with_fan = A_LAT_BASE + (FAN_K * (FAN_MAX_V**2))
v_corner = np.sqrt(a_lat_max_with_fan * track_radius)
v_ideal = np.clip(v_corner, 0, V_MAX).values

ds = (dt * df['Robot_Vel']).values 

# Forward Pass (Acceleration Limits)
for i in range(1, len(v_ideal)):
    v_possible = np.sqrt(v_ideal[i-1]**2 + 2 * A_LON_MAX * ds[i])
    if v_possible < v_ideal[i]:
        v_ideal[i] = v_possible

# Backward Pass (Braking Limits)
for i in range(len(v_ideal)-2, -1, -1):
    v_possible = np.sqrt(v_ideal[i+1]**2 + 2 * D_BRAKE_MAX * ds[i+1])
    if v_possible < v_ideal[i]:
        v_ideal[i] = v_possible

df['Ideal_Vel'] = v_ideal

# Calculate IDEAL FAN VOLTAGE needed to sustain the calculated Ideal Speed
a_lat_required = (df['Ideal_Vel']**2) / track_radius
# If required grip is more than baseline, turn on fan. Otherwise, 0V.
ideal_fan_v2 = (a_lat_required - A_LAT_BASE) / FAN_K
df['Ideal_Fan_V'] = np.sqrt(np.clip(ideal_fan_v2, 0, FAN_MAX_V**2))


# ==========================================
# 3. MARKER PROJECTION LOGIC
# ==========================================
left_marker_shapes = []
right_marker_shapes = []

left_marker_events = df[df['Left_Markers'].diff() > 0]
right_marker_events = df[df['Right_Markers'].diff() > 0]

def calc_marker_coords(row, side):
    x, y, theta = row['X_m'], row['Y_m'], row['Yaw_rad']
    offset = length = 0.04
    angle_offset = np.pi/2 if side == 'left' else -np.pi/2
    perp_angle = theta + angle_offset
    
    start_x = x + offset * np.cos(perp_angle)
    start_y = y + offset * np.sin(perp_angle)
    end_x = start_x + length * np.cos(perp_angle)
    end_y = start_y + length * np.sin(perp_angle)
    
    return start_x, start_y, end_x, end_y

for _, row in left_marker_events.iterrows():
    sx, sy, ex, ey = calc_marker_coords(row, 'left')
    left_marker_shapes.append(go.Scatter(x=[sx, ex], y=[sy, ey], mode='lines', line=dict(color=OUTPUT_0_COLOR, width=4), hoverinfo='skip', showlegend=False))

for _, row in right_marker_events.iterrows():
    sx, sy, ex, ey = calc_marker_coords(row, 'right')
    right_marker_shapes.append(go.Scatter(x=[sx, ex], y=[sy, ey], mode='lines', line=dict(color=READING_1_COLOR, width=4), hoverinfo='skip', showlegend=False))


# ==========================================
# 4. BUILD THE DASHBOARD VISUALS
# ==========================================
app = dash.Dash(__name__)
app.title = "Raven Telemetry Center"

layout_template = go.layout.Template(
    layout=dict(
        plot_bgcolor=PANEL_COLOR, paper_bgcolor=PANEL_COLOR, 
        font=dict(color=SETPOINT_COLOR, family=FONT_FAMILY),
        xaxis=dict(showgrid=True, gridcolor=GRID_COLOR, zeroline=False, gridwidth=1),
        yaxis=dict(showgrid=True, gridcolor=GRID_COLOR, zeroline=False, gridwidth=1),
        margin=dict(l=30, r=30, t=50, b=30),
        hovermode="x unified",
        hoverlabel=dict(bgcolor=PANEL_COLOR, font_family=FONT_FAMILY)
    )
)

# --- MAP 1: REFERENCE TRACK ---
fig_ref_map = go.Figure()
fig_ref_map.add_trace(go.Scatter(
    x=df['X_m'], y=df['Y_m'], mode='lines', line=dict(width=2, color='white'), name="Reference Track"
))
for shape in left_marker_shapes: fig_ref_map.add_trace(shape)
for shape in right_marker_shapes: fig_ref_map.add_trace(shape)
fig_ref_map.add_trace(go.Scatter(x=[None], y=[None], mode='lines', line=dict(color=OUTPUT_0_COLOR, width=4), name='Track Markers'))
fig_ref_map.add_trace(go.Scatter(x=[df['X_m'].iloc[0]], y=[df['Y_m'].iloc[0]], mode='markers', marker=dict(size=14, color=SETPOINT_COLOR, symbol='cross'), name='Robot'))
fig_ref_map.update_layout(template=layout_template, title=dict(text="REFERENCE MAP", font=dict(size=16, color=ACCENT_COLOR)), yaxis_scaleanchor="x", showlegend=True) 

# --- MAP 2: SPEED PROFILE ---
fig_speed_map = go.Figure()
fig_speed_map.add_trace(go.Scatter(
    x=df['X_m'], y=df['Y_m'], mode='markers+lines',
    marker=dict(size=4, color=df['Ideal_Vel'], colorscale='Plasma', showscale=True, colorbar=dict(title="Ideal (m/s)")), 
    line=dict(width=1, color='rgba(255,255,255,0.1)'), name="Theoretical Velocity"
))
fig_speed_map.add_trace(go.Scatter(x=[df['X_m'].iloc[0]], y=[df['Y_m'].iloc[0]], mode='markers', marker=dict(size=14, color=SETPOINT_COLOR, symbol='cross'), name='Robot'))
fig_speed_map.update_layout(template=layout_template, title=dict(text="IDEAL SPEED MAP", font=dict(size=16, color=ACCENT_COLOR)), yaxis_scaleanchor="x", showlegend=False) 

# --- FRICTION CIRCLE ---
fig_gg = go.Figure()
fig_gg.add_trace(go.Scatter(x=df['Lat_Accel_G'], y=df['Long_Accel_G'], mode='markers', marker=dict(size=4, color=ERROR_1_COLOR, opacity=0.5), name="G-Envelope"))
fig_gg.add_trace(go.Scatter(x=[0], y=[0], mode='markers', marker=dict(size=14, color=SETPOINT_COLOR), name='Current G'))
fig_gg.add_shape(type="circle", x0=-1, y0=-1, x1=1, y1=1, line_color=GRID_COLOR)
fig_gg.add_shape(type="circle", x0=-2, y0=-2, x1=2, y1=2, line_color=GRID_COLOR)
fig_gg.update_layout(template=layout_template, title=dict(text="G-G DIAGRAM", font=dict(size=16, color=ACCENT_COLOR)), xaxis_title="Lat (G)", yaxis_title="Long (G)", xaxis_range=[-3,3], yaxis_range=[-3,3])

# --- TIME SERIES ---
fig_time = make_subplots(rows=5, cols=1, shared_xaxes=True, vertical_spacing=0.04, subplot_titles=("Velocity (m/s)", "Line Position & PID", "Motor Setpoints", "Motor Outputs", "Vitals & Aerodynamics"))

fig_time.add_trace(go.Scatter(x=df['Time_s'], y=df['VelL_mps'], name='Vel L', line=dict(color=READING_0_COLOR, width=2)), row=1, col=1)
fig_time.add_trace(go.Scatter(x=df['Time_s'], y=df['VelR_mps'], name='Vel R', line=dict(color=OUTPUT_1_COLOR, width=2)), row=1, col=1)
fig_time.add_trace(go.Scatter(x=df['Time_s'], y=df['Robot_Vel'], name='Actual Vel', line=dict(color='white', width=2)), row=1, col=1)
fig_time.add_trace(go.Scatter(x=df['Time_s'], y=df['Ideal_Vel'], name='Ideal Limit', line=dict(color=ERROR_0_COLOR, dash='dash', width=2)), row=1, col=1)

fig_time.add_trace(go.Scatter(x=df['Time_s'], y=df['LinePos'], name='Line Pos', line=dict(color=ERROR_0_COLOR)), row=2, col=1)
fig_time.add_trace(go.Scatter(x=df['Time_s'], y=df['PidLine'], name='PID Line', line=dict(color=ACCENT_COLOR)), row=2, col=1)

fig_time.add_trace(go.Scatter(x=df['Time_s'], y=df['PidLSet'], name='Set L', line=dict(color=OUTPUT_0_COLOR)), row=3, col=1)
fig_time.add_trace(go.Scatter(x=df['Time_s'], y=df['PidRSet'], name='Set R', line=dict(color=READING_1_COLOR)), row=3, col=1)

fig_time.add_trace(go.Scatter(x=df['Time_s'], y=df['PidLOut'], name='Out L', line=dict(color=READING_0_COLOR)), row=4, col=1)
fig_time.add_trace(go.Scatter(x=df['Time_s'], y=df['PidROut'], name='Out R', line=dict(color=OUTPUT_1_COLOR)), row=4, col=1)

fig_time.add_trace(go.Scatter(x=df['Time_s'], y=df['Bat_V'], name='Bat (V)', line=dict(color=OUTPUT_1_COLOR)), row=5, col=1)
fig_time.add_trace(go.Scatter(x=df['Time_s'], y=df['Fan_V'], name='Actual Fan (V)', line=dict(color=OUTPUT_0_COLOR)), row=5, col=1)

# NEW: The Ideal Fan Voltage
fig_time.add_trace(go.Scatter(x=df['Time_s'], y=df['Ideal_Fan_V'], name='Ideal Fan (V)', line=dict(color=ERROR_0_COLOR, dash='dash')), row=5, col=1)

fig_time.update_layout(template=layout_template, height=1000)

for annotation in fig_time['layout']['annotations']:
    annotation['font'] = dict(size=14, color=SETPOINT_COLOR)

# ==========================================
# APP LAYOUT (WIDENED TO FULL SCREEN)
# ==========================================
app.layout = html.Div(style={'backgroundColor': BG_COLOR, 'minHeight': '100vh', 'padding': '20px', 'fontFamily': FONT_FAMILY}, children=[
    
    html.H1("RAVEN TELEMETRY", style={'color': SETPOINT_COLOR, 'textAlign': 'left', 'fontWeight': 'bold', 'letterSpacing': '2px', 'marginBottom': '20px', 'marginLeft': '10px'}),
    
    # ROW 1: The Maps (Taking up 100% of the screen width)
    html.Div([
        html.Div([dcc.Graph(id='ref-map', figure=fig_ref_map, style={'height': '600px'})], style={'width': '49%', 'display': 'inline-block'}),
        html.Div([dcc.Graph(id='speed-map', figure=fig_speed_map, style={'height': '600px'})], style={'width': '49%', 'display': 'inline-block', 'float': 'right'}),
    ], style={**panel_style, 'width': '100%', 'display': 'block', 'boxSizing': 'border-box'}),
    
    # ROW 2: Time Series and GG Diagram
    html.Div([
        html.Div([dcc.Graph(id='time-series', figure=fig_time)], style={**panel_style, 'width': '68%', 'display': 'inline-block', 'verticalAlign': 'top', 'marginRight': '2%', 'boxSizing': 'border-box'}),
        html.Div([dcc.Graph(id='gg-diagram', figure=fig_gg, style={'height': '1000px'})], style={**panel_style, 'width': '30%', 'display': 'inline-block', 'verticalAlign': 'top', 'boxSizing': 'border-box'})
    ], style={'width': '100%', 'display': 'block'})
])

# ==========================================
# 5. CALLBACKS
# ==========================================
@app.callback(
    [Output('ref-map', 'figure'), Output('speed-map', 'figure'), Output('gg-diagram', 'figure')],
    [Input('time-series', 'hoverData')]
)
def update_map_position(hoverData):
    patched_ref = go.Figure(fig_ref_map)
    patched_spd = go.Figure(fig_speed_map)
    patched_gg = go.Figure(fig_gg)
    
    if hoverData:
        hover_time = hoverData['points'][0]['x']
        idx = (df['Time_s'] - hover_time).abs().idxmin()
        
        patched_ref.data[-1].x = [df['X_m'].iloc[idx]]
        patched_ref.data[-1].y = [df['Y_m'].iloc[idx]]
        
        patched_spd.data[-1].x = [df['X_m'].iloc[idx]]
        patched_spd.data[-1].y = [df['Y_m'].iloc[idx]]
        
        patched_gg.data[1].x = [df['Lat_Accel_G'].iloc[idx]]
        patched_gg.data[1].y = [df['Long_Accel_G'].iloc[idx]]
        
    return patched_ref, patched_spd, patched_gg

if __name__ == '__main__':
    app.run(debug=True)