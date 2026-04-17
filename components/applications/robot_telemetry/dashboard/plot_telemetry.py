import dash
from dash import dcc, html
from dash.dependencies import Input, Output
import plotly.graph_objects as go
from plotly.subplots import make_subplots
import pandas as pd
import numpy as np
import io

# ==========================================
# 1. CUSTOM LOG PARSER
# ==========================================

def load_telemetry_log(filepath):
    clean_csv_lines = []
    
    # Define our strict expected headers (fixing the truncated 'F' to 'Fan_dv')
    headers = "Time_ms,Dist_mm,X_mm,Y_mm,Yaw_mrad,VelL_mmps,VelR_mmps,LinePos,PidLine,PidLSet,PidRSet,PidLOut,PidROut,Bat_dv,Markers,Fan_dv"
    clean_csv_lines.append(headers)
    
    with open(filepath, "r") as f:
        for line in f:
            # Only process lines from the Telemetry tag
            if "[TLM]" in line:
                # Ignore the START/END headers and the raw CSV header line
                if "TELEMETRY START" in line or "TELEMETRY END" in line or "Time_ms" in line:
                    continue
                
                # Split at "[TLM] " to isolate the CSV payload
                parts = line.split("[TLM] ")
                if len(parts) > 1:
                    csv_payload = parts[1].strip()
                    # Basic validation to ensure it's a data row
                    if "," in csv_payload:
                        clean_csv_lines.append(csv_payload)
                        
    # Convert the cleaned lines into a pandas dataframe
    csv_text = "\n".join(clean_csv_lines)
    return pd.read_csv(io.StringIO(csv_text))

# ==========================================
# 2. DATA PROCESSING & MINING
# ==========================================

# Load the raw text log directly
df = load_telemetry_log("telemetry.txt")

# Convert Units to standard SI
df['Time_s'] = df['Time_ms'] / 1000.0
df['X_m'] = df['X_mm'] / 1000.0
df['Y_m'] = df['Y_mm'] / 1000.0
df['Yaw_rad'] = df['Yaw_mrad'] / 1000.0
df['VelL_mps'] = df['VelL_mmps'] / 1000.0
df['VelR_mps'] = df['VelR_mmps'] / 1000.0
df['Robot_Vel'] = (df['VelL_mps'] + df['VelR_mps']) / 2.0
df['Bat_V'] = df['Bat_dv'] / 10.0
df['Fan_V'] = df['Fan_dv'] / 10.0

# Force the Markers column to be an integer (handles any missing values gracefully)
df['Markers'] = df['Markers'].fillna(0).astype(int)

# Unpack 16-bit Markers using standard math instead of bitwise operators
# [Status:2][Left:7][Right:2][Cross:5]
df['Marker_Status'] = (df['Markers'] // 16384) % 4    # Shift 14 (2^14 = 16384), Mask 0x03 (Modulo 4)
df['Left_Markers']  = (df['Markers'] // 128) % 128    # Shift 7  (2^7 = 128),    Mask 0x7F (Modulo 128)
df['Right_Markers'] = (df['Markers'] // 32) % 4       # Shift 5  (2^5 = 32),     Mask 0x03 (Modulo 4)
df['Crossings']     = df['Markers'] % 32              # No shift,

# --- ADVANCED F1 DERIVED METRICS ---

# 1. Longitudinal Acceleration (G-Force)
# Calculate dv/dt and convert to Gs (1 G = 9.81 m/s^2)
dt = df['Time_s'].diff().fillna(0.01) # Assuming 10ms default
df['Long_Accel_G'] = (df['Robot_Vel'].diff() / dt) / 9.81
df['Long_Accel_G'] = df['Long_Accel_G'].rolling(window=5, center=True).mean().fillna(0) # Light smoothing

# 2. Yaw Rate & Lateral Acceleration (G-Force)
# Lat_Accel = v * yaw_rate
df['Yaw_Rate'] = df['Yaw_rad'].diff() / dt
df['Lat_Accel_G'] = (df['Robot_Vel'] * df['Yaw_Rate']) / 9.81
df['Lat_Accel_G'] = df['Lat_Accel_G'].rolling(window=5, center=True).mean().fillna(0)

# 3. Throttle / Brake "Pedals"
# Derived from PID Setpoints logic (assuming positive setpoints drive forward)
df['Throttle'] = np.clip(df['PidLSet'] + df['PidRSet'], 0, None) / (df['PidLSet'].max() + 1) * 100
df['Brake'] = np.clip(-df['Long_Accel_G'], 0, None) / (df['Long_Accel_G'].min() * -1 + 0.01) * 100

# ==========================================
# 3. MARKER PROJECTION LOGIC
# ==========================================

left_marker_shapes = []
right_marker_shapes = []

# Detect where counter increments to find the exact frame a marker was seen
left_marker_events = df[df['Left_Markers'].diff() > 0]
right_marker_events = df[df['Right_Markers'].diff() > 0]

def calc_marker_coords(row, side):
    """Calculates coordinates for a 4cm marker offset 4cm from the track."""
    x, y, theta = row['X_m'], row['Y_m'], row['Yaw_rad']
    offset = 0.04 # 4cm
    length = 0.04 # 4cm
    
    # Left is +90 deg (pi/2), Right is -90 deg (-pi/2) relative to heading
    angle_offset = np.pi/2 if side == 'left' else -np.pi/2
    perp_angle = theta + angle_offset
    
    # Start point (4cm perpendicular from robot center)
    start_x = x + offset * np.cos(perp_angle)
    start_y = y + offset * np.sin(perp_angle)
    
    # End point (4cm parallel to track from start point)
    end_x = start_x + length * np.cos(theta)
    end_y = start_y + length * np.sin(theta)
    
    return start_x, start_y, end_x, end_y

for _, row in left_marker_events.iterrows():
    sx, sy, ex, ey = calc_marker_coords(row, 'left')
    left_marker_shapes.append(go.Scatter(x=[sx, ex], y=[sy, ey], mode='lines', line=dict(color='#00ffcc', width=4), hoverinfo='skip'))

for _, row in right_marker_events.iterrows():
    sx, sy, ex, ey = calc_marker_coords(row, 'right')
    right_marker_shapes.append(go.Scatter(x=[sx, ex], y=[sy, ey], mode='lines', line=dict(color='#ff00ff', width=4), hoverinfo='skip'))

# ==========================================
# 4. BUILD THE DASHBOARD VISUALS
# ==========================================

app = dash.Dash(__name__)
app.title = "Raven Telemetry Center"

# F1 Style Theme Colors
bg_color = "#0e1117"
text_color = "#c9d1d9"
grid_color = "#30363d"

layout_template = go.layout.Template(
    layout=dict(
        plot_bgcolor=bg_color, paper_bgcolor=bg_color, font=dict(color=text_color),
        xaxis=dict(showgrid=True, gridcolor=grid_color, zerolinecolor=grid_color),
        yaxis=dict(showgrid=True, gridcolor=grid_color, zerolinecolor=grid_color),
        margin=dict(l=40, r=40, t=40, b=40)
    )
)

# --- TRACK MAP (Color-coded by speed) ---
fig_map = go.Figure()
fig_map.add_trace(go.Scatter(
    x=df['X_m'], y=df['Y_m'], mode='markers+lines',
    marker=dict(size=4, color=df['Robot_Vel'], colorscale='Turbo', showscale=True, colorbar=dict(title="Speed (m/s)")),
    line=dict(width=1, color='rgba(255,255,255,0.3)'),
    name="Racing Line"
))
# Add the calculated track markers
for shape in left_marker_shapes: fig_map.add_trace(shape)
for shape in right_marker_shapes: fig_map.add_trace(shape)

# Interactive Robot Dot (Will be moved by callbacks)
fig_map.add_trace(go.Scatter(x=[df['X_m'].iloc[0]], y=[df['Y_m'].iloc[0]], mode='markers', marker=dict(size=12, color='white', symbol='circle-cross'), name='Robot'))
fig_map.update_layout(template=layout_template, title="Track Map (Speed Gradient)", yaxis_scaleanchor="x") 

# --- FRICTION CIRCLE (G-G Diagram) ---
fig_gg = go.Figure()
fig_gg.add_trace(go.Scatter(x=df['Lat_Accel_G'], y=df['Long_Accel_G'], mode='markers', marker=dict(size=3, color='rgba(255, 0, 0, 0.4)'), name="G-Envelope"))
fig_gg.add_trace(go.Scatter(x=[0], y=[0], mode='markers', marker=dict(size=12, color='white'), name='Current G'))
# Add friction limits rings
fig_gg.add_shape(type="circle", x0=-1, y0=-1, x1=1, y1=1, line_color="gray", opacity=0.3)
fig_gg.add_shape(type="circle", x0=-2, y0=-2, x1=2, y1=2, line_color="gray", opacity=0.3)
fig_gg.update_layout(template=layout_template, title="Friction Circle (G-G)", xaxis_title="Lat Accel (G)", yaxis_title="Long Accel (G)", xaxis_range=[-3,3], yaxis_range=[-3,3])

# --- TIME SERIES GRAPHS ---
fig_time = make_subplots(rows=6, cols=1, shared_xaxes=True, vertical_spacing=0.03,
                         subplot_titles=("Velocity", "Line Position & PID Out", "Motor Setpoints", "Motor Voltages", "Derived Pedals (Throttle/Brake)", "Battery & Fan"))

# 1. Velocity
fig_time.add_trace(go.Scatter(x=df['Time_s'], y=df['VelL_mps'], name='Vel L', line=dict(color='#ff0055')), row=1, col=1)
fig_time.add_trace(go.Scatter(x=df['Time_s'], y=df['VelR_mps'], name='Vel R', line=dict(color='#00ccff')), row=1, col=1)
fig_time.add_trace(go.Scatter(x=df['Time_s'], y=df['Robot_Vel'], name='Robot Vel', line=dict(color='white', dash='dot')), row=1, col=1)

# 2. Line Pos & Line PID
fig_time.add_trace(go.Scatter(x=df['Time_s'], y=df['LinePos'], name='Line Pos', line=dict(color='yellow')), row=2, col=1)
fig_time.add_trace(go.Scatter(x=df['Time_s'], y=df['PidLine'], name='PID Line Out', line=dict(color='orange')), row=2, col=1)

# 3. Setpoints
fig_time.add_trace(go.Scatter(x=df['Time_s'], y=df['PidLSet'], name='Set L', line=dict(color='#ff0055')), row=3, col=1)
fig_time.add_trace(go.Scatter(x=df['Time_s'], y=df['PidRSet'], name='Set R', line=dict(color='#00ccff')), row=3, col=1)

# 4. Outputs
fig_time.add_trace(go.Scatter(x=df['Time_s'], y=df['PidLOut'], name='Out L', line=dict(color='#ff0055')), row=4, col=1)
fig_time.add_trace(go.Scatter(x=df['Time_s'], y=df['PidROut'], name='Out R', line=dict(color='#00ccff')), row=4, col=1)

# 5. Pedals (F1 Style)
fig_time.add_trace(go.Scatter(x=df['Time_s'], y=df['Throttle'], name='Throttle %', fill='tozeroy', line=dict(color='green')), row=5, col=1)
fig_time.add_trace(go.Scatter(x=df['Time_s'], y=df['Brake'], name='Brake %', fill='tozeroy', line=dict(color='red')), row=5, col=1)

# 6. Vitals
fig_time.add_trace(go.Scatter(x=df['Time_s'], y=df['Bat_V'], name='Bat (V)', line=dict(color='#00ff00')), row=6, col=1)
fig_time.add_trace(go.Scatter(x=df['Time_s'], y=df['Fan_V'], name='Fan (V)', line=dict(color='#bbbbbb')), row=6, col=1)

fig_time.update_layout(template=layout_template, height=1200, hovermode="x unified")


# APP LAYOUT
app.layout = html.Div(style={'backgroundColor': bg_color, 'padding': '20px', 'fontFamily': 'sans-serif'}, children=[
    html.H1("Raven Telemetry Center", style={'color': 'white', 'textAlign': 'center'}),
    
    html.Div([
        # Left Panel (Map)
        html.Div([dcc.Graph(id='track-map', figure=fig_map, style={'height': '600px'})], style={'width': '60%', 'display': 'inline-block', 'verticalAlign': 'top'}),
        
        # Right Panel (G-G Diagram)
        html.Div([dcc.Graph(id='gg-diagram', figure=fig_gg, style={'height': '600px'})], style={'width': '38%', 'display': 'inline-block', 'verticalAlign': 'top', 'paddingLeft': '2%'})
    ]),
    
    # Bottom Panel (Time Series)
    html.Div([dcc.Graph(id='time-series', figure=fig_time)])
])

# ==========================================
# 5. INTERACTIVE HOVER CALLBACKS
# ==========================================

@app.callback(
    [Output('track-map', 'figure'),
     Output('gg-diagram', 'figure')],
    [Input('time-series', 'hoverData')]
)
def update_map_position(hoverData):
    # Create patched figures to avoid re-sending the whole dataset to the browser
    patched_map = go.Figure(fig_map)
    patched_gg = go.Figure(fig_gg)
    
    if hoverData:
        # Get the time index from the hover event on the time-series graph
        hover_time = hoverData['points'][0]['x']
        
        # Find the closest row in the dataframe to this exact time
        idx = (df['Time_s'] - hover_time).abs().idxmin()
        
        # Update the Robot marker position on the Map (It's the last trace added)
        patched_map.data[-1].x = [df['X_m'].iloc[idx]]
        patched_map.data[-1].y = [df['Y_m'].iloc[idx]]
        
        # Update the Current G dot on the G-G Diagram (It's the 2nd trace)
        patched_gg.data[1].x = [df['Lat_Accel_G'].iloc[idx]]
        patched_gg.data[1].y = [df['Long_Accel_G'].iloc[idx]]
        
    return patched_map, patched_gg


if __name__ == '__main__':
    # Run the local web server
    app.run(debug=True)