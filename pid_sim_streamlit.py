import streamlit as st
import plotly.graph_objects as go
import plotly.express as px
from plotly.subplots import make_subplots
import numpy as np
import time
import pandas as pd

class SkidSteerPIDSimulator:
    def __init__(self):
        # PID Controller State
        self.current_heading = 0.0  # Current vehicle heading in degrees
        self.integral = 0.0         # Accumulated error
        self.prev_error = 0.0       # Previous error for derivative calculation
        self.start_time = time.time()
        
        # Simulation State
        self.simulation_running = True
        self.dt = 0.1  # Time step in seconds
        
        # Data for plotting
        self.time_series = []
        self.error_series = []
        self.output_series = []
        self.heading_series = []
        self.desired_series = []
        
        # Vehicle physics parameters
        self.max_turn_rate = 45.0  # Maximum turn rate in degrees/second
        self.vehicle_width = 2.0   # Vehicle width for skid-steer calculations
        
        # PID Parameters (will be controlled by UI)
        self.kp = 1.0
        self.ki = 0.0
        self.kd = 0.1
        self.desired_heading = 90.0
        
    def calculate_pid_output(self):
        """Calculate PID controller output with proper error handling"""
        # Calculate error (handle angle wrapping)
        error = self.desired_heading - self.current_heading
        
        # Handle angle wrapping (shortest path)
        while error > 180:
            error -= 360
        while error < -180:
            error += 360
        
        # PID calculations
        self.integral += error * self.dt
        derivative = (error - self.prev_error) / self.dt
        
        # Calculate output
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        
        # Apply realistic constraints (max turn rate)
        output = max(-self.max_turn_rate, min(self.max_turn_rate, output))
        
        self.prev_error = error
        return output, error
    
    def update_simulation(self):
        """Update the simulation state"""
        if not self.simulation_running:
            # Return current values when paused
            error = self.desired_heading - self.current_heading
            while error > 180: error -= 360
            while error < -180: error += 360
            output = self.kp * error + self.ki * self.integral + self.kd * (error - self.prev_error) / self.dt
            output = max(-self.max_turn_rate, min(self.max_turn_rate, output))
            return output, error
        
        # Calculate PID output
        output, error = self.calculate_pid_output()
        
        # Update vehicle heading (skid-steer physics)
        self.current_heading += output * self.dt
        
        # Normalize heading to 0-360 range
        self.current_heading = self.current_heading % 360
        
        # Record data for plotting
        t = time.time() - self.start_time
        self.time_series.append(t)
        self.error_series.append(error)
        self.output_series.append(output)
        self.heading_series.append(self.current_heading)
        self.desired_series.append(self.desired_heading)
        
        # Keep only last 500 data points to prevent memory issues
        if len(self.time_series) > 500:
            self.time_series = self.time_series[-500:]
            self.error_series = self.error_series[-500:]
            self.output_series = self.output_series[-500:]
            self.heading_series = self.heading_series[-500:]
            self.desired_series = self.desired_series[-500:]
        
        return output, error
    
    def reset_simulation(self):
        """Reset the simulation to initial state"""
        self.current_heading = 0.0
        self.integral = 0.0
        self.prev_error = 0.0
        self.start_time = time.time()
        
        # Clear data series
        self.time_series.clear()
        self.error_series.clear()
        self.output_series.clear()
        self.heading_series.clear()
        self.desired_series.clear()
        
        st.success("Simulation reset!")

def main():
    st.set_page_config(page_title="Skid-Steer PID Visualizer", layout="wide")
    
    st.title("🧭 Skid-Steer PID Visualizer")
    st.markdown("Real-time PID controller simulation for skid-steer vehicle heading control")
    
    # Initialize session state
    if 'simulator' not in st.session_state:
        st.session_state.simulator = SkidSteerPIDSimulator()
    
    simulator = st.session_state.simulator
    
    # Sidebar controls
    st.sidebar.header("🎛️ PID Controller Parameters")
    
    # PID sliders
    kp = st.sidebar.slider("Kp (Proportional)", 0.0, 10.0, simulator.kp, 0.1, key="kp_slider")
    ki = st.sidebar.slider("Ki (Integral)", 0.0, 2.0, simulator.ki, 0.01, key="ki_slider")
    kd = st.sidebar.slider("Kd (Derivative)", 0.0, 2.0, simulator.kd, 0.01, key="kd_slider")
    desired_heading = st.sidebar.slider("Desired Heading (degrees)", 0.0, 360.0, simulator.desired_heading, 1.0, key="desired_slider")
    
    # Update simulator parameters
    simulator.kp = kp
    simulator.ki = ki
    simulator.kd = kd
    simulator.desired_heading = desired_heading
    
    # Control buttons
    st.sidebar.header("🎮 Controls")
    col1, col2 = st.sidebar.columns(2)
    
    if col1.button("🔄 Reset", use_container_width=True):
        simulator.reset_simulation()
    
    if col2.button("⏸️ Pause/Resume", use_container_width=True):
        simulator.simulation_running = not simulator.simulation_running
        if simulator.simulation_running:
            st.sidebar.success("Simulation resumed")
        else:
            st.sidebar.warning("Simulation paused")
    
    # Main content area
    col1, col2 = st.columns([2, 1])
    
    with col1:
        st.header("📊 Simulation Results")
        
        # Update simulation
        output, error = simulator.update_simulation()
        
        # Create plots
        if len(simulator.time_series) > 1:
            # Create subplots
            fig = make_subplots(
                rows=3, cols=1,
                subplot_titles=("Vehicle Heading Over Time", "Heading Error Over Time", "PID Controller Output Over Time"),
                vertical_spacing=0.08,
                row_heights=[0.4, 0.3, 0.3]
            )
            
            # Heading plot
            fig.add_trace(
                go.Scatter(x=simulator.time_series, y=simulator.heading_series, 
                          mode='lines', name='Current Heading', line=dict(color='green', width=2)),
                row=1, col=1
            )
            fig.add_trace(
                go.Scatter(x=simulator.time_series, y=simulator.desired_series, 
                          mode='lines', name='Desired Heading', line=dict(color='red', width=2, dash='dash')),
                row=1, col=1
            )
            
            # Error plot
            fig.add_trace(
                go.Scatter(x=simulator.time_series, y=simulator.error_series, 
                          mode='lines', name='Error', line=dict(color='red', width=2)),
                row=2, col=1
            )
            
            # Output plot
            fig.add_trace(
                go.Scatter(x=simulator.time_series, y=simulator.output_series, 
                          mode='lines', name='PID Output', line=dict(color='blue', width=2)),
                row=3, col=1
            )
            
            # Update layout
            fig.update_layout(
                height=600,
                showlegend=True,
                legend=dict(orientation="h", yanchor="bottom", y=1.02, xanchor="right", x=1)
            )
            
            # Update axes labels
            fig.update_xaxes(title_text="Time (s)", row=3, col=1)
            fig.update_yaxes(title_text="Heading (degrees)", row=1, col=1)
            fig.update_yaxes(title_text="Error (degrees)", row=2, col=1)
            fig.update_yaxes(title_text="Output (degrees/s)", row=3, col=1)
            
            st.plotly_chart(fig, use_container_width=True)
        else:
            st.info("Simulation starting... Move the sliders to see the PID controller in action!")
    
    with col2:
        st.header("📈 Current Values")
        
        # Current values display
        st.metric("Current Heading", f"{simulator.current_heading:.1f}°")
        st.metric("Error", f"{error:.1f}°")
        st.metric("PID Output", f"{output:.1f}°/s")
        st.metric("Desired Heading", f"{simulator.desired_heading:.1f}°")
        
        # PID component breakdown
        st.subheader("🔧 PID Components")
        
        # Calculate individual components
        error = simulator.desired_heading - simulator.current_heading
        while error > 180: error -= 360
        while error < -180: error += 360
        
        proportional = simulator.kp * error
        integral = simulator.ki * simulator.integral
        derivative = simulator.kd * (error - simulator.prev_error) / simulator.dt
        
        st.metric("Proportional (P)", f"{proportional:.1f}")
        st.metric("Integral (I)", f"{integral:.1f}")
        st.metric("Derivative (D)", f"{derivative:.1f}")
        
        # Simulation info
        st.subheader("ℹ️ Simulation Info")
        st.write(f"**Data Points:** {len(simulator.time_series)}")
        st.write(f"**Simulation Time:** {simulator.time_series[-1] if simulator.time_series else 0:.1f}s")
        st.write(f"**Status:** {'🟢 Running' if simulator.simulation_running else '🔴 Paused'}")
    
    # Auto-refresh every 100ms (10 FPS)
    time.sleep(0.1)
    st.rerun()

if __name__ == "__main__":
    main() 