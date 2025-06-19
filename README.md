# PID Simulator

A Python project for PID (Proportional-Integral-Derivative) control simulation using Streamlit and Plotly.

## 🧭 High-Level Purpose

This script simulates a skid-steer vehicle being steered by a PID controller. The goal is to match the vehicle's actual heading (direction it's pointing) to a desired heading over time.

It allows a user to experiment with different PID parameters (Kp, Ki, Kd) and observe the effects in real time through:

- A live numerical readout of the current heading
- A graph of the heading error over time
- A graph of the PID controller's output over time
- Real-time parameter adjustment with immediate visual feedback

## Setup Instructions

This project uses Python 3.13 with pyenv for version management and virtual environments.

### Prerequisites

1. **Install pyenv** (if not already installed):

   ```bash
   # On macOS with Homebrew
   brew install pyenv
   
   # On Linux
   curl https://pyenv.run | bash
   
   # Add to your shell profile (~/.zshrc, ~/.bashrc, etc.)
   echo 'export PYENV_ROOT="$HOME/.pyenv"' >> ~/.zshrc
   echo 'command -v pyenv >/dev/null || export PATH="$PYENV_ROOT/bin:$PATH"' >> ~/.zshrc
   echo 'eval "$(pyenv init -)"' >> ~/.zshrc
   ```

2. **Restart your shell** or reload your profile:

   ```bash
   source ~/.zshrc
   ```

### Project Setup

1. **Navigate to the project directory**:

   ```bash
   cd /Users/chrisvaughn/code/pidSim
   ```

2. **Install Python 3.13** (if not already installed):

   ```bash
   pyenv install 3.13
   ```

3. **Create a virtual environment**:

   ```bash
   pyenv virtualenv 3.13 pidSim
   ```

4. **Set the local Python version** for this project:

   ```bash
   pyenv local pidSim
   ```

5. **Verify the setup**:

   ```bash
   python --version  # Should show Python 3.13
   which python      # Should show path to your virtual environment
   ```

6. **Activate the virtual environment** (if not already active):

   ```bash
   pyenv activate pidSim
   ```

### Install Dependencies

1. **Install required packages**:

   ```bash
   pip install -r requirements.txt
   ```

2. **Verify installation**:

   ```bash
   streamlit --version
   ```

## Running the Simulation

### Start the PID Simulator

1. **Run the Streamlit app**:

   ```bash
   streamlit run pid_sim_streamlit.py
   ```

2. **Open your browser** to the URL shown (usually `http://localhost:8501`)

3. **Start experimenting** with the PID parameters!

### Using the Interface

#### 🎛️ **PID Controller Parameters** (Sidebar)

- **Kp (Proportional)**: Controls how aggressively the system responds to error
- **Ki (Integral)**: Eliminates steady-state error over time
- **Kd (Derivative)**: Reduces overshoot and improves stability
- **Desired Heading**: Target direction for the vehicle (0-360°)

#### 🎮 **Controls**

- **Reset**: Clear simulation data and start over
- **Pause/Resume**: Stop or continue the simulation

#### 📊 **Real-Time Displays**

- **Current Values**: Live metrics showing current state
- **PID Components**: Breakdown of P, I, D contributions
- **Simulation Info**: Data points, time, and status

#### 📈 **Interactive Plots**

- **Vehicle Heading**: Current vs desired heading over time
- **Heading Error**: Error magnitude over time
- **PID Output**: Controller output over time

## Development Workflow

- **Activate the environment**: `pyenv activate pidSim`
- **Deactivate the environment**: `pyenv deactivate`
- **Install packages**: `pip install <package-name>`
- **Update requirements**: `pip freeze > requirements.txt`
- **Install from requirements**: `pip install -r requirements.txt`

## Project Structure

```
pidSim/
├── README.md
├── requirements.txt
├── pid_sim_streamlit.py      # Main Streamlit application
├── pid_sim.py               # Original DearPyGui version (experimental)
└── .python-version          # Created by pyenv local
```

## ⚙️ What the Simulation Does Internally

1. **PID Controller Loop**
   - Each iteration (every 0.1 seconds):
     - **Inputs**: Desired heading, current heading, time elapsed
     - **Calculations**:
       - `error = desired_heading - current_heading`
       - `integral += error * dt` (accumulates total error)
       - `derivative = (error - previous_error) / dt` (rate of change)
       - `output = kp * error + ki * integral + kd * derivative`
     - **Effect**: Output is applied as turning force to update heading

2. **Simulation State Tracking**
   - Stores time series, error, and PID output data
   - Used for real-time plotting and analysis
   - Maintains last 500 data points for performance

3. **Realistic Physics**
   - Maximum turn rate constraint (45°/s)
   - Proper angle wrapping for shortest-path error calculation
   - Skid-steer vehicle dynamics

## 💡 Why This Is Useful

This simulation provides an intuitive understanding of PID controller behavior:

- **Visual Learning**: See how each component (P, I, D) affects system response
- **Parameter Tuning**: Experiment with different gains to understand their effects
- **Real-Time Feedback**: Immediate visual response to parameter changes
- **Educational Tool**: Perfect for learning control theory concepts

## Next Steps

1. **Experiment with Parameters**: Try different Kp, Ki, Kd combinations
2. **Add Features**: 2D vehicle simulation, realistic physics, disturbances
3. **Extend Functionality**: Multiple PID controllers, different vehicle types
4. **Share Results**: Document interesting parameter combinations and behaviors

## Troubleshooting

### Common Issues

1. **Port already in use**: Change port with `streamlit run pid_sim_streamlit.py --server.port 8502`
2. **Slow performance**: Reduce data points or increase update interval
3. **Plot not updating**: Check browser console for errors

### Getting Help

- Check the Streamlit documentation: <https://docs.streamlit.io/>
- Review the Plotly documentation: <https://plotly.com/python/>
- Examine the console output for error messages

## Notes

- The virtual environment will be automatically activated when you enter the project directory
- You can see all available Python versions with: `pyenv versions`
- You can see all virtual environments with: `pyenv virtualenvs`
- The simulation runs at 10 FPS for smooth real-time updates
