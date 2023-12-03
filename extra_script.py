Import("env")

# List installed packages
env.Execute("$PYTHONEXE -m pip list")

# Install custom packages from the PyPi registry
env.Execute("$PYTHONEXE -m pip install opencv-python pyserial")

# Install missed package
try:
    import cv2
    import serial
except ImportError:
    env.Execute("$PYTHONEXE -m pip install opencv-python")