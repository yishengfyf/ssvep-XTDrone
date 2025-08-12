import subprocess
import time

def open_terminal(command):
    """Open a new gnome-terminal and execute the given command."""
    subprocess.Popen(['gnome-terminal', '--', 'bash', '-c', command])

if __name__ == "__main__":
    # List of (command, delay) tuples
    # delay: time to wait (in seconds) after executing the command before starting the next one
    commands = [
        ("cd ~/PX4_Firmware && roslaunch px4 outdoor3.launch", 5.0),  # 2 seconds delay after first command
        ("cd ~/XTDrone/communication && python3 multirotor_communication.py iris 0", 2.0),  # 5 second delay after second command
        ("cd ~/XTDrone/bci && python3 bci_keyboard_control.py iris vel", 0.0)  # No delay after third command
    ]

    # Open each terminal and execute the command with specified delay
    for cmd, delay in commands:
        open_terminal(cmd)
        time.sleep(delay)  # Wait for the specified delay before opening the next terminal
