import time
from datetime import datetime
import threading
import re

import RPi.GPIO as GPIO
from astropy.coordinates import AltAz, EarthLocation, SkyCoord, get_body, get_sun, get_moon
from astropy.time import Time
import astropy.units as u

# ----- CONFIGURATION -----
SENSOR_PINS = [13, 19, 20, 21]  # IR sensor GPIO pins (BOARD numbering)
RELAY_PIN_CW = 18   # Clockwise relay control
RELAY_PIN_CCW = 17  # Counter-clockwise relay control
HALL_SENSOR_PIN = 16  # Hall effect sensor for zero position (BOARD pin 16)

DEGREE_PER_TICK = 3.75        # 360° / (24 strips * 4 steps/strip)
TOLERANCE = DEGREE_PER_TICK   # Acceptable error = one step

# MITWPU coordinates
LAT = '18d31m05.8s'
LON = '73d48m45.6s'
ALT = 560  # MITWPU altitude in meters

# Create EarthLocation object
OBSERVER_LOC = EarthLocation(lat=LAT, lon=LON, height=ALT*u.m)

# ----- SYSTEM VARIABLES -----
current_angle = 0.0
angle_lock = threading.Lock()
last_state = [None] * len(SENSOR_PINS)
is_homed = False  # Track if dome has been homed

# ----- GPIO SETUP -----
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# Motor control setup
GPIO.setup(RELAY_PIN_CW, GPIO.OUT)
GPIO.setup(RELAY_PIN_CCW, GPIO.OUT)
GPIO.output(RELAY_PIN_CW, GPIO.HIGH)  # Initialize off (active low)
GPIO.output(RELAY_PIN_CCW, GPIO.HIGH) # Initialize off (active low)

# Sensor setup with pull-ups
for i, pin in enumerate(SENSOR_PINS):
    GPIO.setup(pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    last_state[i] = GPIO.input(pin)

# Hall effect sensor setup (normally open, goes LOW when triggered)
GPIO.setup(HALL_SENSOR_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

# ----- ENCODER READING -----
def read_encoder():
    global current_angle, last_state
    
    while True:
        for i, pin in enumerate(SENSOR_PINS):
            current_state = GPIO.input(pin)
            if current_state != last_state[i]:
                with angle_lock:
                    # Determine direction based on sensor activation order
                    if i == 0 and current_state == GPIO.LOW:
                        current_angle += DEGREE_PER_TICK
                    elif i == 1 and current_state == GPIO.LOW:
                        current_angle -= DEGREE_PER_TICK
                    current_angle %= 360  # Keep within 0-360 range
                last_state[i] = current_state
        time.sleep(0.01)  # Polling interval

# ----- HOMING FUNCTION -----
def home_dome():
    """Move dome to zero position using Hall effect sensor"""
    global current_angle, is_homed
    
    print("Homing dome to zero position...")
    
    # Check if already at home position
    if GPIO.input(HALL_SENSOR_PIN) == GPIO.LOW:
        print("Already at home position")
        with angle_lock:
            current_angle = 0.0
        is_homed = True
        return True
    
    # Move clockwise until sensor is triggered
    move_motor("CW")
    start_time = time.time()
    
    while GPIO.input(HALL_SENSOR_PIN) == GPIO.HIGH:
        # Timeout after 30 seconds
        if time.time() - start_time > 30:
            print("Homing timeout! Sensor not triggered.")
            move_motor("STOP")
            return False
        time.sleep(0.05)
    
    # Sensor triggered - stop motor and set position to 0°
    move_motor("STOP")
    
    # Move slightly past the sensor to ensure we're at the exact position
    move_motor("CCW")
    time.sleep(0.2)  # Move back for 200ms
    move_motor("STOP")
    
    with angle_lock:
        current_angle = 0.0
    is_homed = True
    print("Dome homed at 0° position")
    return True

# ----- OBJECT COORDINATE RESOLVER -----
def get_azimuth(target_input):
    """
    Resolve celestial objects or coordinates to azimuth
    Supports:
    - Object names (M31, Sirius, Orion Nebula, etc.)
    - Solar system bodies (Sun, Moon, planets)
    - Direct coordinates (RA/Dec or Alt/Az)
    """
    try:
        now = Time(datetime.utcnow(), scale='utc')
        target = target_input.strip().lower()
        
        # Check for coordinate input (RA Dec or Alt Az)
        coord_match = re.match(r'(-?\d+\.?\d*)[, ]\s*(-?\d+\.?\d*)', target_input)
        if coord_match:
            try:
                # Try to interpret as RA/Dec
                ra, dec = map(float, coord_match.groups())
                coord = SkyCoord(ra=ra*u.degree, dec=dec*u.degree, frame='icrs')
            except ValueError:
                # Try to interpret as Alt/Az
                alt, az = map(float, coord_match.groups())
                coord = SkyCoord(alt=alt*u.degree, az=az*u.degree, frame='altaz')
        else:
            # Handle named objects
            if target == "sun":
                coord = get_sun(now)
            elif target == "moon":
                coord = get_moon(now)
            elif target in ["mercury", "venus", "mars", "jupiter", "saturn", "uranus", "neptune"]:
                coord = get_body(target, now, OBSERVER_LOC)
            else:
                # Resolve deep-sky objects using online databases
                coord = SkyCoord.from_name(target_input)
        
        # Convert to Alt/Az coordinates
        altaz = coord.transform_to(AltAz(obstime=now, location=OBSERVER_LOC))
        return altaz.az.degree
        
    except Exception as e:
        print(f"[ERROR] Could not resolve '{target_input}': {str(e)}")
        return None

# ----- MOTOR CONTROL -----
def move_motor(direction):
    """Control motor with safety interlocks"""
    # First stop all motors
    GPIO.output(RELAY_PIN_CW, GPIO.HIGH)
    GPIO.output(RELAY_PIN_CCW, GPIO.HIGH)
    time.sleep(0.1)  # Safety delay
    
    if direction == "CW":
        GPIO.output(RELAY_PIN_CCW, GPIO.HIGH)  # Ensure no conflict
        GPIO.output(RELAY_PIN_CW, GPIO.LOW)
    elif direction == "CCW":
        GPIO.output(RELAY_PIN_CW, GPIO.HIGH)    # Ensure no conflict
        GPIO.output(RELAY_PIN_CCW, GPIO.LOW)
    else:  # Stop
        GPIO.output(RELAY_PIN_CW, GPIO.HIGH)
        GPIO.output(RELAY_PIN_CCW, GPIO.HIGH)

# ----- DOME MOVEMENT -----
def move_dome(target_azimuth):
    """Move dome to target azimuth with shortest path"""
    global current_angle
    
    # Ensure we're homed before moving
    if not is_homed:
        print("Warning: Dome not homed. Homing now...")
        if not home_dome():
            print("Homing failed! Cannot move to target.")
            return False
    
    with angle_lock:
        current = current_angle
        
    # Calculate shortest rotational path
    diff = (target_azimuth - current) % 360
    if diff > 180:
        diff -= 360
        
    # If already within tolerance, don't move
    if abs(diff) <= TOLERANCE:
        print(f"Already within tolerance ({abs(diff):.1f}° ≤ {TOLERANCE}°)")
        return True
        
    # Movement parameters
    direction = "CW" if diff > 0 else "CCW"
    print(f"Moving dome: {current:.1f}° → {target_azimuth:.1f}° ({direction}, Δ={diff:.1f}°)")
    
    move_motor(direction)
    start_time = time.time()
    last_update = time.time()
    
    while True:
        # Update position
        with angle_lock:
            current = current_angle
            
        # Calculate new difference
        diff = (target_azimuth - current) % 360
        if diff > 180:
            diff -= 360
            
        # Check if we're within tolerance
        if abs(diff) <= TOLERANCE:
            break
            
        # Update display every 0.5 seconds
        if time.time() - last_update > 0.5:
            print(f"Current: {current:.1f}°, Target: {target_azimuth:.1f}°, Remaining: {abs(diff):.1f}°")
            last_update = time.time()
            
        # Reverse direction if overshoot detected
        if (direction == "CW" and diff < 0) or (direction == "CCW" and diff > 0):
            print(f"Overshoot detected, reversing to {'CCW' if direction == 'CW' else 'CW'}")
            direction = "CCW" if direction == "CW" else "CW"
            move_motor(direction)
            
        # Timeout safety (30 seconds)
        if time.time() - start_time > 30:
            print("Movement timeout!")
            break
            
        time.sleep(0.05)
    
    move_motor("STOP")
    with angle_lock:
        final_angle = current_angle
    final_diff = (target_azimuth - final_angle) % 360
    if final_diff > 180:
        final_diff -= 360
    print(f"Dome positioned at {final_angle:.1f}° (Error: {abs(final_diff):.1f}°)")
    return abs(final_diff) <= TOLERANCE

# ----- MAIN CONTROL LOOP -----
def main():
    global is_homed
    
    # Start encoder reader thread
    encoder_thread = threading.Thread(target=read_encoder, daemon=True)
    encoder_thread.start()
    
    # Allow time for encoder initialization
    time.sleep(1)
    
    # Home the dome at startup
    if not home_dome():
        print("Critical error: Homing failed! Proceeding with unknown position")
    
    try:
        print("\n" + "="*50)
        print("DOME CONTROL SYSTEM - MITWPU OBSERVATORY")
        print("="*50)
        print(f"Current dome position: {current_angle:.1f}°")
        print(f"Tolerance: ±{TOLERANCE}°")
        print("Supported inputs:")
        print("- Named objects (e.g., 'M31', 'Orion Nebula', 'Sirius')")
        print("- Solar system bodies (e.g., 'Sun', 'Moon', 'Jupiter')")
        print("- Coordinates (e.g., '34.5, 56.7' for RA/Dec or Alt/Az)")
        print("- 'home' to re-home the dome")
        print("- 'exit' to quit\n")
        
        while True:
            try:
                # Get user input
                user_input = input("Enter celestial object or command: ").strip()
                if user_input.lower() == 'exit':
                    break
                    
                # Handle homing command
                if user_input.lower() == 'home':
                    home_dome()
                    continue
                    
                # Get target azimuth
                target_az = get_azimuth(user_input)
                if target_az is None:
                    print("Invalid target, please try again")
                    continue
                    
                print(f"Target azimuth: {target_az:.1f}°")
                
                # Move dome
                success = move_dome(target_az)
                
                if not success:
                    print("Warning: Failed to reach target within tolerance")
                else:
                    # Tracking mode
                    print("\nStarting tracking mode (press Ctrl+C to stop)...")
                    last_az = target_az
                    last_print = time.time()
                    try:
                        while True:
                            new_az = get_azimuth(user_input)
                            if new_az is None:
                                print("Skipping update due to resolution error")
                            else:
                                # Only move if significant change (beyond tolerance)
                                with angle_lock:
                                    current = current_angle
                                az_diff = (new_az - current) % 360
                                if az_diff > 180:
                                    az_diff -= 360
                                    
                                if abs(az_diff) > TOLERANCE:
                                    print(f"\nTarget moved: {new_az:.1f}° (Δ={az_diff:.1f}°)")
                                    move_dome(new_az)
                                    last_az = new_az
                                else:
                                    # Print status every 30 seconds
                                    if time.time() - last_print > 30:
                                        print(f"Tracking: Dome {current_angle:.1f}°, Target {new_az:.1f}°")
                                        last_print = time.time()
                            time.sleep(5)  # Update every 5 seconds
                    except KeyboardInterrupt:
                        print("Stopping tracking...")
                        move_motor("STOP")
                    
            except Exception as e:
                print(f"System error: {str(e)}")
                
    except KeyboardInterrupt:
        print("\nExiting...")
    finally:
        move_motor("STOP")
        GPIO.cleanup()
        print("System shutdown complete")

if __name__ == "__main__":
    main()