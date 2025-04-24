"""
Issues to discuss:
- Rover Module resets Encoders after each Command. Causes problems with velocity control.
- Rover Module is most likely not reading from the gyro properly, or the gyro isn't really a gyro.
- Rover Module's documentation is heavily lacking and very confusing.
- Rover Docs don't mention the Command Queue.
- Rovers have inconsistent drift when driving forwards. (can be fixed with PID, though I believe you have this already)
- Rover has very slow update rate, for example 0.1 seconds is the lowest time you can set for the motors.
- Rover Module doesn't provide a way to send a continuous stream of signal to the motors.
- Rover Module doesn't provide a way to read the encoders continuously, rover must be executing a command to read the encoders.
- Rover Module couples the Encoders and Gyro together when reading, which is weird.
- The Rover Module doesn't send enough data to us.

Examples of why these are issues:
- Say you want to write code to drive until the USS detects an object, then stop.
- Currently, you would be driving forwards in the smallest time increments possible, then stopping once the USS detects an object.
- This is just not a fun solution at all, especially when you're not provided with any alternative method.
- Now if the Module provided a way to send a continuous stream of signal to the motors, you could just send a signal to the motors to drive forwards, then stop once the USS detects an object.
- This would eliminate the jitter, and make the code much cleaner and easier to read.

- Not having a continuous signal to the motors, and resetting the encoders means I cannot PID tune the Velocity of the motors properly.

- The Command Queue, while it is easy to understand, and I'm not saying it should be removed. It needs to be documented, and there should be a way to not use it.
- Especially when the Command Queue adds a overhead to the system. The Queue will queue commands after the requested 0.1 seconds.
- Basically if you call motors.drive(cw, 100, ccw, 100, 0.1) it takes longer than 0.1 seconds, lowest I've gotten without hitting a queue is about 0.235 seconds.
- This is a huge problem, since speed is everything in robotics, this queue system is practically halfing the potential update rate of the rover.

- The Rover's gyro is extremely confusing. It looks like its reading velocity and not yaw, which is what you would expect from the gyro.
- I'm unsure if this is an issue with the Gyro, or if the Module is just reading the wrong data.
- This is a big problem, since I want to be able to drive forward and turn at the same time, but I cannot do this because I don't know what the gyro is doing.
- While there is work arounds for this, they take too much time to implement, especially for students who are just starting out, and don't know all the niche math behind Robotic systems like this.

- The rover module doesn't send enough data to us.
- I know that you must have data like: Velocity, Distance, Angle, etc. Since you have Commands that ask for a desired Velocity, Angle, and Distance, but none of these are avilable to us.
- If they are available to us, it's not documented, easy to find the documentation, or easy to find on the calculator.
"""


# Rover Coding 
import ti_rover as rv 
import ti_plotlib as plt 
from ti_system import * 
from ti_hub import * 
import math 
import time 



# Rover Constants 
PWM_MAX = 255 
PWM_MIN = 0 
WHEEL_RADIUS_METERS = 0.03175 # 1.25 inches
# TODO: Recalc GearRatio with a greater distance to be more accurate
GEAR_RATIO = 366.2788 # Calculated using Gr = (2*pi*r*Re) / d | r = wheel radius in meters, Re = encoder ticks, d = distance traveled by the wheel in meters
MOTOR_DEBOUNCE_MS = 235 # Minimum amount of time to wait before updating Motors again || The number could be lower 
WHEEL_BASE_METERS = 0.15 # Distance between the wheels in meters 


# Helper functions / classes | Don't rely on Globals to work, thought may use Constants 
class PIDController: 
    def __init__(self, kP: float, kI: float, kD: float, iz: float, period: float=(MOTOR_DEBOUNCE_MS/1000.0), tolerance: float=0): 
        """ 
        PID Controller for the Rover. \n 
        kP, kI, kD are the PID constants. \n 
        iz is the integral zone. If the error is less than this value, the integral term will be updated. \n 
        period is the time between updates in seconds. \n 
        tolerance is the tolerance for the setpoint. If the error is less than this value, the controller will consider it at setpoint. 
        """ 
        self.kP = kP 
        self.kI = kI 
        self.kD = kD 
        self.iz = iz 
        self.periodSeconds = period 
        self.prevError = 0 
        self.totalError = 0 
        self.error = 0 
        self.measurement = 0 
        self.setpoint = 0 
        self.tolerance = tolerance 

    def calc(self, measurement): 
        self.measurement = measurement 
        self.error = self.setpoint - self.measurement 

        if math.fabs(self.error) < self.iz: 
            self.totalError += self.error * self.periodSeconds 
        else: 
            self.totalError = 0 

        pTerm = self.kP * self.error 
        iTerm = self.kI * self.totalError 
        dTerm = self.kD * (self.error - self.prevError) / self.periodSeconds 

        output = pTerm + iTerm + dTerm 

        self.prevError = self.error 

        return output 
    
    def reset(self, newSetpoint: float=0.0): 
        self.prevError = 0 
        self.totalError = 0 
        self.error = 0 
        self.measurement = 0 
        self.setpoint = newSetpoint 

    def atSetpoint(self): 
        return math.fabs(self.error) < self.tolerance 


def clamp(num, mini, maxi): 
    return max(mini, min(num, maxi)) 


def getDir(dutyCycle: int): 
    """ 
    Duty Cycle is read as Right Motor positive is forward. \n 
    DutyCycle param should be inverted for the Left Motor. 
    """ 
    return "cw" if dutyCycle > 0 else "ccw" 


def drive(leftDC, rightDC, period = 0.1): 
    """ 
    leftDC and rightDC are the DutyCycle for the Left and Right Motors. \n 
    Positive DutyCycle is forwards, Negative is backwards. \n 
    Period defaults to lowest possible value for the Rover. 
    """ 
    rv.motors(getDir(-leftDC), clamp(leftDC, PWM_MIN, PWM_MAX), getDir(rightDC), clamp(rightDC, PWM_MIN, PWM_MAX), period) 


def getDist(rots): 
    """ 
    Get the distance travelled by the wheel in meters. \n 
    rots is the number of rotations read from the encoders. 
    """ 
    return (rots * 2 * math.pi * WHEEL_RADIUS_METERS) / GEAR_RATIO 


def calculateVelocity(currentTicks, previousTicks, dt): 
    """ 
    Calculate the velocity of the wheel in m/s. \n 
    currentTicks is the current number of ticks read from the encoders. \n 
    previousTicks is the previous number of ticks read from the encoders. \n 
    dt is the time since the last update in seconds. 
    """ 
    return getDist(currentTicks - previousTicks) / dt if dt != 0 else 0.0 



# Globals 
deltaTime = 0.0 # Change in time of the Main Loop in seconds 
lastTime = time.time() # Last time the loop was run 

encoders_gyro_measurement = [0, 0, 0, 0] # [Left Encoder Revs, Right Encoder Revs, Gyro???, Time???] 
leftTicks = 0 # Left Encoder Ticks 
rightTicks = 0 # Right Encoder Ticks 
prevLeftTicks = 0 # Previous Left Encoder Ticks | Used to calculate velocity 
prevRightTicks = 0 # Previous Right Encoder Ticks | Used to calculate velocity 

leftDistance = 0.0 # Distance travelled by the left wheel in meters 
rightDistance = 0.0 # Distance travelled by the right wheel in meters 
avgDistance = 0.0 # Average distance travelled by both wheels in meters 

angleTurned = 0.0 # Angle turned in radians 

leftVelocityController = PIDController(800, 0.0, 0.0, 0.0, MOTOR_DEBOUNCE_MS/1000, 0) 
rightVelocityController = PIDController(800, 0.0, 0.0, 0.0, MOTOR_DEBOUNCE_MS/1000, 0) 

driveDebounceLastTime = 0.0 # Last time the drive function was called 



def driveWithDebounce(leftDC, rightDC): 
    """ 
    leftDC and rightDC are the DutyCycle for the Left and Right Motors. \n 
    Positive DutyCycle is forwards, Negative is backwards. \n 
    Period defaults to lowest possible value for the Rover. 
    This function will only call the drive function if the time since the last call is greater than MOTOR_DEBOUNCE_MS. 
    This is to prevent the motors from being called too often causing the Rover library to queue up commands and not work properly. 
    """ 
    global driveDebounceLastTime 
    if get_time_ms() - driveDebounceLastTime > MOTOR_DEBOUNCE_MS: 
        driveDebounceLastTime = get_time_ms() 
        rv.motors(getDir(-leftDC), clamp(leftDC, PWM_MIN, PWM_MAX), getDir(rightDC), clamp(rightDC, PWM_MIN, PWM_MAX), 0.1) 


def driveVelocityWithDebounce(leftVelocitySetpoint, rightVelocitySetpoint): 
    """ 
    leftVelocity and rightVelocity are the velocity of the Left and Right Motors in m/s. \n 
    Positive Velocity is forwards, Negative is backwards.  \n
    This function will only call the drive function if the time since the last call is greater than MOTOR_DEBOUNCE_MS. \n
    This is to prevent the motors from being called too often causing the Rover library to queue up commands and not work properly. \n

    This doesn't work since the Rover Module resets the Encoders after each Command :)
    """ 
    global driveDebounceLastTime, deltaTime, leftTicks, rightTicks, prevLeftTicks, prevRightTicks
    dt = get_time_ms() - driveDebounceLastTime
    if dt >= MOTOR_DEBOUNCE_MS: 
        driveDebounceLastTime = get_time_ms() 
        leftVelocityController.setpoint = leftVelocitySetpoint 
        rightVelocityController.setpoint = rightVelocitySetpoint
        lm = calculateVelocity(leftTicks, prevLeftTicks, dt / 1000)
        rm = calculateVelocity(rightTicks, prevRightTicks, dt / 1000)
        leftDutyCycle = leftVelocityController.calc(lm) 
        rightDutyCycle = rightVelocityController.calc(rm) 
        # print(leftDutyCycle) 
        # print(rightDutyCycle)
        # print(lm)
        drive(leftDutyCycle, rightDutyCycle) 


# Rover Functions | Requires Globals 
def driveVelocityTime(leftVelocitySetpoint, rightVelocitySetpoint, seconds = 0.1): 
    """ 
    leftVelocity and rightVelocity are the velocity of the Left and Right Motors in m/s. \n 
    Positive Velocity is forwards, Negative is backwards. 
    time is the time to drive in seconds. \n 
    This is a closed system, once this function is called, it won't end until the time is up 
    """ 
    leftVelocityController.reset(leftVelocitySetpoint) 
    rightVelocityController.reset(rightVelocitySetpoint) 

    endTime = (get_time_ms() / 1000) + seconds 
    currentTime = get_time_ms() / 1000 
    previousTime = 0 
    previousLeftTicks = 0 
    previousRightTicks = 0 
    while endTime > currentTime: 
        # Get the current time 
        currentTime = get_time_ms() / 1000 
        deltaTime = currentTime - previousTime 

        # Get Encoder readings 
        encoders_gyro_measurement = rv.encoders_gyro_measurement() 
        leftTicks = encoders_gyro_measurement[0] 
        rightTicks = encoders_gyro_measurement[1] 

        leftVelocity = calculateVelocity(leftTicks, previousLeftTicks, deltaTime) 
        rightVelocity = calculateVelocity(rightTicks, previousRightTicks, deltaTime) 

        driveWithDebounce(leftVelocityController.calc(leftVelocity), rightVelocityController.calc(rightVelocity)) 

        previousTime = currentTime 
        previousLeftTicks = leftTicks 
        previousRightTicks = rightTicks 




# Rover Tests 
def rvTest_driveForward(): 
    """ 
    Shows off the drift the Rover experiences when driving forwards. 
    """ 
    rv.forward_time(5) # Drive forwards for 5 seconds 
    rv.stop() 


def rvTest_commandQueue(): 
    """ 
    Tests the undocumented Rover drive Command queue. 
    Very annoying when trying to continuously update motor speeds. 
    Especially when none of the specifics for how it works are documented.
    Test the how long the Queue takes VS. how long the drive command says it takes 
    """ 
    print("Starting Drive Command Time Test") 
    startSeconds = get_time_ms() / 1000.0 # convert to seconds 
    rv.motors("cw", 100, "ccw", 100, 0.1) # 0.1 lowest input available 
    rv.wait_until_done() # Wait for the command to finish 
    endSeconds = get_time_ms() / 1000.0 # convert to seconds 
    print("Drive Command Time Test Done") 
    print("Drive Command Time: " + str(endSeconds - startSeconds) + " seconds") 


def rvTest_gyro(): 
    """ 
    This is definitely not reading a gyro. 
    """ 
    def getGyroFromEncodersGyro(): 
        return rv.encoders_gyro_measurement()[2] 
    # Define the function to test reading from 
    read = rv.gyro_measurement 
    # read = getGyroFromEncodersGyro 

    intervalSeconds = 0.5 # Time between readings in seconds 
    prevInverval = 0 
    while get_key() != "esc": 
        if get_time_ms() - prevInverval > intervalSeconds * 1000: 
            print(read()) 
            prevInverval = get_time_ms() 




def main(): 
    """ 
    Main function for the Rover. \n 
    This is placed inside the Main Loop. \n 
    This is here to make sure you don't mess up the variables updating each loop. 
    """ 
    # driveVelocityWithDebounce(0.2, 0.2) # Drive forwards at 0.5 m/s 
    # print("LV: " + str(calculateVelocity(leftTicks, prevLeftTicks, deltaTime))) 
    # rvTest_gyro() 
    # print(avgDistance)
    # print(leftTicks)
    # print(calculateVelocity(leftTicks, prevLeftTicks, deltaTime))


# Run Command Tests here, setup while loop to display additional data
# rvTest_gyro()
# rvTest_commandQueue()
# rvTest_driveForward()
# rv.forward(0.305,"m")
# rv.forward_time(3,0.2,"m/s")

# MAIN LOOP 
print("Start")
while get_key() != "esc": 
    currentTime = get_time_ms() / 1000.0 # Convert to seconds 
    deltaTime = currentTime - lastTime 

    # Get the encoders and gyro measurement 
    encoders_gyro_measurement = rv.encoders_gyro_measurement() # returns nothing until code tells the Rover to move 
    leftTicks = encoders_gyro_measurement[0] 
    rightTicks = encoders_gyro_measurement[1] 

    # Calculate the distance travelled by each wheel in meters 
    leftDistance = getDist(encoders_gyro_measurement[0]) # Left Encoder Rots 
    rightDistance = getDist(encoders_gyro_measurement[1]) # Right Encoder Rots 
    avgDistance = (leftDistance + rightDistance) / 2.0 

    # Calculate the angle turned in radians
    # untested
    # angleTurned = (rightDistance - leftDistance) / WHEEL_BASE_METERS 

    main() 

    lastTime = currentTime 
    prevLeftTicks = leftTicks 
    prevRightTicks = rightTicks 


