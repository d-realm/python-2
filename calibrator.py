def ultraTurn(x):
    if (ultraMotor.position != x):
        ultraMotor.position_sp = x
        ultraMotor.run_to_abs_pos(speed_sp = SCAN_SPEED)
        ultraMotor.wait_while('running')
        sleep(0.1)

def ultraCalibrate(direction):
    TURN_SPEED = 750
    U_OFFSET = 10
    ultraTurn(0)
    front = ultraSensor.value()
    print("front is %d" % front)
    ultraTurn(-95)
    right = ultraSensor.value()
    print("right is %d" % right)
    ultraTurn(95)
    left = ultraSensor.value()
    print("left is %d" % left)
    ultraTurn(190)
    ultraTurn(0)

    while (ultraSensor.value() >= right - OFFSET && ultraSensor.value <= right + OFFSET ):
        rightMotor.run_forever(speed_sp = -direction * TURN_SPEED)
        leftMotor.run_forever(speed_sp = direction * TURN_SPEED)



# ? direction is probably the opposite
clockFudge = gyroCalibrate(1)
antiClockFudge = gyroCalibrate(-1)



# Multiply the value returned by this function by
# the angle you actually want to turn

# If direction is 1 it goes one way and if it is -1
# it goes the other way
def gyroCalibrate(direction):
    # When this function is run, the direction the robot is
    # currently facing will be used as the reference point (front)
    # for the robot
    front = gyroSensor.value()
    print("%d" % front)

    turn(90 * direction)
    val1 = gyroSensor.value()
    turn(90 * direction)
    val2 = gyroSensor.value() - val1
    turn(90 * direction)
    val3 = gyroSensor.value() - val2
    turn(90 * direction)
    val4 = gyroSensor.value() - val3

    avgTurnConstant = (val1 + val2 + val3 + val4) / 4

    fudgedDegree = avgTurnConstant / 90
    return fudgedDegree
