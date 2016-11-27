package main;

import lejos.hardware.motor.EV3LargeRegulatedMotor;

/**
 * Contains the methods responsible for moving the robot.
 * 
 * @author Scott Sewell
 */
public class Driver
{
    private EV3LargeRegulatedMotor m_leftMotor;
    private EV3LargeRegulatedMotor m_rightMotor;
    private Odometer m_odometer;
    
    /**
     * Constructor.
     */
    public Driver(Odometer odometer)
    {
        m_odometer = odometer;
        
        m_leftMotor = Robot.MOTOR_LEFT;
        m_rightMotor = Robot.MOTOR_RIGHT;

        // reset the motors
        for (EV3LargeRegulatedMotor motor : new EV3LargeRegulatedMotor[] { m_leftMotor, m_rightMotor })
        {
            motor.stop();
            motor.setAcceleration(Robot.ACCELERATION);
        }
        //m_leftMotor.synchronizeWith(new EV3LargeRegulatedMotor[] { m_rightMotor });
    }

    /**
     * Begins moving the robot a certain distance. The calling thread is not
     * blocked.
     * 
     * @param distance
     *            the distance that is traveled by the robot in cm.
     * @param blockThread
     *            If true this call returns only after the turn is complete.
     */
    public void goForward(float distance, boolean blockThread)
    {
        // set motor speed
        m_leftMotor.setSpeed(Robot.MOVE_SPEED);
        m_rightMotor.setSpeed(Robot.MOVE_SPEED);

        // rotate motors
        //m_leftMotor.startSynchronization();
        m_leftMotor.rotate(convertDistance(Robot.WHEEL_RADIUS, distance), true);
        m_rightMotor.rotate(convertDistance(Robot.WHEEL_RADIUS, distance), !blockThread);
        //m_leftMotor.endSynchronization();
    }

    /**
     * Begins moving the robot towards a specific position. The calling thread
     * is not blocked.
     * 
     * @param destination
     *            The position to move towards on the board in cm.
     * @param blockThread
     *            If true this call returns only after the turn is complete.
     */
    public void travelTo(Vector2 destination, boolean blockThread)
    {
        Vector2 currentLocation = m_odometer.getPosition();

        // turn to face new waypoint
        turnTo(Vector2.subtract(destination, currentLocation).angle(), Robot.ROTATE_SPEED, blockThread);

        // a short pause to allow time for distance sensor to get some values
        Utils.sleep(350);

        // calculate the distance to the new location
        goForward(Vector2.distance(currentLocation, destination), blockThread);
    }

    /**
     * Turns the robot to face the specified angle. Blocks the calling thread
     * until the action is complete.
     * 
     * @param targetAngle
     *            the world space angle to face in degrees.
     * @param speed
     *            The speed of rotation.
     * @param blockThread
     *            If true this call returns only after the turn is complete.
     */
    public void turnTo(float targetAngle, int speed, boolean blockThread)
    {
        float bearing = Utils.toBearing(targetAngle - m_odometer.getTheta());

        // set rotating speeds
        m_leftMotor.setSpeed(speed);
        m_rightMotor.setSpeed(speed);

        // rotate the motors to complete the motion
        //m_leftMotor.startSynchronization();
        m_leftMotor.rotate(-convertAngle(Robot.WHEEL_RADIUS, bearing), true);
        m_rightMotor.rotate(convertAngle(Robot.WHEEL_RADIUS, bearing), !blockThread);
        //m_leftMotor.endSynchronization();
    }

    /**
     * Turns the robot a specific amount relative to the current facing. The
     * thread is blocked until the action is complete.
     * 
     * @param angle
     *            The angle in degrees the robot will turn.
     * @param speed
     *            The speed of rotation.
     * @param blockThread
     *            If true this call returns only after the turn is complete.
     */
    public void turn(float angle, int speed, boolean blockThread)
    {
        // set rotating speeds
        m_leftMotor.setSpeed(speed);
        m_rightMotor.setSpeed(speed);

        // rotate the motors to complete the motion
        //m_leftMotor.startSynchronization();
        m_leftMotor.rotate(-convertAngle(Robot.WHEEL_RADIUS, angle), true);
        m_rightMotor.rotate(convertAngle(Robot.WHEEL_RADIUS, angle), !blockThread);
        //m_leftMotor.endSynchronization();
    }

    /**
     * Immediately stops the robot's motion.
     */
    public void stop()
    {
        m_leftMotor.stop(true);
        m_rightMotor.stop(false);
    }

    /**
     * @return true if the robot is still attempting to reach a destination.
     */
    public boolean isTravelling()
    {
        return m_leftMotor.isMoving() || m_rightMotor.isMoving();
    }

    /**
     * Calculates the number of motor rotations needed to travel a specific
     * distance
     * 
     * @param radius
     *            the radius of the wheel attached to the motor in cm.
     * @param distance
     *            how far to move in cm.
     * @return The amount a motor must rotate to travel the given distance.
     */
    private int convertDistance(float radius, float distance)
    {
        return (int) ((180 * distance) / (Math.PI * radius));
    }

    /**
     * Calculates the number of motor rotations needed to rotate the robot a
     * specified angle.
     * 
     * @param radius
     *            the radius of the wheel attached to the motor in cm.
     * @param angle
     *            the angle to turn in degrees.
     * @return The amount the motor must rotate to turn the robot the given
     *         angle.
     */
    private int convertAngle(float radius, float angle)
    {
        return convertDistance(radius, (float) Math.PI * Robot.WHEEL_TRACK * angle / 360f);
    }
}
