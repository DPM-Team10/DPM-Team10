package main;

import lejos.hardware.motor.EV3LargeRegulatedMotor;

/**
 * Maneuvers the claw and keeps track of the number of blocks being held.
 * 
 * @author
 */
public class HeldBlockManager
{
    private final int MOTOR_CLAW_STALL_THRESHOLD = 4;
    private final int MOTOR_PULLEY_STALL_THRESHOLD = 7;

    private EV3LargeRegulatedMotor m_clawMotor;
    private EV3LargeRegulatedMotor m_pulleyMotor;
    private int m_blocksHeld;

    /**
     * Constructor.
     */
    public HeldBlockManager()
    {
        m_blocksHeld = 0;
        m_clawMotor = Robot.MOTOR_CLAW;
        m_pulleyMotor = Robot.MOTOR_PULLEY;

        m_clawMotor.setAcceleration(Robot.CLAW_ACCELERATION);
        m_pulleyMotor.setAcceleration(Robot.CLAW_ACCELERATION);
        m_clawMotor.setSpeed(Robot.CLAW_SPEED);
        m_pulleyMotor.setSpeed(Robot.PULLEY_SPEED);
    }

    /**
     * Initializes the claw system by raising the pulley and closing the claw.
     */
    public void initializeClaw()
    {
        raisePulley();
        m_clawMotor.rotateTo(90);
        //m_clawMotor.waitComplete();
    }
    
    /**
     * Raises the pulley until reaches the claw reaches the top position.
     */
    public void raisePulley()
    {
        rotateUntilStall(m_pulleyMotor, false, MOTOR_PULLEY_STALL_THRESHOLD);
    }

    /**
     * Captures a block positioned below the claw. This assumes the claw is
     * already in a raised position.
     */
    public void captureBlock()
    {
        // open claw to release all blocks that the claw is holding
        m_clawMotor.rotateTo(0);
        m_clawMotor.waitComplete();

        // lower claw completely
        m_pulleyMotor.rotateTo(0);
        m_pulleyMotor.waitComplete();

        // close claw to capture the lowest block
        rotateUntilStall(m_clawMotor, true, MOTOR_CLAW_STALL_THRESHOLD);

        // rise claw
        raisePulley();

        m_blocksHeld++;
    }

    /**
     * Drops all held blocks.
     */
    public void releaseBlock()
    {
        // lower claw completely
        m_pulleyMotor.rotateTo(0);
        m_pulleyMotor.waitComplete();
        
        // open claw to release all blocks that the claw is holding
        m_clawMotor.rotateTo(0);
        m_clawMotor.waitComplete();

        // rise claw
        raisePulley();

        m_blocksHeld = 0;
    }

    /**
     * Rotates a motor until it stops rotating because it is blocked.
     * 
     * @param motor
     *            the motor to rotate.
     * @param forward
     *            indicates the direction the motor is rotated in.
     * @param threshold
     *            the amount the motor must move between checks to be considered
     *            not stalled.
     */
    public static void rotateUntilStall(EV3LargeRegulatedMotor motor, boolean forward, int threshold)
    {
        if (forward)
        {
            motor.forward();
        }
        else
        {
            motor.backward();
        }
            
        int lastTacho = 999999;
        boolean stall = false;
        while (!stall)
        {
            Utils.sleep(100);
            int currentTacho = motor.getTachoCount();
            if (Math.abs(currentTacho - lastTacho) < threshold)
            {
                stall = true;
            }
            lastTacho = currentTacho;
        }
        motor.stop();
    }

    /**
     * @return the number of blocks held by the claw.
     */
    public int getBlockCount()
    {
        return m_blocksHeld;
    }
}
