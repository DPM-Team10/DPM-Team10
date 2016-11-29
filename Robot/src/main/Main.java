package main;

import java.util.*;
import lejos.hardware.Button;
import lejos.hardware.Sound;

/**
 * The main class that manages most of the decision making aspects of the robot.
 *
 * @author Scott Sewell
 */
public class Main
{
    // the duration of the match in seconds
    private static final long MATCH_DURATION = 5 * 60;
    // how far the robot sees while localizing in cm
    private static final float LOCALIZATION_DISTANCE = 45;
    // number of blocks the the robot will try to stack before dropping them off
    private static final int BLOCK_STACK_SIZE = 1;
    // the distance in cm ahead of the robot in which obstacles are seen 
    private static final float OBSTACLE_DISTANCE = 11f;
    // how much error is allowed between the odometer position and destination position.
    private static final float POSITION_TOLERANCE = 4.0f;
    // once this much time in seconds is left the robot will try to return the the start.
    private static final float END_TIME = 20f;
    
    private StartParameters m_startParams;
    private Board m_board;
    private Odometer m_odometer;
    private OdometryCorrection m_odoCorrection;
    private UltrasonicPoller m_usMain;
    private UltrasonicPoller m_usUpper;
    private Driver m_driver;
    private HeldBlockManager m_blockManager;
    private Display m_display;
    
    private long m_startTime;
    
    // search algorithm
    private static final float OFFSET = 30; // to give enough space for the robot to turn around
    private float m_usPreviousDistance = 0;
    private boolean m_usHasStartedCollectingData = false;
    private float m_discontinuityStartAngle = 0;
    private float m_discontinuityEndAngle = 0;
    private boolean  m_discontinuitySpotted = false;
    
    
    /**
     * Launches the main program.
     */
    public static void main(String[] args)
    {
        Main main = new Main();
        main.launch();
    }

    /**
     * Gets starting info, runs threads, and begins the main logic loop.
     */
    private void launch()
    {
        // initialize
        m_usMain = new UltrasonicPoller(Robot.ULTRASOUND_MAIN);
        m_usUpper = new UltrasonicPoller(Robot.ULTRASOUND_UPPER);
        m_odometer = new Odometer();
        m_odoCorrection = new OdometryCorrection(m_odometer);
        m_driver = new Driver(m_odometer);
        m_blockManager = new HeldBlockManager();
        m_display = new Display(m_odometer);

        // choose whether to use wifi or test parameters.
        m_startParams = new StartParameters();
        if (m_display.getMenuResponse("Use Wifi", "Use Test Data") == Button.ID_LEFT)
        {
            // wait to progress until start information is received via wifi
            while (!m_startParams.hasRecievedData())
            {
                m_startParams.getWifiData();
                Utils.sleep(100);
            }
        }
        else
        {
            m_startParams.useTestData();
        }
        
        Robot.SCREEN.clear();
        
        // record the starting time
        m_startTime = System.currentTimeMillis();

        // get the board
        m_board = m_startParams.getBoard();

        // start threads
        m_usMain.start();
        m_usUpper.start();
        m_odometer.start();
        m_display.start();

        // localize
        localize(false);

        // start odometry correction now that localization is done
        m_odoCorrection.start();

        // initialize the claw
        m_blockManager.initializeClaw();

        List<Vector2> searchPoints = new ArrayList<Vector2>();
        searchPoints.add(new Vector2(1.5f, 1.5f).scale(Board.TILE_SIZE));
        searchPoints.add(new Vector2(3.5f, 1.5f).scale(Board.TILE_SIZE));
        searchPoints.add(new Vector2(5.5f, 1.5f).scale(Board.TILE_SIZE));
        searchPoints.add(new Vector2(7.5f, 1.5f).scale(Board.TILE_SIZE));

        searchPoints.add(new Vector2(1.5f, 3.5f).scale(Board.TILE_SIZE));
        searchPoints.add(new Vector2(1.5f, 5.5f).scale(Board.TILE_SIZE));
        searchPoints.add(new Vector2(1.5f, 7.5f).scale(Board.TILE_SIZE));
        
        searchPoints.add(new Vector2(2.5f, 8.5f).scale(Board.TILE_SIZE));
        searchPoints.add(new Vector2(4.5f, 8.5f).scale(Board.TILE_SIZE));
        searchPoints.add(new Vector2(6.5f, 8.5f).scale(Board.TILE_SIZE));
        searchPoints.add(new Vector2(8.5f, 8.5f).scale(Board.TILE_SIZE));

        searchPoints.add(new Vector2(8.5f, 2.5f).scale(Board.TILE_SIZE));
        searchPoints.add(new Vector2(8.5f, 4.5f).scale(Board.TILE_SIZE));
        searchPoints.add(new Vector2(8.5f, 6.5f).scale(Board.TILE_SIZE));

        
        searchPoints.add(new Vector2(4f, 4f).scale(Board.TILE_SIZE));
        searchPoints.add(new Vector2(6f, 6f).scale(Board.TILE_SIZE));
        searchPoints.add(new Vector2(4f, 6f).scale(Board.TILE_SIZE));
        searchPoints.add(new Vector2(6f, 4f).scale(Board.TILE_SIZE));
        
        
        List<Vector2> validPoints = new ArrayList<Vector2>();
        for (Vector2 v : searchPoints)
        {
            if (!m_board.inEnemyZone(v) && !m_board.inTeamZone(v) && m_board.inBounds(v))
            {
                validPoints.add(v);
            }
        }
        searchPoints = validPoints;
        
        // main logic loop
        while (getTimeRemaining() > END_TIME)
        {
            // search for blocks until we are facing a probably block
            boolean facingBlock = false;
            Vector2 lastSearchPosition = Vector2.zero();
            while (!facingBlock)
            {
                Vector2 searchPoint = Utils.getNearestPoint(m_odometer.getPosition(), searchPoints);
                
                if (moveWhileAvoiding(searchPoint, POSITION_TOLERANCE, Vector2.distance(lastSearchPosition, m_odometer.getPosition()) > 10))
                {
                    facingBlock = blockSearch(m_odometer.getTheta(), 120);
                    lastSearchPosition = m_odometer.getPosition();
                }
                
                if (!facingBlock)
                {
                    // if we reached a search point, do two searches for blocks
                    Vector2 direction = m_board.getBoardCenter().subtract(m_odometer.getPosition());
                    facingBlock = blockSearch(direction.angle(), 220);
                }
                
                // if we are near the search point we were going to, remove it so we go to a new one next
                if (Vector2.distance(m_odometer.getPosition(), searchPoint) < Board.TILE_SIZE)
                {
                    searchPoints.remove(searchPoint);
                }
                
                if (getTimeRemaining() < END_TIME) { break; }
            }
            
            if (getTimeRemaining() < END_TIME) { break; }
            
            // identify the object in front of the robot
            float blockDistance = m_usMain.getFilteredDistance() + Robot.US_MAIN_OFFSET.getX();
            if (blockDistance < Robot.RADIUS + 20)
            {
                // identify the block in front of the robot
                m_driver.turn(-90, Robot.ROTATE_SPEED, true);
                boolean isBlueBlock = m_usUpper.getFilteredDistance() + Robot.US_UPPER_OFFSET.getY() > blockDistance + 10;
                m_driver.turn(90, Robot.ROTATE_SPEED, true);
                
                // if a blue block, grab hold of it
                if (isBlueBlock)
                {
                    Sound.beepSequenceUp();
                    m_driver.goForward(blockDistance - Robot.US_MAIN_OFFSET.getX(), true);

                    // try to center the block
                    m_driver.turn(25, Robot.ROTATE_SPEED, true);
                    m_driver.turn(-50, Robot.ROTATE_SPEED, true);
                    m_driver.turn(21, Robot.ROTATE_SPEED, true);
                    
                    m_blockManager.captureBlock();
                }
            }
            
            if (getTimeRemaining() < END_TIME) { break; }

            // drop off any held blocks once we have enough
            if (m_blockManager.getBlockCount() >= BLOCK_STACK_SIZE)
            {
                // move to the appropriate zone
                moveWhileAvoiding(m_board.getTeamZoneCenter(), POSITION_TOLERANCE, false);
                
                // back up so blocks are placed more in the center
                m_driver.goForward(-8, true);
                
                // drop off the held blocks
                m_blockManager.releaseBlock();
                
                // back up so the dropped stack is not knocked over
                m_driver.goForward(-20, true);
            }
        }
        
        // we must move back to the start corner before the end of the match
        moveWhileAvoiding(m_board.getStartPos(), POSITION_TOLERANCE, false);
        
        // finish
        System.exit(0);
    }

    /**
     * Attempts to set the odometer's angle to match the board's coordinates by
     * rotating near a board corner. Uses the ultrasonic sensor to determine
     * angles at which the walls are seen.
     * 
     * @param moveToOrigin 
     *            if true moves the robot to the line intersection nearest to
     *            the corner after calculating its position.
     */
    private void localize(boolean moveToOrigin)
    {
        m_odometer.setTheta(0);
        m_odometer.setPosition(Vector2.zero());

        // start the robot turning one revolution and record the seen distances
        // along with the angles they were captured at
        List<Float> orientations = new ArrayList<Float>();
        List<Float> distances = new ArrayList<Float>();
        m_driver.turn(360, Robot.LOCALIZATION_SPEED, false);
        while (m_driver.isTravelling())
        {
            orientations.add(Utils.normalizeAngle(m_odometer.getTheta() + 90));
            distances.add(m_usUpper.getFilteredDistance() + Robot.US_UPPER_OFFSET.getY());
            Utils.sleep(UltrasonicPoller.UPDATE_PERIOD * 2);
        }
        
        // find all the angles that correspond to when the distance rises above
        // LOCALIZATION_DISTANCE and when it lowers below LOCALIZATION_DISTANCE
        List<Float> risingAngles = new ArrayList<Float>();
        List<Float> fallingAngles = new ArrayList<Float>();
        for (int i = 0; i < orientations.size(); i++)
        {
            float dist = distances.get(i);
            float nextDist = distances.get((i + 1) % distances.size());
            
            if (dist < LOCALIZATION_DISTANCE && nextDist > LOCALIZATION_DISTANCE)
            {
                risingAngles.add(orientations.get(i));
            }
            
            if (dist > LOCALIZATION_DISTANCE && nextDist < LOCALIZATION_DISTANCE)
            {
                fallingAngles.add(orientations.get(i));
            }
        }

        // determine which falling and rising edge angles correspond to the wall
        // as to filter out any blocks new the start point. We know that rising
        // falling angle pair with the largest angle between them is the pair
        // that belong to the wall.
        float largestBearing = Float.MIN_VALUE;
        float angle = 0;
        for (float risingAng : risingAngles)
        {
            for (float fallingAng : fallingAngles)
            {
                float bearing = Math.abs(Utils.toBearing(risingAng - fallingAng));
                if (bearing > largestBearing)
                {
                    largestBearing = bearing;
                    angle = 315 - (bearing / 2) - risingAng;
                }
            }
        }
        
        // account for the starting corner the robot is in
        float cornerAngOffset = 90 * m_startParams.getStartCorner();

        // set odometer angle accounting for start corner
        m_odometer.setTheta(angle + cornerAngOffset);
        
        Vector2 startPos = new Vector2(
                distances.get(Utils.closestIndex(Utils.normalizeAngle(180 - angle), orientations)) - Board.TILE_SIZE,
                Board.TILE_SIZE - distances.get(Utils.closestIndex(Utils.normalizeAngle(90 - angle), orientations))
                );
        
        m_odometer.setPosition(m_board.getStartPos().add(startPos.rotate(cornerAngOffset)));
        
        Sound.beepSequenceUp();
        
        // if applicable, move to the nearest line intersection
        if (moveToOrigin)
        {
            m_driver.travelTo(Board.getNearestIntersection(m_odometer.getPosition()), true);
            m_driver.turnTo(cornerAngOffset - 90, Robot.ROTATE_SPEED, true);
        }
    }

    /**
     * Moves the robot to a position while avoiding obstacles on the way.
     * 
     * @param destination
     *            the destination point.
     * @param positionTolerance
     *            the distance under which the robot must be to the given
     *            position before returning.
     * @returns true if the robot has stopped before an obstacle.
     */
    private boolean moveWhileAvoiding(Vector2 destination, float positionTolerance, boolean stopFacingObstacle)
    {
        List<Vector2> obstacles = new ArrayList<Vector2>();
        List<Vector2> path = m_board.findPath(m_odometer.getPosition(), destination, obstacles);
        
        // travel the entire path
        while (path.size() > 0)
        {
            if (Vector2.distance(m_odometer.getPosition(), path.get(0)) > positionTolerance)
            {
                Utils.writeDebug("Moving to " + path.get(0).toString());
                if (moveUntilObstacle(path.get(0)))
                {
                    Vector2 obstaclePos = m_odometer.toWorldSpace(Vector2.unitX().scale(Robot.RADIUS + OBSTACLE_DISTANCE + 4));

                    // if there is an obstacle blocking the endpoint give up
                    if (stopFacingObstacle || Vector2.distance(obstaclePos, destination) < Board.TILE_SIZE)
                    { 
                        return true;
                    }
                    
                    obstacles.add(obstaclePos);
                    path = m_board.findPath(m_odometer.getPosition(), destination, obstacles);
                }
            }
            else
            {
                Utils.writeDebug("Arrived at " + path.get(0).toString());
                // remove waypoints we have arrived at
                path.remove(0);
            }
        }
        return false;
    }

    /**
     * Tries to move the robot to a given position, but stops if an obstacle is
     * encountered.
     * 
     * @param destination
     *            the destination point.
     * @returns true if the robot has stopped before an obstacle.
     */
    private boolean moveUntilObstacle(Vector2 destination)
    {
        m_driver.turnTo(Vector2.subtract(destination, m_odometer.getPosition()).angle(), Robot.ROTATE_SPEED, true);
        Utils.sleep(50);
        m_driver.goForward(Vector2.distance(m_odometer.getPosition(), destination), false);
        
        while (m_driver.isTravelling() && (
                    m_usMain.getFilteredDistance() + Robot.US_MAIN_OFFSET.getX() > Robot.RADIUS + OBSTACLE_DISTANCE ||
                    Vector2.distance(destination, m_odometer.getPosition()) < OBSTACLE_DISTANCE || 
                    m_board.inTeamZone(m_odometer.toWorldSpace(Vector2.unitX().scale(m_usMain.getFilteredDistance() + Robot.US_MAIN_OFFSET.getX()))))
                ) {}
        if (m_driver.isTravelling())
        {
            m_driver.stop();
            return true;
        }
        return false;
    }

    /**
     * Gets the time remaining in the match.
     * 
     * @return the time in seconds.
     */
    private float getTimeRemaining()
    {
        return MATCH_DURATION - ((System.currentTimeMillis() - m_startTime) / 1000f);
    }
    

    /**
     * Sweeps an arc and moves to the first object seen.
     * 
     * @param searchDirection
     *            the direction in degrees that the robot will center the sweep
     *            around.
     * @param searchWidth
     *            how many degrees the sweep will cover.
     * @return true if the robot has approached a block.
     */
    private boolean blockSearch(float searchDirection, float searchWidth)
    {
        // turn to face the start angle of the sweep
        float startAngle = Utils.normalizeAngle(searchDirection - (searchWidth / 2));
        m_driver.turnTo(startAngle, Robot.ROTATE_SPEED, true);

        // start turning the robot
        m_driver.turn(searchWidth, Robot.SEARCH_SPEED, false);
        
        boolean foundBlock = false;
        while (m_driver.isTravelling())
        {
            while (m_driver.isTravelling() && m_usMain.getFilteredDistance() + Robot.US_MAIN_OFFSET.getX() > 60) {}
            if (m_driver.isTravelling())
            {
                Sound.beep();
                Utils.writeDebug("Ang1:" + m_odometer.getTheta());
                List<Float> distances = new ArrayList<Float>();
                List<Float> angles = new ArrayList<Float>();
                Utils.sleep(200);
                while (m_driver.isTravelling() && m_usMain.getFilteredDistance() + Robot.US_MAIN_OFFSET.getX() < 60)
                {
                    distances.add(m_usMain.getFilteredDistance() + Robot.US_MAIN_OFFSET.getX());
                    angles.add(m_odometer.getTheta());
                    Utils.sleep(UltrasonicPoller.UPDATE_PERIOD);
                }
                if (m_driver.isTravelling() && angles.size() > 2)
                {
                    if (Math.abs(Utils.toBearing(angles.get(0) - angles.get(angles.size() - 1))) > 6)
                    {
                        Sound.beep();
                        int middleIndex = distances.size() / 2;
                        Vector2 blockPos = m_odometer.getPosition().add(Vector2.fromPolar(angles.get(middleIndex), distances.get(middleIndex) + 5));
                        
                        if (m_board.inBounds(blockPos) && !m_board.inEnemyZone(blockPos) && !m_board.inTeamZone(blockPos))
                        {
                            foundBlock = true;
                            m_driver.stop();
                            Utils.sleep(50);
                            m_driver.travelTo(m_odometer.getPosition().add(Vector2.fromPolar(angles.get(middleIndex) - 7.75f, Math.max(distances.get(middleIndex) - (Robot.RADIUS + 7.5f), 1f))), true);
                        }
                        Utils.writeDebug("Ang2:" + m_odometer.getTheta());
                    }
                }
                if (!foundBlock)
                {
                    Utils.sleep(65);
                }
            }
        }
        return foundBlock;
    }
}