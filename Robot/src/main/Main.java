package main;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
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
    private static final float OBSTACLE_DISTANCE = 7.5f;
    // the distance in cm the robot moves to either side of an obstacle when trying to avoid it
    private static final float AVOID_DISTANCE = 40;
    // how much error is allowed between the odometer position and destination position.
    private static final float POSITION_TOLERANCE = 2.0f;
    
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
        localize(true);

        // start odometry correction now that localization is done
        m_odoCorrection.start();

        // initialize the claw
        m_blockManager.initializeClaw();

        // main logic loop
        while (getTimeRemaining() > 20)
        {
            // search for blocks until we are facing a probably block
            while (!searchForBlocks(45, 90))
            {
                
            }
            
            // if there is an object in front of the robot, identify it
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
                    m_blockManager.captureBlock();
                }
            }

            // drop off any held blocks once we have enough
            if (m_blockManager.getBlockCount() >= BLOCK_STACK_SIZE)
            {
                // move to the appropriate zone
                moveWhileAvoiding(m_board.getTeamZoneCenter(), POSITION_TOLERANCE);
                m_blockManager.releaseBlock();
            }
        }
        
        // we must move back to the start corner before the end of the match
        moveWhileAvoiding(m_board.getStartPos(), POSITION_TOLERANCE);
        
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
     * @param position
     *            the destination point.
     * @param positionTolerance
     *            the distance under which the robot must be to the given
     *            position before returning.
     */
    private void moveWhileAvoiding(Vector2 position, float positionTolerance)
    {
        while (Vector2.distance(m_odometer.getPosition(), position) > positionTolerance)
        {
            if (moveUntilObstacle(position))
            {
                avoidObstacle();
            }
        }
    }

    /**
     * Tries to move the robot to a given position, but stops if an obstacle is
     * encountered.
     * 
     * @param position
     *            the destination point.
     * @returns true if the robot has stopped before an obstacle.
     */
    private boolean moveUntilObstacle(Vector2 position)
    {
        m_driver.turnTo(Vector2.subtract(position, m_odometer.getPosition()).angle(), Robot.ROTATE_SPEED, true);
        Utils.sleep(50);
        m_driver.goForward(Vector2.distance(m_odometer.getPosition(), position), false);
        
        while ( m_driver.isTravelling() &&
                m_usMain.getFilteredDistance() + Robot.US_MAIN_OFFSET.getX() > Robot.RADIUS + OBSTACLE_DISTANCE
                ) {}
        if (m_driver.isTravelling() && Vector2.distance(position, m_odometer.getPosition()) > OBSTACLE_DISTANCE)
        {
            m_driver.stop();
            return true;
        }
        return false;
    }

    /**
     * Moves the robot some distance to the left or right, depending on which
     * direction is clearer of other obstacles.
     */
    private void avoidObstacle()
    {
        // check if there is no obstacle nearby on the left using the left ultrasound sensor
        // also check if the avoidance detour will cross into a invalid position
        Vector2 leftAvoidWaypoint1 = m_odometer.toWorldSpace(new Vector2(0, AVOID_DISTANCE));
        Vector2 leftAvoidWaypoint2 = m_odometer.toWorldSpace(new Vector2(AVOID_DISTANCE, AVOID_DISTANCE));
        Vector2 rightAvoidWaypoint1 = m_odometer.toWorldSpace(new Vector2(0, -AVOID_DISTANCE));
        Vector2 rightAvoidWaypoint2 = m_odometer.toWorldSpace(new Vector2(AVOID_DISTANCE, -AVOID_DISTANCE));

        Vector2 detour1;
        Vector2 detour2;
        
        float leftDistance = m_usUpper.getFilteredDistance() + Robot.US_UPPER_OFFSET.getY();
        if (leftDistance > Robot.RADIUS + AVOID_DISTANCE && checkValidity(leftAvoidWaypoint1))
        {
            detour1 = leftAvoidWaypoint1;
            detour2 = leftAvoidWaypoint2;
        }
        else
        {
            // if the left way around is invalid, try looking right at check if it is clear
            m_driver.turn(-90, Robot.ROTATE_SPEED, true);
            detour1 = rightAvoidWaypoint1;
            detour2 = rightAvoidWaypoint2;

            float rightDistance = m_usMain.getFilteredDistance() + Robot.US_MAIN_OFFSET.getX();
            // if both left and right direction are not clear, pick the better bet
            if (!(rightDistance > Robot.RADIUS + AVOID_DISTANCE && checkValidity(rightAvoidWaypoint1)) && leftDistance > rightDistance)
            {
                detour1 = leftAvoidWaypoint1;
                detour2 = leftAvoidWaypoint2;
            }
        }
        
        if (!moveUntilObstacle(detour1))
        {            
            moveUntilObstacle(detour2);
        }
    }

    /**
     * Determines if a point on the board in a valid place for the robot to go.
     * 
     * @param destination
     *            the world space position to check.
     * @return true if the destination point doesn't overlap any invalid
     *         regions.
     */
    private boolean checkValidity(Vector2 destination)
    {
        return m_board.inBounds(destination) && !m_board.inEnemyZone(destination);
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
     * Searches for blocks. Sweeps from current angle to +90 degrees, collects
     * data into a Map then analyzes that data and moves the robot accordingly.
     * 
     * @param searchDirection
     *            the direction in degrees that the robot will center the sweep
     *            around.
     * @param searchWidth
     *            how many degrees the sweep will cover.
     * @return true if the robot has approached a block.
     */
    private boolean searchForBlocks(float searchDirection, float searchWidth)
    {
        resetSearchProperties();
        
        // turn to face the start angle of the sweep
        float startAngle = Utils.normalizeAngle(searchDirection - (searchWidth / 2));
        m_driver.turnTo(startAngle, Robot.ROTATE_SPEED, true);

        // start turning the robot
        m_driver.turn(searchWidth, Robot.SEARCH_SPEED, false);
        
        // take samples while turning
        Map<Float,Float> angleDistanceMap = new HashMap<Float,Float>();
        while (m_driver.isTravelling())
        {
            angleDistanceMap.put(m_odometer.getTheta(), getDistanceMain());
            // prevents grabbing samples than they can be generated
            Utils.sleep(UltrasonicPoller.UPDATE_PERIOD);
        }

        // sort the map by increasing angle
        Map<Float,Float> sortedData = new TreeMap<Float,Float>(angleDistanceMap);
        writeToFile(sortedData, "sorted.txt");

        // sanitize data to remove incorrect discontinuities
        Map<Float,Float> sanitizedData = sanitize(sortedData);
        writeToFile(sanitizedData, "sanitized.txt");

        // set the iterator to the first sanitized data point
        Iterator<Map.Entry<Float,Float>> entries = sanitizedData.entrySet().iterator();
        Map.Entry<Float,Float> entry = entries.next();
        float previousDistance = entry.getValue();

        // check for discontinuities
        Map<Float,Float> discontinuitiesMap = new HashMap<Float,Float>();
        
        while (entries.hasNext())
        {
            entry = entries.next();
            float currentAngle = entry.getKey();
            float currentDistance = entry.getValue();

            // if there is a discontinuity increase the discontinuity counter
            // and store the data point on another map
            if (Math.abs(currentDistance - previousDistance) > 10)
            {
                discontinuitiesMap.put(currentAngle, currentDistance);
            }

            // updating for next iteration
            previousDistance = currentDistance;
        }

        // sort the discontinuities map
        Map<Float,Float> sortedDiscontinuities = new TreeMap<Float,Float>(discontinuitiesMap);

        // take action based on the number of possible blocks
        if (sortedDiscontinuities.size() == 0)
        {
            Sound.buzz();
            noDiscontinuities(sortedData);
        }
        else if (sortedDiscontinuities.size() == 1)
        {
            Sound.beep();
            writeToFile(sortedDiscontinuities, "disc1.txt");
            oneDiscontinuity(sortedData, sortedDiscontinuities);
            return true;
        }
        else
        {
            Sound.twoBeeps();
            writeToFile(sortedDiscontinuities, "disc2.txt");

            if (moreThanOneBlock(sortedDiscontinuities))
            {
                Sound.beepSequenceUp();
            }
            else
            {
                manyDiscontinuities(sortedDiscontinuities);
                return true;
            }
        }
        return false;
    }
    
    /**
     * This method resets relevant search properties. 
     * 
     */
    private void resetSearchProperties()
    {
        m_usPreviousDistance = 0;
        m_usHasStartedCollectingData = false;
        m_discontinuityStartAngle = 0;
        m_discontinuityEndAngle = 0;
        m_discontinuitySpotted = false;
    }

    /**
     * This method removes any false positives from the data.
     * 
     * @param rawData
     *            angles to distance map containing search data.
     * @return the cleaned up map containing the angles and the distances
     *         recorded by the ultrasonic sensor.
     */
    private Map<Float,Float> sanitize(Map<Float,Float> rawData)
    {
        float currentAngle;
        float currentDistance;
        float previousAngle;
        float previousDistance;
        float angleGap;
        float minAngleGap = 8;
        float absDistanceGap;
        float minDistanceGap = 0;
        Iterator<Map.Entry<Float,Float>> entries;
        Map.Entry<Float,Float> entry;

        // in case we can't modify a parameter
        // creating new reference
        // Map<Float, Float> localRawData = new TreeMap<Float, Float>(rawData);

        // setting the discontinuities
        entries = rawData.entrySet().iterator();

        // set previousAngle and previousDistance to first data point
        entry = entries.next();
        previousAngle = entry.getKey();
        previousDistance = entry.getValue();

        // hardcoding 55 for the first value
        rawData.put(entry.getKey(), 55f);
        previousDistance = 55f;

        // set m_discontinuityStartAngle to first angle
        m_discontinuityStartAngle = previousAngle;

        // check for discontinuities
        while (entries.hasNext())
        {
            entry = entries.next();
            currentAngle = entry.getKey();
            currentDistance = entry.getValue();

            absDistanceGap = Math.abs(currentDistance - previousDistance);

            if (absDistanceGap > minDistanceGap)
            {
                // so here we know the sensor detected something
                // let's see what angle does it span
                // if it's less than 8 degress, we change the data
                // meaning at currentAngle, change distance

                // if 55 prev and now 30
                // the discontinuity was detected at THIS angle
                // so set m_start to this angle, ie currentAngle
                if (!m_discontinuitySpotted) // if discon has been spotted
                {
                    m_discontinuityStartAngle = currentAngle;
                    m_discontinuitySpotted = true;

                }
                else
                {
                    m_discontinuityEndAngle = currentAngle;
                    m_discontinuitySpotted = false;

                    angleGap = Math.abs(m_discontinuityEndAngle - m_discontinuityStartAngle);

                    writeDebug("Angle gap is: " + angleGap);

                    if (angleGap < minAngleGap) // if we have a discontinuity
                                                // lower than this, delete
                    {
                        writeDebug("will delete from: " + m_discontinuityStartAngle + " to: " + m_discontinuityEndAngle);

                        // modifiying this reference
                        filterOutFalsePositives(rawData, m_discontinuityStartAngle, m_discontinuityEndAngle);
                    }
                }
            }

            previousAngle = currentAngle;
            previousDistance = currentDistance;

            // if we are at the end of the data, add a 55 to make sure
            // that we spot a discontinuity in the case there's a block
            // that is detected up until the end of the scan.
            if (!entries.hasNext())
            {
                rawData.put(currentAngle, 55f);
            }
        }
        return rawData;
    }
    
    /**
     * This method checks if any discontinuities is covering more than a certain
     * angle (80 for one block). If it is, it lets the any method calling it
     * know that there is more than one block around the robot.
     * 
     * @param sortedDiscontinuities
     * @return Whether there is more than one block in front of the robot.
     */
    private boolean moreThanOneBlock(Map<Float,Float> sortedDiscontinuities)
    {
        float currentAngle;
        float previousAngle;
        float oneBlockAngle = 80;
        Iterator<Map.Entry<Float,Float>> entries;
        Map.Entry<Float,Float> entry;

        entries = sortedDiscontinuities.entrySet().iterator();

        entry = entries.next();
        previousAngle = entry.getKey();

        while (entries.hasNext())
        {
            entry = entries.next();
            currentAngle = entry.getKey();

            if (Math.abs(currentAngle - previousAngle) > oneBlockAngle)
            {
                writeDebug("Detected two blocks. Abort.");
                return true;
            }

            // updating for next iteration
            previousAngle = currentAngle;
        }
        return false;
    }

    /**
     * Method that iterates through the data from one angle to the other to
     * remove any discontinuities in between.
     * 
     * @param rawData
     * @param discontinuityStartAngle
     * @param discontinuityEndAngle
     */
    private void filterOutFalsePositives(Map<Float,Float> rawData, float discontinuityStartAngle, float discontinuityEndAngle)
    {
        float correctedDistance = 55f;
        float currentAngle;

        for (Map.Entry<Float,Float> entry : rawData.entrySet())
        {
            currentAngle = entry.getKey();

            // do not include endAngle as it is the other discontinuity
            if ((currentAngle >= discontinuityStartAngle) && (currentAngle < discontinuityEndAngle))
            {
                rawData.put(currentAngle, correctedDistance);
            }
        }
    }
    
    /**
     * This method is to be called for analyzing a dataset containing no
     * discontinuities.
     * 
     * @param data
     *            a map containing all the angles and the distances for each
     *            of them.
     */
    private void noDiscontinuities(Map<Float,Float> data)
    {
        float[] meanAngleDistance = getMeanAngleDistance(data);

        float meanAngle = meanAngleDistance[0];
        float meanDistance = meanAngleDistance[1];

        Vector2 destination = Vector2.fromPolar(meanAngle, meanDistance - OFFSET);

        // move robot to mean distance, at mean angle
        //m_odometer.getPosition().add(Vector2.fromPolar(90,5))
        m_driver.travelTo(m_odometer.getPosition().add(destination), true);
    }
    
    /**
     * This method is called when only one discontinuity is detected by the main
     * ultrasonic sensor.
     * 
     * @param data
     *            a map containing all the angles measured and their
     *            corresponding distances.
     * @param discontinuitiesMap
     *            a map containing the only discontinuity found.
     */
    private void oneDiscontinuity(Map<Float,Float> data, Map<Float,Float> discontinuitiesMap)
    {
        // partitioning data in 2, on either side of the single discontinuity
        float dividingAngle = discontinuitiesMap.keySet().iterator().next();
        Map<Float,Float> partitionOne = new HashMap<Float,Float>();
        Map<Float,Float> partitionTwo = new HashMap<Float,Float>();

        for (Map.Entry<Float,Float> entry : data.entrySet())
        {
            if (entry.getKey() < dividingAngle)
            {
                partitionOne.put(entry.getKey(), entry.getValue());
            }
            else
            {
                partitionTwo.put(entry.getKey(), entry.getValue());
            }
        }

        // get the mean angle and mean distance of each partitions
        // first eleement of the array is the angle, second is distance
        float[] meansPartitionOne = getMeanAngleDistance(partitionOne);
        float[] meansPartitionTwo = getMeanAngleDistance(partitionTwo);

        float meanAnglePartitionOne = meansPartitionOne[0];
        float meanDistancePartitionOne = meansPartitionOne[1];
        float meanAnglePartitionTwo = meansPartitionTwo[0];
        float meanDistancePartitionTwo = meansPartitionTwo[1];

        Vector2 destination;

        // selecting the partition with the smaller average for the
        // distance measured
        if (meanDistancePartitionOne < meanDistancePartitionTwo)
        {
            destination = Vector2.fromPolar(meanAnglePartitionOne, meanDistancePartitionOne - OFFSET);
        }
        else
        {
            destination = Vector2.fromPolar(meanAnglePartitionTwo, meanDistancePartitionTwo - OFFSET);
        }

        m_driver.travelTo(m_odometer.getPosition().add(destination), true);
    }
    
    /**
     * This method is called when the main ultrasonic sensor has detected more
     * than one discontinuity. The robot will move towards the first blue block
     * it detects.
     * 
     * @param data
     *            a map that includes all the angles and the distances of the
     *            detected discontinuities.
     */
    private void manyDiscontinuities(Map<Float,Float> data)
    {
        float currentAngle;
        float currentDistance;
        float previousAngle;
        float previousDistance;
        Iterator<Map.Entry<Float,Float>> entries;
        Map.Entry<Float,Float> entry;
        float width;
        ArrayList<Float> objectWidths = new ArrayList<Float>(); // delete later
                                                                // if not used
        float destinationAngle;
        float destinationDistance;

        // An Iterator to iterate through all the discontinuities in the Map
        entries = data.entrySet().iterator();

        // initialize the previous variables with first element on the Map
        entry = entries.next();
        previousAngle = entry.getKey();
        previousDistance = entry.getValue();

        // do while to go through the loop once in case there are only
        // 2 discontinuities in the Map
        do
        {
            entry = entries.next();
            currentAngle = entry.getKey();
            currentDistance = entry.getValue();

            // calculating widths and adding to the arraylist
            width = calculateWidth(previousAngle, previousDistance, currentAngle, currentDistance);
            objectWidths.add(width);

            // TODO need a better width calculator
            // mb get some data and then calculate what could be considered
            // a block and what is more than than.
            // for example, if width exceeds 100, then assume it's two blocks
            // and do something else
            // this could also be just checking the angle instead of the width
            if (width < 120)
            {
                // trying to aim for the middle of the block
                destinationAngle = (currentAngle + previousAngle) / 2.0f;

                // an offset might be needed here to not land on top of the
                // block
                destinationDistance = (currentDistance + previousDistance) / 2.0f;

                // DEBUG
                writeDebug("Planning on traveling: " + destinationDistance);

                // if it detects an object to close to it, abort search and
                // relocate
                if (destinationDistance < 16)
                {
                    Sound.beepSequenceUp();
                    resetSearchProperties();
                    break;
                }
                
                Vector2 destination = Vector2.fromPolar(destinationAngle, destinationDistance - OFFSET);
                
                m_driver.travelTo(m_odometer.getPosition().add(destination), true);
                break;
            }

            // updating the previous variables for next iteration
            previousAngle = currentAngle;
            previousDistance = currentDistance;
        }
        while (entries.hasNext());
    }

    /**
     * This method calculates the width of a perceived object.
     * 
     * @param angleA
     *            angle of first edge of the perceived object.
     * @param distanceA
     *            distance to the first edge of the perceived object.
     * @param angleB
     *            angle of the second edge of the perceived object.
     * @param distanceB
     *            distance to the second edge of the perceived object.
     * @return the calculated width.
     */
    private float calculateWidth(float angleA, float distanceA, float angleB, float distanceB)
    {
        float meanDistance = (distanceA + distanceB) / 2.0f;
        float arcSize = Math.abs(Utils.toBearing(angleB - angleA));

        return (float)(meanDistance * Math.sqrt(2 * (1 - Math.cos(arcSize))));
    }

    /**
     * This method calculates the average of all angles and all distances passed
     * to it in a Map.
     * 
     * @param data
     *            a map containing angles as keys and distances as values.
     * @return the average of all the angles and all the distances that were
     *         on the map in an array of size 2. The first element is the angle,
     *         distance is the second.
     */
    private float[] getMeanAngleDistance(Map<Float,Float> data)
    {
        float meanDistance = 0;
        float meanAngle = 0;
        
        if (data.size() > 0)
        {
            for (Map.Entry<Float,Float> entry : data.entrySet())
            {
                meanAngle += entry.getKey();
                meanDistance += entry.getValue();
            }
            meanDistance = meanDistance / data.size();
            meanAngle = meanAngle / data.size();
        }
        
        return new float[] { meanAngle, meanDistance };
    }
 
    /**
     * Wrapper to get the main ultrasonic sensor's instantaneous reading.
     * 
     * NOTE: if the difference between the first value and the second is higher
     * than 8, then keep the previous value. otherwise go with the newest value.
     * 
     * @return the distance detected by the ultrasonic sensor.
     */
    private float getDistanceMain()
    {   
        float distance = Math.min(m_usMain.getLastDistance(), 55);

        // if it's the first time running this method
        // intialize the previous distance as the current distance
        if (!m_usHasStartedCollectingData)
        {
            m_usHasStartedCollectingData = true;
            m_usPreviousDistance = distance;
            return distance;
        }
        else
        {
            // if the difference between the previous value and the current
            // value is less than 7f, keep the previous value
            if (Math.abs(distance - m_usPreviousDistance) < 10)
            {
                distance = m_usPreviousDistance;
            }
            else
            {
                // if the difference between the previous and the current value
                // is greater, then keep distance as it is, but update the
                // previous distance;
                m_usPreviousDistance = distance;
            }
            return distance;
        }
    }

    /**
     * This method writes a map to a file.
     * 
     * @param data
     *            a non-null map.
     */
    private void writeToFile(Map<Float,Float> data, String filename)
    {
        // Print output on to a txt file
        File file = new File(filename);
        try (BufferedWriter writer = new BufferedWriter(new FileWriter(filename)))
        {
            if (!file.exists())
            {
                file.createNewFile();
            }
            int index = 0;
            for (Map.Entry<Float,Float> entry : data.entrySet())
            {
                writer.write(index + "\t" + String.format("%.1f", entry.getKey()) + "\t" + String.format("%.1f", entry.getValue()) + "\n");
                index++;
            }
        }
        catch (IOException e)
        {
            System.out.println(e.getMessage());
        }
    }
    
    /**
     * Method for writing to a debug file. Appends to same file.
     * 
     * @param data
     *            any string.
     */
    private static void writeDebug(String data)
    {
        // Print output on to a txt file
        File file = new File("Debug.txt");
        try (BufferedWriter writer = new BufferedWriter(new FileWriter("Debug.txt", true)))
        {
            if (!file.exists())
            {
                file.createNewFile();
            }

            writer.write(data + "\n");
        }
        catch (IOException e)
        {
            System.out.println(e.getMessage());
        }
    }
}