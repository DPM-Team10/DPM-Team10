package main;

import java.util.*;
import graph.*;
import lejos.robotics.geometry.*;

/**
 * Keeps track of properties of the board being driven on.
 * 
 * @author Scott Sewell
 */
public class Board
{
    // the number of tiles along one side of the board
    public static final int TILE_COUNT = 12;
    // the width of a board tile in cm
    public static final float TILE_SIZE = 30.4f;
    // the distance the robot tries to leave between itself and the walls to
    // prevent a collision in cm
    public static final float WALL_BUFFER = 2.0f;
    // the distance the robot tries to leave between itself and a zone to
    // prevent entering in cm
    public static final float ZONE_BUFFER = 2.0f;

    private StartParameters m_startParameters;
    private List<Vertex> m_navGraph;
    
    // various zone rectangles
    private Rectangle m_board;
    private Rectangle m_dumpZone;
    private Rectangle m_buildZone;
    private Vector2 m_startCornerPos;

    /**
     * Constructor for the board.
     * 
     * @param lrzx
     *            the red zone lower corner x position.
     * @param lrzy
     *            the red zone lower corner y position.
     * @param urzx
     *            the red zone upper corner x position.
     * @param urzy
     *            the red zone upper corner y position.
     * @param lgzx
     *            the green zone lower corner x position.
     * @param lgzy
     *            the green zone lower corner y position.
     * @param ugzx
     *            the green zone upper corner x position.
     * @param ugzy
     *            the green zone upper corner y position.
     * @param startParams
     *            the StartParameters instance.
     */
    public Board(int lrzx, int lrzy, int urzx, int urzy, int lgzx, int lgzy, int ugzx, int ugzy, StartParameters startParams)
    {
        Vector2 wallLowerCorner = Vector2.one().scale(-TILE_SIZE);
        Vector2 wallUpperCorner = Vector2.one().scale(TILE_SIZE * (TILE_COUNT - 1));
        m_board = Utils.toRect(wallLowerCorner, wallUpperCorner);

        Vector2 dumpLowerCorner = new Vector2(lrzx, lrzy).scale(TILE_SIZE);
        Vector2 dumpUpperCorner = new Vector2(urzx, urzy).scale(TILE_SIZE);
        m_dumpZone = Utils.toRect(dumpLowerCorner, dumpUpperCorner);

        Vector2 buildLowerCorner = new Vector2(lgzx, lgzy).scale(TILE_SIZE);
        Vector2 buildUpperCorner = new Vector2(ugzx, ugzy).scale(TILE_SIZE);
        m_buildZone = Utils.toRect(buildLowerCorner, buildUpperCorner);
        
        m_startParameters = startParams;
        
        int startCorner = m_startParameters.getStartCorner();
        m_startCornerPos = new Vector2(
                            startCorner == 2 || startCorner == 3 ? (Board.TILE_COUNT - 2) * Board.TILE_SIZE : 0,
                            startCorner == 3 || startCorner == 4 ? (Board.TILE_COUNT - 2) * Board.TILE_SIZE : 0);
        
        // get all allowable board position
        m_navGraph = new ArrayList<Vertex>();
        for (int i = 0; i < Board.TILE_COUNT; i++)
        {
            for (int j = 0; j < Board.TILE_COUNT; j++)
            {
                Vector2 point = new Vector2(i * TILE_SIZE, j * TILE_SIZE);
                if (inBounds(point) && !inEnemyZone(point))
                {
                    m_navGraph.add(new Vertex(point));
                }
            }
        }
        
        // connect adjacent vertices
        List<Vertex> unconnected = new ArrayList<Vertex>(m_navGraph);
        for (Vertex i : m_navGraph)
        {
            unconnected.remove(i);
            for (Vertex j : unconnected)
            {
                if (Vector2.distance(i.getValue(), j.getValue()) < Board.TILE_SIZE + 1)
                {
                    i.addNeighbor(new Edge(i, j));
                    j.addNeighbor(new Edge(i, j));
                }
            }
        }
    }
    
    /**
     * Gets the position of the line intersection nearest to the starting corner.
     */
    public Vector2 getStartPos()
    {
        return m_startCornerPos;
    }

    /**
     * @return the position of the center of our zone.
     */
    public Vector2 getTeamZoneCenter()
    {
        Rectangle teamZone = m_startParameters.isBuilder() ? m_buildZone : m_dumpZone;
        return new Vector2((float) teamZone.getCenterX(), (float) teamZone.getCenterY());
    }

    /**
     * Checks if the robot would fit on the board while at a specified position.
     * 
     * @param position
     *            the position to check the validity of.
     * @return true if the robot should fit at the position.
     */
    public boolean inBounds(Vector2 position)
    {
        float padding = -(Robot.RADIUS + WALL_BUFFER);
        return Utils.rectContains(position, Utils.padRect(m_board, padding));
    }

    /**
     * Checks if the robot could overlap the enemy zone while at a specified
     * position.
     * 
     * @param position
     *            the position to check the validity of.
     * @return true if the robot could overlap at the position.
     */
    public boolean inEnemyZone(Vector2 position)
    {
        float padding = Robot.RADIUS + ZONE_BUFFER;
        Rectangle enemyZone = m_startParameters.isBuilder() ? m_dumpZone : m_buildZone;
        return Utils.rectContains(position, Utils.padRect(enemyZone, padding));
    }

    /**
     * Checks if the robot would overlap the enemy zone while traveling between
     * two points.
     * 
     * @param lineStart
     *            the start of the travel path.
     * @param lineEnd
     *            the end of the travel path.
     * @return true if the robot could overlap while traveling.
     */
    public boolean crossesEnemyZone(Vector2 lineStart, Vector2 lineEnd)
    {
        float padding = Robot.RADIUS + ZONE_BUFFER;
        Rectangle enemyZone = m_startParameters.isBuilder() ? m_dumpZone : m_buildZone;
        return Utils.lineIntersectsRect(lineStart, lineEnd, Utils.padRect(enemyZone, padding));
    }

    /**
     * Calculates the x-axis and y-axis line intersection on the board that is
     * closest to a given coordinate.
     * 
     * @return a Vector2 with the position of the intersection in cm.
     */
    public static Vector2 getNearestIntersection(Vector2 position)
    {
        float x = (float) Math.floor((position.getX() / Board.TILE_SIZE) + 0.5f);
        float y = (float) Math.floor((position.getY() / Board.TILE_SIZE) + 0.5f);
        return new Vector2(
                Utils.clamp(x, 0, Board.TILE_COUNT - 2),
                Utils.clamp(y, 0, Board.TILE_COUNT - 2)
                ).scale(Board.TILE_SIZE);
    }

    /**
     * Gets the shortest path between two board points that avoids any invalid
     * regions.
     * 
     * @param from
     *            the position traveled from.
     * @param to
     *            the position traveled to.
     * @param obstacles
     *            a list of the center of any obstacles that should be navigated
     *            around.
     * @return a new list of waypoints the robot should travel between.
     */
    public List<Vector2> findPath(Vector2 from, Vector2 to, List<Vector2> obstacles)
    {
        List<Vector2> path = new ArrayList<Vector2>();
        
        // find the closest nodes in the navigation graph
        Vertex pathStart = getNearestNavPoint(from);
        Vertex pathEnd = getNearestNavPoint(to);

        path.add(from);
        if (!pathStart.equals(pathEnd))
        {
            // do a breadth first search to find the first short path
            Queue<Vertex> toVisit = new LinkedList<Vertex>();
            HashMap<Vertex,Vertex> moves = new HashMap<Vertex,Vertex>();
            
            toVisit.add(pathStart);
            moves.put(pathStart, null);
            
            // find vertices that are temporarily excluded from the navmesh
            HashSet<Vertex> blockedVertices = new HashSet<Vertex>();

            for (Vertex v : m_navGraph)
            {
                for (Vector2 obstacle : obstacles)
                {
                    if (!v.equals(pathEnd) && Vector2.distance(obstacle, v.getValue()) < Robot.RADIUS + 8)
                    {
                        blockedVertices.add(v);
                    }
                }
            }

            Utils.writeDebug("Blocked...");
            for (Vertex v : blockedVertices)
            {
                //System.out.println("B: " + v.getValue().toString());
                Utils.writeDebug(v.getValue().toString());
            }
            
            // until we find a path keep visiting nodes
            while (!toVisit.isEmpty())
            {
                Vertex v = toVisit.remove();
                for (Edge e : v.getNeighbors()) 
                {
                    Vertex next = e.getNeighbor(v);
                    if (!moves.containsKey(next) && !blockedVertices.contains(next))
                    {
                        moves.put(next, v);
                        if (next.equals(pathEnd))
                        {
                            toVisit = new LinkedList<Vertex>();
                            break;
                        }
                        toVisit.add(next);
                    }
                }
            }
            
            // build the waypoint path
            List<Vector2> reversePath = new ArrayList<Vector2>();
            reversePath.add(to);
            
            Vertex currentNode = pathEnd;
            while (currentNode != null)
            {
                reversePath.add(currentNode.getValue());
                currentNode = moves.get(currentNode);
            }
            
            for (Vector2 v : reversePath)
            {
                //System.out.println("R: " + v.toString());
            }
            
            reversePath.add(from);

            // construct a forward path with redundant waypoints removed
            for (int i = reversePath.size() - 1; i >= 0; i--)
            {
                Vector2 lastValidPoint = reversePath.get(i);
                for (int j = i - 1; j >= 0; j--)
                {
                    boolean crossesObstacle = false;
                    for (Vector2 obstacle : obstacles)
                    {
                        Rectangle obstacleArea = Utils.padRect(Utils.toRect(obstacle, obstacle), Robot.RADIUS + 5);
                        if (Utils.lineIntersectsRect(reversePath.get(i), reversePath.get(j), obstacleArea))
                        {
                            crossesObstacle  = true;
                        }
                    }
                    
                    if (crossesEnemyZone(reversePath.get(i), reversePath.get(j)) || crossesObstacle)
                    {
                        path.add(lastValidPoint);
                        i = j + 1;
                        break;
                    }
                    else
                    {
                        lastValidPoint = reversePath.get(j);
                    }
                }
            }
        }
        path.add(to);

        Utils.writeDebug("Path...");
        for (Vector2 v : path)
        {
            //System.out.println("P: " + v.toString());
            Utils.writeDebug(v.toString());
        }

        return path;
    }

    /**
     * Get the nearest point on the board that is in the navigation graph.
     * 
     * @return the closest Vertex.
     */
    public Vertex getNearestNavPoint(Vector2 position)
    {
        float closestDistance = Float.MAX_VALUE;
        Vertex closest = null;
        for (Vertex v : m_navGraph)
        {
            float distance = Vector2.distance(v.getValue(), position);
            if (distance < closestDistance)
            {
                closestDistance = distance;
                closest = v;
            }
        }
        return closest;
    }
}