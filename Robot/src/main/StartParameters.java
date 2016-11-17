package main;

import java.io.IOException;
import java.util.HashMap;

import wifi.WifiConnection;

/**
 * Responsible for interfacing with the wifi connection and parsing the received
 * data into a more usable format for our use.
 * 
 * @author Scott Sewell
 */
public class StartParameters
{
    // the IP address of the computer running the server application
    private static final String SERVER_IP = "192.168.2.26";
    // our project team number
    private static final int TEAM_NUMBER = 10;

    private HashMap<String,Integer> m_data;

    /**
     * Constructor.
     */
    public StartParameters() {}

    /**
     * Use mock wifi data to work around the need for a server to allow for
     * quick testing.
     */
    public void useTestData()
    {
        m_data = new HashMap<String,Integer>();
        m_data.put("Role", 10); // Role     0 = builder, 1 = garbage collector
        m_data.put("SC", 1);    // Start Corner [1,4]
        m_data.put("LRZx", 0);  // Red Zone Lower Left Corner x     [-1,11]
        m_data.put("LRZy", 5);  // Red Zone Lower Left Corner y     [-1,11]
        m_data.put("URZx", 2);  // Red Zone Upper Right Corner x    [-1,11]
        m_data.put("URZy", 9);  // Red Zone Upper Right Corner y    [-1,11]
        m_data.put("LGZx", 6);  // Green Zone Lower Left Corner x   [-1,11]
        m_data.put("LGZy", 3);  // Green Zone Lower Left Corner y   [-1,11]
        m_data.put("UGZx", 8);  // Green Zone Upper Right Corner x  [-1,11]
        m_data.put("UGZy", 4);  // Green Zone Upper Right Corner y  [-1,11]
    }
    
    /**
     * Attempts to connect to the server and get the start data from 
     * the wifi connection. If the connection with the server is successful,
     * it will wait until the data is received before continuing.
     */
    public void getWifiData()
    {
        WifiConnection connection = null;
        try
        {
            System.out.println("Connecting...");
            connection = new WifiConnection(SERVER_IP, TEAM_NUMBER, true);
        }
        catch (IOException e)
        {
            System.out.println("Connection failed");
        }
        
        // if successfully connected, read and print the data received from the server
        if (connection != null)
        {
            m_data = connection.StartData;
            if (m_data == null)
            {
                System.out.println("Failed to read transmission");
            }
            else
            {
                System.out.println("Transmission read:\n" + m_data.toString());
            }
        }
    }
    
    /**
     * @return true if the wifi data has been received.
     */
    public boolean hasRecievedData()
    {
        return m_data != null;
    }
    
    /**
     * @return true if this robot is the builder.
     */
    public boolean isBuilder()
    {
        return m_data.get("Role") == 0;
    }
    
    /**
     * Gets the starting of the corner based on whether the robot is building or collecting.
     * @return the starting corner, where 1 is the bottom left, 2 is the bottom right,
     * 3 is the top right, and 4 is the top left.
     */
    public int getStartCorner()
    {
        return m_data.get("SC");
    }
    
    /**
     * @return a new board constructed using the received data.
     */
    public Board getBoard()
    {
        return new Board(
                m_data.get("LRZx"),
                m_data.get("LRZy"),
                m_data.get("URZx"),
                m_data.get("URZy"),
                m_data.get("LGZx"),
                m_data.get("LGZy"),
                m_data.get("UGZx"),
                m_data.get("UGZy")
                );
    }
}
