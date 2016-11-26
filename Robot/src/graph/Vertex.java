package graph;

import java.util.ArrayList;
import main.Vector2;

public class Vertex
{
    private ArrayList<Edge> m_neighborhood;
    private Vector2 m_value;
    
    public Vertex(Vector2 value)
    {
        m_value = value;
        m_neighborhood = new ArrayList<Edge>();
    }

    public void addNeighbor(Edge edge)
    {
        if (m_neighborhood.contains(edge))
        {
            return;
        }
        m_neighborhood.add(edge);
    }

    public boolean containsNeighbor(Edge other)
    {
        return m_neighborhood.contains(other);
    }
    
    public Edge getNeighbor(int index)
    {
        return m_neighborhood.get(index);
    }
    
    Edge removeNeighbor(int index)
    {
        return m_neighborhood.remove(index);
    }
    
    public void removeNeighbor(Edge e)
    {
        m_neighborhood.remove(e);
    }
    
    public int getNeighborCount()
    {
        return m_neighborhood.size();
    }
    
    public Vector2 getValue()
    {
        return m_value;
    }
    
    public boolean equals(Object other)
    {
        if (!(other instanceof Vertex))
        {
            return false;
        }
        Vertex v = (Vertex)other;
        return m_value.equals(v.getValue());
    }
    
    public ArrayList<Edge> getNeighbors()
    {
        return new ArrayList<Edge>(m_neighborhood);
    }
}

