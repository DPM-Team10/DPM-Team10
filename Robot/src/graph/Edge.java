package graph;

public class Edge
{
    private Vertex m_one, m_two;

    public Edge(Vertex one, Vertex two)
    {
        m_one = one;
        m_two = two;
    }

    public Vertex getNeighbor(Vertex current)
    {
        if (!(current.equals(m_one) || current.equals(m_two)))
        {
            return null;
        }
        return (current.equals(m_one)) ? m_two : m_one;
    }

    public Vertex getOne()
    {
        return m_one;
    }

    public Vertex getTwo()
    {
        return m_two;
    }

    public String toString()
    {
        return "({" + m_one + ", " + m_two + "})";
    }

    public boolean equals(Object other)
    {
        if (!(other instanceof Edge))
        {
            return false;
        }
        Edge e = (Edge)other;
        return e.m_one.equals(m_one) && e.m_two.equals(m_two);
    }
}
