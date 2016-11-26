package main;

import java.security.InvalidParameterException;

/**
 * Represents a 2d vector.
 * 
 * @author Scott Sewell
 */
public class Vector2
{
    private float m_x;
    private float m_y;

    // Define useful vectors.
    public static Vector2 zero()    { return new Vector2(0,0); }
    public static Vector2 one()     { return new Vector2(1,1); }
    public static Vector2 unitX()   { return new Vector2(1,0); }
    public static Vector2 unitY()   { return new Vector2(0,1); }
    
    /**
     * Constructor.
     * 
     * @param x
     *            the x-axis coordinate.
     * @param y
     *            the y-axis coordinate.
     */
    public Vector2(float x, float y)
    {
        m_x = x;
        m_y = y;
    }

    /**
     * Clone constructor.
     * 
     * @param v
     *            a vector to clone.
     */
    public Vector2(Vector2 v)
    {
        if (v != null)
        {
            m_x = v.getX();
            m_y = v.getY();
        }
        else
        {
            throw new InvalidParameterException("Attempted to clone a null vector!");
        }
    }

    /**
     * Adds a vector to this vector.
     * 
     * @param v
     *            the vector to add.
     * @return this vector after the addition.
     */
    public Vector2 add(Vector2 v)
    {
        if (v != null)
        {
            m_x += v.getX();
            m_y += v.getY();
        }
        else
        {
            throw new InvalidParameterException("Attempted to add a null vector!");
        }
        return this;
    }

    /**
     * Subtracts a vector from this vector.
     * 
     * @param v
     *            the vector to subtract.
     * @return this vector after the subtraction.
     */
    public Vector2 subtract(Vector2 v)
    {
        if (v != null)
        {
            m_x -= v.getX();
            m_y -= v.getY();
        }
        else
        {
            throw new InvalidParameterException("Attempted to subtract a null vector!");
        }
        return this;
    }

    /**
     * Multiplies this vector by a.
     * 
     * @param a
     *            the scalar to multiply the vector by.
     * @return this vector after the multiplication.
     */
    public Vector2 scale(float a)
    {
        m_x *= a;
        m_y *= a;
        return this;
    }
    
    /**
     * Makes the vector a unit vector.
     * 
     * @return this vector after normalization.
     */
    public Vector2 normalize()
    {
        return this.scale(1 / this.magnitude());
    }
    
    /**
     * Compares two vectors.
     * 
     * @return true if they have the same value.
     */
    public boolean equals(Vector2 v)
    {
        if (v != null)
        {
            return m_x == v.getX() && m_y == v.getY();
        }
        else
        {
            throw new InvalidParameterException("Attempted to compare a null vector!");
        }
    }

    /**
     * Rotates this vector by a specified angle.
     * 
     * @param angle
     *            counter-clockwise angle in degrees.
     * @return this vector after the rotation.
     */
    public Vector2 rotate(float angle)
    {
        double radians = Math.toRadians(angle);
        float sin = (float) Math.sin(radians);
        float cos = (float) Math.cos(radians);

        float tx = m_x;
        float ty = m_y;

        m_x = cos * tx - sin * ty;
        m_y = sin * tx + cos * ty;
        return this;
    }

    /**
     * Calculates the magnitude of this vector.
     * 
     * @return the magnitude.
     */
    public float magnitude()
    {
        return (float) Math.sqrt(m_x * m_x + m_y * m_y);
    }

    /**
     * Calculates the angle of this vector.
     * 
     * @return the angle in degrees counter-clockwise from the x-axis.
     */
    public float angle()
    {
        double angle = Math.atan(m_y / m_x);
        if (m_x < 0)
        {
            angle += Math.PI;
        }
        // brings range [-pi/2, 3pi/2] to [0, 2pi]
        if (angle < 0)
        {
            angle += 2 * Math.PI;
        }
        return (float) (Math.toDegrees(angle));
    }

    /**
     * Creates a formatted string to represent the vector.
     * 
     * @return a string formatted like "(3.21,30.53)".
     */
    public String toString()
    {
        return "(" + String.format("%.2f", m_x) + "," + String.format("%.2f", m_y) + ")";
    }

    /**
     * Adds two vectors.
     * 
     * @param v1
     *            a vector.
     * @param v2
     *            a vector to add to v1.
     * @return a new vector containing the sum.
     */
    public static Vector2 add(Vector2 v1, Vector2 v2)
    {
        return new Vector2(v1).add(v2);
    }

    /**
     * Subtracts v2 from v1.
     * 
     * @param v1
     *            a vector.
     * @param v2
     *            a vector to subtract from v1.
     * @return a new vector containing the difference.
     */
    public static Vector2 subtract(Vector2 v1, Vector2 v2)
    {
        return new Vector2(v1).subtract(v2);
    }

    /**
     * Multiplies a vector.
     * 
     * @param v
     *            a vector.
     * @param a
     *            scaler to multiply v by.
     * @return a new scaled vector.
     */
    public static Vector2 scale(Vector2 v, float a)
    {
        return new Vector2(v).scale(a);
    }

    /**
     * Rotates a vector.
     * 
     * @param v
     *            a vector.
     * @param a
     *            counter-clockwise angle in degrees from the x-axis to rotate v
     *            by.
     * @return a new rotated vector.
     */
    public static Vector2 rotate(Vector2 v, float a)
    {
        return new Vector2(v).rotate(a);
    }

    /**
     * Calculate the distance between two vectors.
     * 
     * @param v1
     *            a vector.
     * @param v2
     *            a vector.
     * @return the distance between the two vectors.
     */
    public static float distance(Vector2 v1, Vector2 v2)
    {
        return Vector2.subtract(v2, v1).magnitude();
    }

    /**
     * Creates a new vector from a set of polar coordinates.
     * 
     * @param angle
     *            the angle in degrees from the x-axis.
     * @param magnitude
     *            the length of the vector.
     * @return new vector with the specified angle and magnitude.
     */
    public static Vector2 fromPolar(float angle, float magnitude)
    {
        return Vector2.unitX().scale(magnitude).rotate(angle);
    }

    /**
     * @return the x-axis coordinate.
     */
    public float getX()
    {
        return m_x;
    }

    /**
     * @return the y-axis coordinate.
     */
    public float getY()
    {
        return m_y;
    }

    /**
     * Sets the x-axis coordinate.
     * 
     * @param x
     *            the new x-axis value.
     */
    public void setX(float x)
    {
        m_x = x;
    }

    /**
     * Sets the y-axis coordinate.
     * 
     * @param y
     *            the new y-axis value.
     */
    public void setY(float y)
    {
        m_y = y;
    }
}
