package main;

import static org.junit.Assert.*;
import java.security.InvalidParameterException;
import java.util.ArrayList;
import java.util.List;
import org.junit.Test;

/**
 * @author yazami
 *
 */
public class Vector2Test
{
    /**
     * Tests construction of vector and retrieval of values. <br>
     * Test method for {@link main.Vector2#Vector2(float, float)}.
     */
    @Test
    public void testConstruct()
    {
        float x = -204.0f;
        float y = 93.057f;
        Vector2 vec = new Vector2(x, y);
        
        assertEquals(x, vec.getX(), 0f);
        assertEquals(y, vec.getY(), 0f);
    }
    
    /**
     * Tests clone constructor. <br>
     * Test method for {@link main.Vector2#Vector2(main.Vector2)}.
     */
    @Test
    public void testCloneConstruct()
    {
        float x = -54.025f;
        float y = 23.057f;
        Vector2 vec1 = new Vector2(x, y);
        Vector2 vec2 = new Vector2(vec1);
        
        assertEquals(x, vec2.getX(), 0f);
        assertEquals(y, vec2.getY(), 0f);
    }
    
    /**
     * Tests null clone constructor. <br>
     * Test method for {@link main.Vector2#Vector2(main.Vector2)}.
     */
    @Test
    public void testNullConstruct()
    {
        String message = "";
        try
        {
            Vector2 vec = new Vector2(null);
        }
        catch (InvalidParameterException e)
        {
            message = e.getMessage();
        }
        assertEquals("Attempted to clone a null vector!", message);
    }
    

    /**
     * Test method for {@link main.Vector2#zero()}.
     */
    @Test
    public void testZero()
    {
        float zero = 0f;
        Vector2 zeroVector = Vector2.zero();
        
        assertEquals(zero, zeroVector.getX(), 0f);
        assertEquals(zero, zeroVector.getY(), 0f);
    }

    /**
     * Test method for {@link main.Vector2#one()}.
     */
    @Test
    public void testOne() 
    {
        float one = 1f;
        Vector2 oneVector = Vector2.one();
        
        assertEquals(one, oneVector.getX(), 0f);
        assertEquals(one, oneVector.getY(), 0f);
    }

    /**
     * Test method for {@link main.Vector2#unitX()}.
     */
    @Test
    public void testUnitX() 
    {
        float x = 1f;
        float y = 0f;
        Vector2 unitXVector = Vector2.unitX();
        
        assertEquals(x, unitXVector.getX(), 0f);
        assertEquals(y, unitXVector.getY(), 0f);
    }

    /**
     * Test method for {@link main.Vector2#unitY()}.
     */
    @Test
    public void testUnitY()
    {
        float x = 0f;
        float y = 1f;
        Vector2 unitXVector = Vector2.unitY();
        
        assertEquals(x, unitXVector.getX(), 0f);
        assertEquals(y, unitXVector.getY(), 0f);
    }


    /**
     * Test method for {@link main.Vector2#add(main.Vector2)}.
     */
    @Test
    public void testAddVector2() 
    { 
        // create 2 Vector2 objects
        float x = 200.3f;
        float y = -200.4f;  
        Vector2 vec1 = new Vector2(x, y);       
        Vector2 vec2 = new Vector2(vec1);
        
        // adding vec2 to vec1
        vec1.add(vec2);
        
        // create variables for expected values
        float sumX = x + x;
        float sumY = y + y;   
        
        assertEquals(sumX, vec1.getX(), 0f);
        assertEquals(sumY, vec1.getY(), 0f);
    }
    
    /**
     * Test method for {@link main.Vector2#add(main.Vector2)}.
     */
    @Test (expected = InvalidParameterException.class)
    public void testAddVector2Error() 
    {       
        // create 2 Vector2 objects
        float x = 200.3f;
        float y = -200.4f;       
        Vector2 vec1 = new Vector2(x, y);       
        Vector2 vec2 = new Vector2(null);
        
        // adding vec2 to vec1
        vec1.add(vec2);
    }

    /**
     * Test method for {@link main.Vector2#subtract(main.Vector2)}.
     */
    @Test
    public void testSubtractVector2() 
    {
        // create 2 Vector2 objects
        float x = 100.4f;
        float y = 10.4f;        
        Vector2 vec1 = new Vector2(x, y);       
        Vector2 vec2 = new Vector2(vec1);
        
        // substract vec2 from vec1
        vec1.subtract(vec2);
        
        // create variables for expected values
        float differenceX = x - x;
        float differenceY = y - y;
        
        assertEquals(differenceX, vec1.getX(), 0f);
        assertEquals(differenceY, vec1.getY(), 0f);
    }
    
    /**
     * Test method for {@link main.Vector2#subtract(main.Vector2)}.
     */
    @Test (expected = InvalidParameterException.class)
    public void testSubtractVector2Error() 
    {
        // create 2 Vector2 objects
        float x = 100.4f;
        float y = 10.4f;       
        Vector2 vec1 = new Vector2(x, y);       
        Vector2 vec2 = new Vector2(null);
        
        // substract vec2 from vec1
        vec1.subtract(vec2);
    }


    /**
     * Test method for {@link main.Vector2#scale(float)}.
     */
    @Test
    public void testScaleFloat() 
    {
        // create 1 Vector2 object
        float x = 30.4f;
        float y = 13.4f;       
        Vector2 vec1 = new Vector2(x, y);       
        
        float multiplier = 2;         
        vec1.scale(multiplier);  
        
        // create variables for expected values
        float expectedScaledX = x * multiplier;
        float expectedScaledY = y * multiplier;
        
        assertEquals(expectedScaledX, vec1.getX(), 0f);
        assertEquals(expectedScaledY, vec1.getY(), 0f);
    }

    /**
     * Test method for {@link main.Vector2#rotate(float)}.
     */
    @Test
    public void testRotateFloat() 
    {
        // create 1 Vector2 object
        float x = 0;
        float y = 1;       
        Vector2 vec1 = new Vector2(x, y); 
        
        float angle = -90;
        vec1.rotate(angle);
        
        // create variables for expected values
        float expectedRotatedX = 1;
        float expectedRotatedY = 0;
        
        assertEquals(expectedRotatedX, vec1.getX(), 0.00001f);
        assertEquals(expectedRotatedY, vec1.getY(), 0.00001f); 
    }

    /**
     * Test method for {@link main.Vector2#magnitude()}.
     */
    @Test
    public void testMagnitude() 
    {
        // create 1 Vector2 object
        float x = 10;
        float y = 0;       
        Vector2 vec1 = new Vector2(x, y); 
        
        float magnitude = vec1.magnitude();
        
        float expectedMagnitude = 10;
        
        assertEquals(expectedMagnitude, magnitude, 0.00001f);      
    }

    /**
     * Test method for {@link main.Vector2#angle()}.
     */
    @Test
    public void testAngle() 
    {
        // create 1 Vector2 object
        float x = 0;
        float y = 12;       
        Vector2 vec1 = new Vector2(x, y);

        float angle = vec1.angle();
        
        float expectedAngle = 90;
        
        assertEquals(expectedAngle, angle, 0.00001f); 
    }

    /**
     * Test method for {@link main.Vector2#toString()}.
     */
    @Test
    public void testToString() 
    {
        float x = 0;
        float y = 5;       
        Vector2 vec1 = new Vector2(x, y); 
        
        String stringVector = vec1.toString();
        
        String expectedString = "(0.00,5.00)";
        
        assertEquals(expectedString, stringVector);    
    }

    /**
     * Test method for {@link main.Vector2#add(main.Vector2, main.Vector2)}.
     */
    @Test
    public void testAddVector2Vector2() 
    {
        float x = -23.025f;
        float y = 45.057f;
        Vector2 vec1 = new Vector2(x, y);
        Vector2 vec2 = new Vector2(vec1);
        
        Vector2 sum = Vector2.add(vec1, vec2);
        
        float expectedSumX = x + x;
        float expectedSumY = y + y;
        
        assertEquals(expectedSumX, sum.getX(), 0f);
        assertEquals(expectedSumY, sum.getY(), 0f);
    }

    /**
     * Test method for {@link main.Vector2#subtract(main.Vector2, main.Vector2)}.
     */
    @Test
    public void testSubtractVector2Vector2() 
    {
        float x = -42.025f;
        float y = 44.057f;
        Vector2 vec1 = new Vector2(x, y);
        Vector2 vec2 = new Vector2(vec1);
        
        Vector2 difference = Vector2.subtract(vec1, vec2);
        
        float expectedDifferenceX = x - x;
        float expectedDifferenceY = y - y;
        
        assertEquals(expectedDifferenceX, difference.getX(), 0f);
        assertEquals(expectedDifferenceY, difference.getY(), 0f);
    }

    /**
     * Test method for {@link main.Vector2#scale(main.Vector2, float)}.
     */
    @Test
    public void testScaleVector2Float() 
    { 
        // create 1 Vector2 object
        float x = 30.4f;
        float y = 13.4f;       
        Vector2 vec1 = new Vector2(x, y);       
        
        float multiplier = 3;         
        Vector2 scaledVec = Vector2.scale(vec1, 3);
        
        // create variables for expected values
        float expectedScaledX = x * multiplier;
        float expectedScaledY = y * multiplier;
        
        assertEquals(expectedScaledX, scaledVec.getX(), 0f);
        assertEquals(expectedScaledY, scaledVec.getY(), 0f);
    }

    /**
     * Test method for {@link main.Vector2#rotate(main.Vector2, float)}.
     */
    @Test
    public void testRotateVector2Float() 
    {
        // create 1 Vector2 object
        float x = 0;
        float y = 1;       
        Vector2 vec1 = new Vector2(x, y); 
        
        float angle = -90;
        Vector2 rotatedVec = Vector2.rotate(vec1, angle);
        
        // create variables for expected values
        float expectedRotatedX = 1;
        float expectedRotatedY = 0;
        
        assertEquals(expectedRotatedX, rotatedVec.getX(), 0.00001f);
        assertEquals(expectedRotatedY, rotatedVec.getY(), 0.00001f); 

    }

    /**
     * Test method for {@link main.Vector2#distance(main.Vector2, main.Vector2)}.
     */
    @Test
    public void testDistance() 
    {
        // create 2 Vector2 objects
        float x = 100f;
        float y = 50f;  
        Vector2 vec1 = new Vector2(x, y);       
        Vector2 vec2 = new Vector2(vec1);
        
        float distance = Vector2.distance(vec1, vec2);
        
        float expectedDistance = 0f;
        
        assertEquals(expectedDistance, distance, 0.00001f);  
    }

    /**
     * Test method for {@link main.Vector2#fromPolar(float, float)}.
     */
    @Test
    public void testFromPolar() 
    {
        float angle = 0;
        float magnitude = 10;
        
        Vector2 fromPolar = Vector2.fromPolar(angle, magnitude);
        
        assertEquals(angle, fromPolar.angle(), 0.00001f);  
        assertEquals(magnitude, fromPolar.magnitude(), 0.00001f);  
    }
}
