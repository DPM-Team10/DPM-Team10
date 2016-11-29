package main;

import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;

/**
 * Contains constants related to robot hardware.
 * 
 * @author Scott Sewell
 */
public class Robot
{
    // sensors
    public static final EV3UltrasonicSensor     ULTRASOUND_MAIN     = new EV3UltrasonicSensor(LocalEV3.get().getPort("S3"));
    public static final EV3UltrasonicSensor     ULTRASOUND_UPPER    = new EV3UltrasonicSensor(LocalEV3.get().getPort("S2"));
    public static final EV3ColorSensor          COLOR_LEFT          = new EV3ColorSensor(LocalEV3.get().getPort("S1"));
    public static final EV3ColorSensor          COLOR_RIGHT         = new EV3ColorSensor(LocalEV3.get().getPort("S4"));

    // motors
    public static final EV3LargeRegulatedMotor  MOTOR_LEFT          = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("B"));
    public static final EV3LargeRegulatedMotor  MOTOR_RIGHT         = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
    public static final EV3LargeRegulatedMotor  MOTOR_PULLEY        = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
    public static final EV3LargeRegulatedMotor  MOTOR_CLAW          = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("C"));
    
    // screen
    public static final TextLCD                 SCREEN              = LocalEV3.get().getTextLCD();
    
    // robot dimensions
    public static final float   RADIUS          = 20.0f;                    // farthest point on robot from the center of rotation in cm
    public static final float   WHEEL_RADIUS    = 2.13f;                    // radius of wheels in cm
    public static final float   WHEEL_TRACK     = 16.0f;                    // distance between wheels in cm
    public static final Vector2 US_MAIN_OFFSET  = new Vector2(5,0);         // the position offset of the main ultrasound sensor from the robot center in cm
    public static final Vector2 US_UPPER_OFFSET = new Vector2(-1,9);        // the position offset of the upper ultrasound sensor from the robot center in cm
    public static final Vector2 CSL_OFFSET      = new Vector2(-8.6f,8.9f);  // the position offset of the left color sensor from the robot center in cm
    public static final Vector2 CSR_OFFSET      = new Vector2(-8.6f,-8.9f); // the position offset of the right color sensor from the robot center in cm
    
    // dynamic properties 
    public static final int     ACCELERATION        = 1400;     // acceleration of the wheel motors
    public static final int     MOVE_SPEED          = 180;      // maximum wheel rotation speed while moving forwards in deg/sec
    public static final int     ROTATE_SPEED        = 125;      // maximum wheel rotation speed while turning in deg/sec
    public static final int     LOCALIZATION_SPEED  = 75;       // maximum wheel rotation speed while localizing in deg/sec
    public static final int     SEARCH_SPEED        = 65;       // maximum wheel rotation speed while searching in deg/sec
    public static final int     CLAW_ACCELERATION   = 3500;     // acceleration of the claw motors
    public static final int     CLAW_SPEED          = 50;       // speed of the claw motor in deg/sec
    public static final int     PULLEY_SPEED        = 100;      // speed of the pulley motor in deg/sec
}   
