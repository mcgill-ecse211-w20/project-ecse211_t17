package ca.mcgill.ecse211.project;

import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

/**
 * LightSensor initializes the sample providers in order to read the data from each physical sensors.
 * 
 * @author Xinyue Chen
 * @author Zheng Yu Cui
 * @author Alixe Delabrousse
 * @author David Kronish
 * @author Ariane Leroux
 *
 */
public class LightSensor {
  
  /**
   * Refers to the corresponding physical sensor
   */
  public EV3ColorSensor sensor;
  
  //Initialize variables for the colour sensor
  /**
   * Create instance of a corresponding sensor mode 
   */
  private SensorModes sensorModes;
  
  /**
   * Create instance of the sample provider
   */
  private SampleProvider sampleProvider;
  
  /**
   * Array to store measured color values
   */
  private float[] colourSensorValues;
  
  /**
   * The color read by the color sensor
   */
  public float color;
  
  /**
   * Constructor to associate physical sensor to the corresponding sampling object 
   * 
   * @param EV3ColorSensor sensor - the physical entity of the sensor
   * @param String mode - the mode in which the sampling should be done
   * 
   */
  public LightSensor(EV3ColorSensor sensor, String mode) {
    this.sensor = sensor;
    this.sensorModes = this.sensor;
    this.sampleProvider = this.sensorModes.getMode(mode);
    this.colourSensorValues = new float[this.sensorModes.sampleSize()];
  }
  
  
  /**
   * Reads the color from the Color Sensor
   */
  public void readColorSampleRed() {
    this.sampleProvider.fetchSample(colourSensorValues, 0);
    color = colourSensorValues[0] * 100;
    
   }
  
  /**
   * Check whether a line is detected or not given a float sensor value
   * 
   * @param colorSensorValue from a EV3ColorSensor
   */
  public boolean lineDetected() {
    return this.color < 0.35;
  }
  
  /**
   * Helper method to make angle positive
   * 
   * @param angle to make positive in degrees
   * @return angle that is positive in degrees
   */
  private double makePositive(double angle) {
    return angle;
  }
  
  /**
   * Helper method to determine the offset from the old reference frame to the actual reference frame
   * @param a angle supposed to be at 180 degrees
   * @param b angle supposed to be at 0 degree
   * @return the offset in degrees between old and new reference frame
   */
  private double fixAngle(double a, double b) {
    double diff = a - b;
    diff = (diff < 0) ? (diff + 360.0) : diff;
    diff = (diff -180)/2.0;
    diff = diff + b;
    diff = (diff > 180) ? (diff - 360) : diff;
    System.out.println("diff" + -diff);
    return -diff;
  }
  
  
}
