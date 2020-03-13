package ca.mcgill.ecse211.project;

import ca.mcgill.ecse211.project.UltrasonicSensor.Direction;

/**
 * Class that performs all the ultrasonic behaviour:
 * <p><ul>
 *  <li> Finding the 0 degrees
 *  <li> Filters the data from the sensor
 *  <li> Returns any distances from the sensor
 * </ul><p>
 *
 * @author Xinyue Chen
 * @author Zheng Yu Cui
 * @author Alixe Delabrousse
 * @author David Kronish
 * @author Ariane Leroux
 *
 */
public class UltrasonicSensor {
  
  //Initialize starting variables
  private float[] usData;
  
  //Initialize variables
  /**
   * Enumeration for which direction the robot should turn, i.e. Clockwise or Counter-clockwise
   *
   */
  public enum Direction { LEFT, RIGHT };
  
  //private objects used by the localizer
  private Odometer odometer;
  
  /**
   * Constructor that initializes the private variable with variables stored in Resources
   */
  public UltrasonicSensor() {
    
  }
  
  /**
   * Method that find the 0deg according to whichever orientation
   */
  public void reorient() {
    
  }
  
  /**
   * Method to move the robot to (1,1) using the ultrasonic sensor
   */
  public void moveToOrigin() {
    
  }
  
  /**
   * Method to move the robot in a specific direction to calculate the falling angle according to the right distance
   * 
   * @param dir direction of rotation to choose which rotation to turn first
   * @return the falling angle
   */
  private double getAngle(Direction dir) {
    return odometer.getTheta();
  }
  
  /**
   * Filter the data using the five values from the sensor and taking the smallest value, if it is infinity cap it at 200
   * 
   * @return the filtered data
   */
  public double getFilteredData() {
    double smallest = 0.0;
    return smallest;
  }
  
  /**
   * Checks whether or not there is an object in front of the sensor.
   * @param distance
   * @return true if there is an object in front of the sensor
   */
  public static boolean ObjectDetected(double distance) {
    return distance < 3.5;
  }
}
