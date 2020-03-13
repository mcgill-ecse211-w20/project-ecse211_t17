package ca.mcgill.ecse211.project;


/**
 * Light Sensor reference to the localization
 * 
 * @author Xinyue Chen
 * @author Zheng Yu Cui
 * @author Alixe Delabrousse
 * @author David Kronish
 * @author Ariane Leroux
 *
 */
public class LightSensor {
  
  //Initialize variables for the colour sensor
  private float[] colourSensorValues;
  
  //Private objects to store odometer and movement
  private Odometer odometer;
  
  /**
   * Constructor that takes nothing as input and initializes the odometer and movement objects using the objects stored in the resources
   * Also initializes the Light sensor and the array with the values
   */
  public LightSensor() {
    
  }
  
  /**
   * Reorientation of the robot, i.e. fixing position AND angles
   */
  public void localize() {
    
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
