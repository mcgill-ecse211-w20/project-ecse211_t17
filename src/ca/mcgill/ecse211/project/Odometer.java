package ca.mcgill.ecse211.project;

import java.util.concurrent.locks.Condition;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

/**
 * Odometer Class keeps track of current x, y position and theta orientation.
 * 
 * @author Xinyue Chen
 * @author Zheng Yu Cui
 * @author Alixe Delabrousse
 * @author David Kronish
 * @author Ariane Leroux
 *
 */
public class Odometer {

  /**
   * The x-axis position in cm.
   */
  private volatile double x;

  /**
   * The y-axis position in cm.
   */
  private volatile double y; // y-axis position

  /**
   * The orientation in degrees.
   */
  private volatile double theta; // Head angle

  /**
   * The (x, y, theta) position as an array.
   */
  private double[] position;

  // Thread control tools
  /**
   * Fair lock for concurrent writing.
   */
  private static Lock lock = new ReentrantLock(true);

  /**
   * Indicates if a thread is trying to reset any position parameters.
   */
  private volatile boolean isResetting = false;

  /**
   * Lets other threads know that a reset operation is over.
   */
  private Condition doneResetting = lock.newCondition();

  private static Odometer odo; // Returned as singleton

  /**
   * Two int motor variables to store current tacho counts for both wheels
   */
  private static int leftMotorTachoCount = 0;

  private static int rightMotorTachoCount = 0;

  /**
   * Two int motor variables to store last tacho counts for both wheels
   */
  private static int lastTachoCountL = 0;

  private static int lastTachoCountR = 0;

  /**
   * The odometer update period in ms.
   */
  private static final long ODOMETER_PERIOD = 20;
  
  /**
   * Records the last gridpoint the device localized at, in integer coordinates
   */
  public int[] gridpoint = {0,0};
  
  /**
   * Default constructor for this class so it returns a singleton
   */
  private Odometer() {
    
  }
  
  /**
   * Returns the Odometer Object. This method is used solely to obtain one instance ever.
   * @return the Odometer object
   */
  public static synchronized Odometer getOdometer() {
    return new Odometer();
  }
  
  /**
   * Method where the logic for the odometer will run and update all the values
   */
  public void run() {
    
  }
  
  /**
   * Returns the Odometer data.
   * <p>
   * Writes the current position and orientation of the robot in the position array.
   * X, Y, Theta are respectively in the indexes 0, 1, 2
   *  
   * @return the odometer in an array
   */
  public double[] getXyt() {
    return position;
  }
  
  /**
   * Get the current x value
   * @return the current x value
   */
  public double getX() {
    return x;
  }
  
  /**
   * Get the current y value
   * @return the current y value
   */
  public double getY() {
    return y;
  }
  
  /**
   * Get the current theta value
   * @return the current theta value
   */
  public double getTheta() {
    return theta;
  }
  
  /**
   * Adds dx, dy and dtheta to the current values of x, y and theta, respectively. Useful for odometry.
   * 
   * @param dx the change in x
   * @param dy the change in y
   * @param dtheta the change in theta
   */
  public void update(double dx, double dy, double dtheta) {
  
  }
  
  /**
   * Overrides the values of x, y and theta. Use for odometry correction.
   * 
   * @param x the value of x
   * @param y the value of y
   * @param theta the value of theta in degrees
   */
  public void setXyt(double x, double y, double theta) {
    
  }
  
  /**
   * Overwrites x. Use for odometry correction.
   * 
   * @param x the value of x
   */
  public void setX(double x) {
    
  }
  
  /**
   * Overwrites y. Use for odometry correction.
   * 
   * @param y the value of y
   */
  public void setY(double y) {
    
  }
  
  /**
   * Overwrites theta. Use for odometry correction.
   * 
   * @param theta the value of theta
   */
  public void setTheta(double theta) {
    
  }
}


