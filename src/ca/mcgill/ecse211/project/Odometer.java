package ca.mcgill.ecse211.project;

import static ca.mcgill.ecse211.project.Main.sleepFor;
import static ca.mcgill.ecse211.project.Resources.BASE_WIDTH;
import static ca.mcgill.ecse211.project.Resources.WHEEL_RAD_LEFT;
import static ca.mcgill.ecse211.project.Resources.WHEEL_RAD_RIGHT;
import static ca.mcgill.ecse211.project.Resources.leftMotor;
import static ca.mcgill.ecse211.project.Resources.rightMotor;
import java.util.concurrent.locks.Condition;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

/**
 * Odometer Class keeps track of current x, y position and theta orientation.
 * 
 * @see <a href="https://www.dropbox.com/s/kyyfui0s3a2m7jj/SOFTWARE%20DOCUMENT.docx?dl=0">Software Documentation for more details</a>
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
    setXyt(0, 0, 0);
  }
  
  /**
   * Returns the Odometer Object. This method is used solely to obtain one instance ever.
   * @return the Odometer object
   */
  public static synchronized Odometer getOdometer() {
    if (odo == null) {
      odo = new Odometer();
    }

    return odo;
  }
  
  /**
   * Starts a new odometer thread 
   */
  public void start() {
    (new Thread() {
      public void run() {
        long updateStart;
        long updateDuration;
        

        while (true) {
          updateStart = System.currentTimeMillis();

          // Calculate new robot position based on tachometer counts
          leftMotorTachoCount = leftMotor.getTachoCount();
          rightMotorTachoCount = rightMotor.getTachoCount();

          // Update odometer values with new calculated values using update()
          double distL; //distance traveled on left wheel
          double distR; //distance traveled on right wheel
          double deltaD; //Vehicle displacement
          double deltaT; //degrees that it rotated
          double dX; //on the board the x displacement
          double dY; // the y displacement on the board

          // compute L and R wheel displacements
          distL = Math.PI * WHEEL_RAD_LEFT * (leftMotorTachoCount - lastTachoCountL) / 180;
          distR = Math.PI * WHEEL_RAD_RIGHT * (rightMotorTachoCount - lastTachoCountR) / 180;
          

          lastTachoCountL = leftMotorTachoCount; // save tacho counts for next iteration
          lastTachoCountR = rightMotorTachoCount;

          deltaD = 0.5 * (distL + distR); // compute vehicle displacement
          deltaT = ((distL - distR) / BASE_WIDTH) * 180 / Math.PI; // convert radians to degrees

          dX = deltaD * Math.sin(theta * Math.PI / 180); // compute X component of displacement
          dY = deltaD * Math.cos(theta * Math.PI / 180); // compute Y component of displacement

          //update(dX, dY, deltaT); // update data on LCD screen
          
          theta += deltaT; // Update direction
          
          //Make sure that the angles will wrap back
          if (theta < -360) {
            theta += 360;
          } else if (theta > 360) {
            theta -= 360;
          }
          
          x = x + dX; // Update estimates of X and Y position
          y = y + dY;

          // this ensures that the odometer only runs once every period
          updateDuration = System.currentTimeMillis() - updateStart;
          if (updateDuration < ODOMETER_PERIOD) {
            sleepFor(ODOMETER_PERIOD - updateDuration);
          }
        }
      }
    }).start();
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
    double[] position = new double[5];
    lock.lock();
    try {
      while (isResetting) { // If a reset operation is being executed, wait until it is over.
        doneResetting.await(); // Using await() is lighter on the CPU than simple busy wait.
      }

      position[0] = x;
      position[1] = y;
      position[2] = theta;
    } catch (InterruptedException e) {
      e.printStackTrace();
    } finally {
      lock.unlock();
    }

    return position;
  }
  
  /**
   * Get the current x value
   * @return the current x value
   */
  public double getX() {
    double value = 0;
    lock.lock();
    try {
      while (isResetting) { // If a reset operation is being executed, wait until it is over.
        doneResetting.await(); // Using await() is lighter on the CPU than simple busy wait.
      }

      value = x;
    } catch (InterruptedException e) {
      e.printStackTrace();
    } finally {
      lock.unlock();
    }
    return value;
  }
  
  /**
   * Get the current y value
   * @return the current y value
   */
  public double getY() {
    double value = 0;
    lock.lock();
    try {
      while (isResetting) { // If a reset operation is being executed, wait until it is over.
        doneResetting.await(); // Using await() is lighter on the CPU than simple busy wait.
      }

      value = y;
    } catch (InterruptedException e) {
      e.printStackTrace();
    } finally {
      lock.unlock();
    }
    return value;
  }
  
  /**
   * Get the current theta value
   * @return the current theta value
   */
  public double getTheta() {
    double value = 0;
    lock.lock();
    try {
      while (isResetting) { // If a reset operation is being executed, wait until it is over.
        doneResetting.await(); // Using await() is lighter on the CPU than simple busy wait.
      }

      value = theta;
    } catch (InterruptedException e) {
      e.printStackTrace();
    } finally {
      lock.unlock();
    }
    return value;
  }
  
  /**
   * Adds dx, dy and dtheta to the current values of x, y and theta, respectively. Useful for odometry.
   * 
   * @param dx the change in x
   * @param dy the change in y
   * @param dtheta the change in theta
   */
  public void update(double dx, double dy, double dtheta) {
    lock.lock();
    isResetting = true;
    try {
      x += dx;
      y += dy;
      theta = (theta + (360 + dtheta) % 360) % 360; // keeps the updates within 360 degrees
      isResetting = false;
      doneResetting.signalAll(); // Let the other threads know we are done resetting
    } finally {
      lock.unlock();
    }
  }
  
  /**
   * Overrides the values of x, y and theta. Use for odometry correction.
   * 
   * @param x the value of x
   * @param y the value of y
   * @param theta the value of theta in degrees
   */
  public void setXyt(double x, double y, double theta) {
    lock.lock();
    isResetting = true;
    try {
      this.x = x;
      this.y = y;
      this.theta = theta;
      isResetting = false;
      doneResetting.signalAll();
    } finally {
      lock.unlock();
    }
  }
  
  /**
   * Overwrites x. Use for odometry correction.
   * 
   * @param x the value of x
   */
  public void setX(double x) {
    lock.lock();
    isResetting = true;
    try {
      this.x = x;
      isResetting = false;
      doneResetting.signalAll();
    } finally {
      lock.unlock();
    }
  }
  
  /**
   * Overwrites y. Use for odometry correction.
   * 
   * @param y the value of y
   */
  public void setY(double y) {
    lock.lock();
    isResetting = true;
    try {
      this.y = y;
      isResetting = false;
      doneResetting.signalAll();
    } finally {
      lock.unlock();
    }
  }
  
  /**
   * Overwrites theta. Use for odometry correction.
   * 
   * @param theta the value of theta
   */
  public void setTheta(double theta) {
    lock.lock();
    isResetting = true;
    try {
      this.theta = theta;
      isResetting = false;
      doneResetting.signalAll();
    } finally {
      lock.unlock();
    }
  }
}


