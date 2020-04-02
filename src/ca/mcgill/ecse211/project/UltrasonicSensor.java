package ca.mcgill.ecse211.project;

import ca.mcgill.ecse211.project.UltrasonicSensor.Direction;
import lejos.hardware.Sound;
import static ca.mcgill.ecse211.project.Resources.*;


/**
 * Class that performs all the ultrasonic behaviour:
 * <p>
 * <ul>
 * <li>Finding the 0 degrees
 * <li>Filters the data from the sensor
 * <li>Returns any distances from the sensor
 * </ul>
 * <p>
 *
 * @author Xinyue Chen
 * @author Zheng Yu Cui
 * @author Alixe Delabrousse
 * @author David Kronish
 * @author Ariane Leroux
 *
 */

public class UltrasonicSensor {

  // Initialize starting variables

  private static float[] usData = new float[US_SENSOR.getMode("Distance").sampleSize()];;
  /**
   * Enumeration for which direction the robot should turn, i.e. Clockwise or Counter-clockwise
   *
   */
  public enum Direction {
    LEFT, RIGHT
  };

  /**
   * Distance measured by the sensor
   */
  private static double distance;

  // private objects used by the localizer
  private static Odometer odometer = Resources.odometer;

  /**
   * Method to create a thread that will run while in the search zone to poll the distance from the Ultrasonic Distance
   */
  public static void usSensorPoller() {
    (new Thread() {
      public void run() {
        while (Robot.isLookingObstacles || Robot.isSearching) {
          while (Robot.isSearching) {
            distance = getFilteredData();
            try {
              Thread.sleep(200);
            } catch (InterruptedException e) {

            }
          }
        }
      }
    }).start();
  }



  /**
   * Method that find the 0deg according to whichever orientation
   */
  public static void reorient() {

    // Getting the two falling angles
    double angleA = getAngle(Direction.RIGHT);
    double angleB = getAngle(Direction.LEFT);

    double deltaT = 0;
    if (angleA > angleB) {
      deltaT = 45 - (angleA + angleB) / 2;
    } else if (angleA < angleB) {
      deltaT = 225 - (angleA + angleB) / 2;
    }

    // New angle that indicates the actual orientation of the robot according to north
    double updatedAngle = deltaT + odometer.getTheta();
    odometer.setXyt(0, 0, updatedAngle);

    // Turn to the true north
    Movement.turnTo(0.0);

  }

  /**
   * Method to move the robot to (1,1) using the ultrasonic sensor
   */
  public static void moveToOrigin() {
    // The x and y coordinates of (1,1) according to the current location of the robot
    double x, y;

    // Move backwards until the distance is at one TILE_SIZE with consideration of the sensor distance in the y
    // direction
    Movement.turnBy(180.0);
    Robot.setYBeginCoord(getFilteredData());
    while (getFilteredData() < TILE_SIZE - US_SENSOR_DISTANCE) {
      Movement.moveBack();
    }

    // Move backwards until the distance is at one TILE_SIZE with consideration of the sensor distance in the x
    // direction
    Movement.turnBy(270.0);
    Robot.setXBeginCoord(getFilteredData());
    while (getFilteredData() < TILE_SIZE - US_SENSOR_DISTANCE) {
      Movement.moveBack();
    }

    /**Since our robot will always face the "open"-end and parallel to the wall on its left, these are adjustments according to the corner 
     *       true 0deg
     * 3-->      |         2
     *           |         |
     *           |         V
     *           |
     * __________|__________
     *           |          
     *           |
     * A         |
     * |         |
     * 0         |     <-- 1       
     */
    if (corner == 0) {
      Movement.turnTo(0.0);
      INIT_NORTH_ANGLE = 0.0;
      odometer.setXyt(1 * TILE_SIZE, TILE_SIZE, 0.0);
      setInitPos(1, 1);
    } else if (corner == 2) {
      Movement.turnTo(180.0);
      INIT_NORTH_ANGLE = -180.0;
      odometer.setXyt((MAP_x - 1) * TILE_SIZE, (MAP_y - 1) * TILE_SIZE, 0.0);
      setInitPos((MAP_x - 1), (MAP_y - 1));
    } else if (corner == 1) {
      Movement.turnTo(90.0);
      INIT_NORTH_ANGLE = -90.0;
      odometer.setXyt((MAP_x - 1) * TILE_SIZE, TILE_SIZE, 0.0);
      setInitPos((MAP_x - 1), 1);
    } else if (corner == 3) {
      Movement.turnTo(-90.0);
      INIT_NORTH_ANGLE = 90.0;
      odometer.setXyt(1 * TILE_SIZE, (MAP_y - 1) * TILE_SIZE, 0.0);
      setInitPos(1, (MAP_y - 1));
    }

  }

  /**
   * Method to move the robot in a specific direction to calculate the falling angle according to the right distance
   * 
   * @param dir direction of rotation to choose which rotation to turn first
   * @return the falling angle
   */
  private static double getAngle(Direction dir) {

    // Choosing which falling edge we're detecting
    if (dir == Direction.RIGHT) {

      // Looking for Empty space
      while (getFilteredData() < 1000) {
        Movement.rotateClockwise();
      }

      // Turn as long as Empty Space
      while (getFilteredData() > WALL_DISTANCE) {
        Movement.rotateClockwise();
      }

    } else {

      // Looking for Empty space
      while (getFilteredData() < 1000) {
        Movement.rotateCounterClockwise();
      }

      // Turn as long as Empty Space
      while (getFilteredData() > WALL_DISTANCE) {
        Movement.rotateCounterClockwise();
      }
    }

    return odometer.getTheta();
  }

  /**
   * Move the robot around an obstacle
   * 
   */
  public static void avoid() {
    double orientation;
    double angle = odometer.getTheta();

    if (isWithin(angle, 180.0) || isWithin(angle, 0.0)) {
      orientation = odometer.getX();
      Movement.turnBy(90.0);

      do {
        Movement.goForward(4.0);
        if (isClose()) {
          Movement.turnBy(90.0);
        }
      } while (odometer.getX() > orientation);



    } else if (isWithin(angle, -90.0) || isWithin(angle, 90.0)) {
      orientation = odometer.getY();

      Movement.turnBy(90.0);

      do {
        Movement.goForward(4.0);
        if (isClose()) {
          Movement.turnBy(90.0);
        }
      } while (odometer.getY() > orientation);
    } else {
      orientation = getCurrentSlope();
      
      Movement.turnBy(90.0);
      
      do {
        Movement.goForward(4.0);
        if (isClose()) {
          Movement.turnBy(90.0);
        }
      } while (getCurrentSlope() > orientation);
    }
  }

  private static boolean isWithin(double comparedAngle, double wantedAngle) {
    return (comparedAngle < wantedAngle + 2.0 && comparedAngle > wantedAngle - 2.0);
  }

  private static boolean isClose() {
    Movement.turnBy(-90.0);
    boolean isClose = getDistance() <= DETECTION_DISTANCE;
    Movement.turnBy(90.0);
    return isClose;
  }
  
  private static double getCurrentSlope() {
    return odometer.getY()/odometer.getX();
  }

  /**
   * Filter the data using the five values from the sensor and taking the smallest value, if it is infinity cap it at
   * 200
   * 
   * @return the filtered data
   */
  private static double getFilteredData() {

    // Allocating 5 spots for the sensor's data
    double[] data = new double[5];

    // Store each sensor datum in the array
    for (int i = 0; i < data.length; i++) {
      US_SENSOR.fetchSample(usData, 0);
      double distance = (double) (usData[0] * 100.0);
      data[i] = distance;
    }

    // Get the smallest value in the array
    double smallest = data[0];
    for (int i = 0; i < data.length; i++) {
      smallest = (data[i] < smallest) ? smallest : data[i];
    }

    // If the smallest distance is infinity then cap it at 1000
    smallest = (smallest > 1000) ? 1000 : smallest;
    return smallest;

  }

  public static double getDistance() {
    return distance;
  }

  /**
   * Checks whether or not there is an object in front of the sensor.
   * 
   * @param distance
   * @return true if there is an object in front of the sensor
   */
  public static boolean ObjectDetected(double distance) {
    return distance < 3.5;
  }
}
