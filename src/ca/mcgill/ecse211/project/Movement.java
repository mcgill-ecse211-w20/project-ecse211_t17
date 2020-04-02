package ca.mcgill.ecse211.project;

import static ca.mcgill.ecse211.project.Resources.*;


/**
 * The Movement class moves the robot in the grid according to the values in the odometer
 * 
 * @author Xinyue Chen
 * @author Zheng Yu Cui
 * @author Alixe Delabrousse
 * @author David Kronish
 * @author Ariane Leroux
 *
 */
public class Movement {
  // The angle error for when you turn to the right angle otherwise it shimmies
  final static double DEG_ERR = 2.0;

  /**
   * Rotate the robot clockwise without stopping, nor resetting speeds
   */
  public static void rotateClockwise() {
    leftMotor.setSpeed(ROTATE_SPEED);
    rightMotor.setSpeed(ROTATE_SPEED);
    leftMotor.forward();
    rightMotor.backward();
  }

  /**
   * Rotate the robot counter clockwise without stopping, nor resetting speeds
   */
  public static void rotateCounterClockwise() {
    leftMotor.setSpeed(ROTATE_SPEED);
    rightMotor.setSpeed(ROTATE_SPEED);
    leftMotor.backward();
    rightMotor.forward();
  }

  /**
   * Move the robot forward for forever without resetting speeds
   */
  public static void goForward() {
    leftMotor.setSpeed(FORWARD_SPEED_LEFT);
    rightMotor.setSpeed(FORWARD_SPEED_RIGHT);
    leftMotor.forward();
    rightMotor.forward();
  }

  /**
   * Moves the robot backward for forever without resetting speeds
   */
  public static void moveBack() {
    leftMotor.setSpeed(FORWARD_SPEED_LEFT);
    rightMotor.setSpeed(FORWARD_SPEED_RIGHT);
    leftMotor.backward();
    rightMotor.backward();
  }

  /**
   * Go forward a set distance in cm
   * 
   * @param distance distance in cm
   */
  public static void goForward(double distance) {
    leftMotor.rotate(convertDistance(distance, 0), true);
    rightMotor.rotate(convertDistance(distance, 1), false);

  }

  /**
   * Moves the robot to an inputed grid point
   * <p>
   * Continuously checks if the distance is reached to be able to pause the motion.
   * 
   * @param x x-coordinate
   * @param y y-coordinate
   * @param isNavigating false when the method is called to localize
   */
  public static void travelTo(int x, int y, boolean isLookingObstacles) {
    double X = x * TILE_SIZE;
    double Y = y * TILE_SIZE;

    double CurX = odometer.getX();
    double CurY = odometer.getY();

    double xDist = X - CurX;
    double yDist = Y - CurY;


    int xTileCount = Math.abs(x - odometer.gridpoint[0]);
    int yTileCount = Math.abs(y - odometer.gridpoint[1]);

    turnTo(0);

    if (yDist < 0) {
      turnTo(180);
    }

    int xCount;
    int yCount;
    // if robot is looking for obstacles being when it's avoid the objects it's hard to keep track of amount of tiles,
    // this ensures that that checking is ignored
    if (isLookingObstacles) {
      xCount = -1;
      yCount = -1;
    } else {
      xCount = 0;
      yCount = 0;
    }

    while ((Math.abs(odometer.getY() - CurY) <= Math.abs(yDist)) && yCount != yTileCount) {

      if (isLookingObstacles && UltrasonicSensor.getDistance() < DETECTION_DISTANCE) {

        UltrasonicSensor.avoid();
      }

      // detecting lines is hard when you need to avoid obstacles
      if (!isLookingObstacles) {
        if (rightLightPoller.lineDetected() || leftLightPoller.lineDetected()) {
          if (rightLightPoller.lineDetected() && !leftLightPoller.lineDetected()) {
            rightMotor.stop();
            while (!leftLightPoller.lineDetected()
                || UltrasonicSensor.getDistance() < DETECTION_DISTANCE) {
              leftMotor.forward();
            }
          } else if (leftLightPoller.lineDetected() && !rightLightPoller.lineDetected()) {
            leftMotor.stop();
            while (!rightLightPoller.lineDetected()
                || UltrasonicSensor.getDistance() < DETECTION_DISTANCE) {
              rightMotor.forward();
            }
          }
          yCount++;
        }
      }

      goForward();
    }
    stopMotors();

    if (xDist < 0) {
      turnTo(-90);
    } else {
      turnTo(90);
    }


    while ((Math.abs(odometer.getX() - CurX) <= Math.abs(xDist)) && xCount != xTileCount) {

      if (isLookingObstacles && UltrasonicSensor.getDistance() < DETECTION_DISTANCE) {

        UltrasonicSensor.avoid();
      }

      // detecting lines is hard when you need to avoid obstacles
      if (!isLookingObstacles) {
        if (rightLightPoller.lineDetected() || leftLightPoller.lineDetected()) {
          if (rightLightPoller.lineDetected() && !leftLightPoller.lineDetected()) {
            rightMotor.stop();
            while (!leftLightPoller.lineDetected()) {
              leftMotor.forward();
            }
          } else if (leftLightPoller.lineDetected() && !rightLightPoller.lineDetected()) {
            leftMotor.stop();
            while (!rightLightPoller.lineDetected()) {
              rightMotor.forward();
            }
          }
          xCount++;
        }
      }

      goForward();
    }


  }

  public static void moveForwardSearch(double distance) {
    double CurX = odometer.getX();
    double CurY = odometer.getY();

    if (!Robot.getFoundCart()) {
      while (distance(odometer.getX(), odometer.getY(), CurX, CurY) < distance) {

        if (UltrasonicSensor.getDistance() < DETECTION_DISTANCE) {

          int colour = ColorDetector.DetectColor();
          if (colour != -1) {
            Robot.setFoundCart(true);

            break;
          }
          UltrasonicSensor.avoid();

        }
        goForward();
      }
    }

  }

  /**
   * Computes distance that needs to be traveled
   * 
   * @param x Current x coordinate
   * @param y Current y coordinate
   * @param zeroX Initial x coordinate
   * @param zeroY Initial y coordinate
   * @return euclidean distance between two points
   */
  public static double distance(double x, double y, double zeroX, double zeroY) {
    return Math.sqrt(Math.pow(x - zeroX, 2) + Math.pow(y - zeroY, 2));
  }

  


  public static void raiseCart() {
    goForward(-15.0);
    turnBy(180.0);
    goForward(-14.0 - DETECTION_DISTANCE); //off by a cm from the cart to give some leeway
    raiseMotor.rotate(135, true);
  }
  
  /**
   * TravelTo function to move the robot to the coordinates x and y in the coordinate system set by the odometer
   * @param x the x coordinate
   * @param y the y coordinate
   */
  public static void travelTo(double x, double y) {
      
      //Difference from current point to the destination point
      double angle; // angle between the current point and destination
      double distance; // distance between current point and destination
      
      //Math to calculate the angle using trigonometry and Pytharian theorem
      angle = Math.toDegrees(Math.atan2(y - odometer.getY(), x - odometer.getX()));
      distance = Math.sqrt((x - odometer.getX())*(x - odometer.getX()) + (y - odometer.getY())*(y - odometer.getY()));
      
      //Turn to the correct angle in the direction of the destination and move the right distance to the destination
      turnTo(angle);
      goForward(distance);
  }


  /**
   * Computes the minimal angle to have it between -180 and 180
   * 
   * @param X Destination x coordinate
   * @param Y Destination y coordinate
   * @param CurX Current x coordinate
   * @param CurY Current y coordinate
   * @return the minimal angle between the two points
   */
  public static double getTheta(double X, double Y, double CurX, double CurY) {
    double theta = 0;
    double exes = Math.abs((X - CurX));
    double whys = Math.abs((Y - CurY));
    if (Y < CurY) {
      theta = Math.toDegrees(Math.atan2(whys, exes));
    } else if (Y > CurY) {
      theta = Math.toDegrees(Math.atan2(exes, whys));
    }
    return theta;
  }

  /**
   * Rotates the robot to a certain angle according to the reference frame Positive is in the clockwise direction
   * <i>Uses {@code turnBy(double)}</i>
   * 
   * @param angle to turn to
   */
  public static void turnTo(double angle) {
    double rotationAngle;

    double curAngle = odometer.getXyt()[2];

    if ((angle - curAngle) > 180) {
      rotationAngle = angle - curAngle - 360;
    } else if ((angle - curAngle) < -180) {
      rotationAngle = angle - curAngle + 360;
    } else {
      rotationAngle = angle - curAngle;

    }
    turnBy(rotationAngle);
  }

  /**
   * Turn the robot by a certain degrees. Clockwise is positive
   * 
   * @param angle the number of degrees the robot needs to rotate
   */
  public static void turnBy(double angle) {
    leftMotor.setSpeed(ROTATE_SPEED);
    rightMotor.setSpeed(ROTATE_SPEED);
    leftMotor.rotate(convertAngle(angle, 0), true);
    rightMotor.rotate(-convertAngle(angle, 1), false);
  }

  /**
   * Converts input angle to the total rotation of each wheel needed to rotate the robot by that angle.
   * 
   * @param angle the input angle
   * @param direction the left wheel or the right wheel (0 is left and otherwise it's right)
   * @return the wheel rotations necessary to rotate the robot by the angle
   */
  public static int convertAngle(double angle, int direction) {
    return convertDistance(Math.PI * BASE_WIDTH * angle / 360.0, direction);
  }

  /**
   * Converts input distance to the total rotation of each wheel needed to cover that distance.
   * 
   * @param distance the input distance
   * @param direction the left wheel or the right wheel (0 is left and otherwise it's right)
   * @return the wheel rotations necessary to cover the distance
   */
  public static int convertDistance(double distance, int direction) {
    double radius;
    if (direction == 0) {
      radius = WHEEL_RAD_LEFT;
    } else {
      radius = WHEEL_RAD_RIGHT;
    }
    int dist = (int) ((180.0 * distance) / (Math.PI * radius));
    return dist;
  }

  /**
   * Stops both motors.
   */
  public static void stopMotors() {
    leftMotor.stop(true);
    rightMotor.stop(false);
  }
}
