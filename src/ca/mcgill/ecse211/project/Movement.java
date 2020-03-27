package ca.mcgill.ecse211.project;

import ca.mcgill.ecse211.project.Resources;


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
  //The angle error for when you turn to the right angle otherwise it shimmies
  final static double DEG_ERR = 2.0;
     
  /**
   * Rotate the robot clockwise without stopping, nor resetting speeds
   */
  public static void rotateClockwise() {
    Resources.leftMotor.setSpeed(Resources.ROTATE_SPEED);
    Resources.rightMotor.setSpeed(Resources.ROTATE_SPEED);
    Resources.leftMotor.forward();
    Resources.rightMotor.backward();
  }
  
  /**
   * Rotate the robot counter clockwise without stopping, nor resetting speeds
   */    
  public static void rotateCounterClockwise() {
    Resources.leftMotor.setSpeed(Resources.ROTATE_SPEED);
    Resources.rightMotor.setSpeed(Resources.ROTATE_SPEED);
    Resources.leftMotor.backward();
    Resources.rightMotor.forward();
  }
  
  /**
   * Move the robot forward for forever without resetting speeds
   */
  public static void goForward() {
    Resources.leftMotor.setSpeed(Resources.FORWARD_SPEED_LEFT);
    Resources.rightMotor.setSpeed(Resources.FORWARD_SPEED_RIGHT);
    Resources.leftMotor.forward();
    Resources.rightMotor.forward();
  }
  
  /**
   * Moves the robot backward for forever without resetting speeds
   */
  public static void moveBack() {
    Resources.leftMotor.setSpeed(Resources.FORWARD_SPEED_LEFT);
    Resources.rightMotor.setSpeed(Resources.FORWARD_SPEED_RIGHT);
    Resources.leftMotor.backward();
    Resources.rightMotor.backward();
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
  public static void travelTo(int x, int y, boolean isNavigating) {
    double X = x * Resources.TILE_SIZE;
    double Y = y * Resources.TILE_SIZE;
    
    double CurX = Resources.odometer.getX();
    double CurY = Resources.odometer.getY();
    
    double xDist = X - CurX;
    double yDist = Y - CurY;
    
    
    int xTileCount = Math.abs(x-Resources.odometer.gridpoint[0]);
    int yTileCount = Math.abs(y-Resources.odometer.gridpoint[1]);
    
    turnTo(0);
    
    if (yDist < 0) {
      turnTo(180);
    }
    
    int xCount = 0;
    int yCount = 0;
    while ((distance(Resources.odometer.getX(), Resources.odometer.getY(), CurX, CurY) <= Math.abs(yDist)) && yCount != yTileCount) {
     boolean rightDetects = Resources.rightLightSensor.lineDetected();
     boolean leftDetects = Resources.leftLightSensor.lineDetected();
     if (isNavigating && (rightDetects || leftDetects)) {
       if (rightDetects && !leftDetects) {
         Resources.rightMotor.stop();
         while (!leftDetects) {
           Resources.leftMotor.forward();
         }
       } else if (leftDetects && !rightDetects) {
         Resources.leftMotor.stop();
         while(!rightDetects){
           Resources.rightMotor.forward();  
         }
       }
      yCount++;
     }
     goForward();
   }
   stopMotors();
   
   if (xDist < 0) {
     turnTo(-90);
   } else {
     turnTo(90);
   }
    
   while ((distance(Resources.odometer.getX(), Resources.odometer.getY(), CurX, CurY) <= Math.abs(xDist)) && xCount != xTileCount) {
      boolean rightDetects = Resources.rightLightSensor.lineDetected();
      boolean leftDetects = Resources.leftLightSensor.lineDetected();
      if (isNavigating && (rightDetects || leftDetects)) {
        if (rightDetects && !leftDetects) {
          Resources.rightMotor.stop();
          while (!leftDetects) {
            Resources.leftMotor.forward();
          }
        } else if (leftDetects && !rightDetects) {
          Resources.leftMotor.stop();
          while(!rightDetects){
            Resources.rightMotor.forward();  
          }
        }
       xCount++;
      }
      goForward();
    }
    
  }
  
  /**
   * Computes distance that needs to be traveled
   * @param x Current x coordinate
   * @param y Current y coordinate
   * @param zeroX Initial x coordinate
   * @param zeroY Initial y coordinate
   * @return euclidean distance between two points
   */
  public static double distance(double x, double y, double zeroX, double zeroY) {
    return Math.sqrt(Math.pow(x - zeroX, 2) + Math.pow(y - zeroY, 2));
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
      theta = Math.toDegrees(Math.atan2(whys,exes));
    } else if (Y > CurY) {
      theta = Math.toDegrees(Math.atan2(exes,whys));
    }
    return theta;
  }
  /**
   * Rotates the robot to a certain angle according to the reference frame
   * Positive is in the clockwise direction
   * <i>Uses {@code turnBy(double)}</i>
   * 
   * @param angle to turn to
   */
  public static void turnTo(double angle) {
    double rotationAngle;
    
    double curAngle = Resources.odometer.getXyt()[2];
    
    if ((angle - curAngle)>180) {
      rotationAngle = angle - curAngle -360;
    }
    else if ((angle - curAngle)< -180) {
      rotationAngle = angle - curAngle + 360;
    }
    else {
      rotationAngle = angle - curAngle;  

    }
    turnBy(rotationAngle);
  }
  
  /**
   * Turn the robot by a certain degrees. Clockwise is positive
   * @param angle the number of degrees the robot needs to rotate
   */
  public static void turnBy(double angle) {
    Resources.leftMotor.setSpeed(Resources.ROTATE_SPEED);
    Resources.rightMotor.setSpeed(Resources.ROTATE_SPEED);
    Resources.leftMotor.rotate(convertAngle(angle, 0), true);
    Resources.rightMotor.rotate(-convertAngle(angle, 1), false);
  }
  
  /**
   * Converts input angle to the total rotation of each wheel needed to rotate the robot by that angle.
   * 
   * @param angle the input angle
   * @param direction the left wheel or the right wheel (0 is left and otherwise it's right)
   * @return the wheel rotations necessary to rotate the robot by the angle
   */
  public static int convertAngle(double angle, int direction) {
    return convertDistance(Math.PI * Resources.BASE_WIDTH * angle / 360.0, direction);
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
      radius = Resources.WHEEL_RAD_LEFT;
    } else {
      radius = Resources.WHEEL_RAD_RIGHT;
    }
    int dist = (int) ((180.0 * distance) / (Math.PI * radius));
    return dist;
  }
  
  /**
   * Stops both motors.
   */
  public static void stopMotors() {
    Resources.leftMotor.stop(true);
    Resources.rightMotor.stop(false);
  }
}
