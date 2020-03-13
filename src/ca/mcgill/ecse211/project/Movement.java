package ca.mcgill.ecse211.project;


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
  
  //Private references to the odometer and motors
  private static Odometer odometer;
  
  /**
   * Private static reference to the US sensor for the donut detection
   */
  private static UltrasonicSensor usLocalizer_Donut;
  
  /**
   * Constructor that initializes the odometer and leftmotors using the objects in the resources
   * 
   */
  private Movement(UltrasonicSensor uss) {
    
  }
  
  /**
   * Returns a Movement Object.
   * <p>
   * Used to initialize all static references and the ultrasonic sensor reference.
   * 
   * @return the Movement Object
   */
  public static synchronized Movement GetMovement(UltrasonicSensor uss) {
    return new Movement(uss);
  }
  
  /**
   * Rotate the robot clockwise without stopping, nor resetting speeds
   */
  public static void RotateClockwise() {
    
  }
  
  /**
   * Rotate the robot counter clockwise without stopping, nor resetting speeds
   */    
  public static void RotateCounterClockwise() {
    
  }
  
  /**
   * Move the robot forward for forever without resetting speeds
   */
  public static void GoForward() {
    
  }
  
  /**
   * Moves the robot backward for forever without resetting speeds
   */
  public static void MoveBack() {
    
  }
  
  /**
   * Moves the robot to an inputed grid point
   * <p>
   * Continuously checks if the distance is reached to be able to pause the motion.
   * 
   * @param x x-coordinate in cm 
   * @param y y-coordinate in cm
   * @param isNavigating false when the method is called to localize
   */
  public static void TravelTo(double x, double y, boolean isNavigating) {
    
  }
  
  /**
   * Computes distance that needs to be traveled
   * @param x Current x coordinate
   * @param y Current y coordinate
   * @param zeroX Initial x coordinate
   * @param zeroY Initial y coordinate
   * @return euclidean distance between two points
   */
  public static double Distance(double x, double y, double zeroX, double zeroY) {
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
  public static double GetTheta(double X, double Y, double CurX, double CurY) {
    return 2.0;
  }
  /**
   * Rotates the robot to a certain angle according to the reference frame
   * Positive is in the clockwise direction
   * <i>Uses {@code turnBy(double)}</i>
   * 
   * @param angle to turn to
   */
  public static void TurnTo(double angle) {
    
  }
  
  /**
   * Turn the robot by a certain degrees. Clockwise is positive
   * @param angle the number of degrees the robot needs to rotate
   */
  public static void turnBy(double angle) {
    
  }
  
  /**
   * Converts input angle to the total rotation of each wheel needed to rotate the robot by that angle.
   * 
   * @param angle the input angle
   * @param direction the left wheel or the right wheel (0 is left and otherwise it's right)
   * @return the wheel rotations necessary to rotate the robot by the angle
   */
  public static int ConvertAngle(double angle, int direction) {
    return 0;
  }
  
  /**
   * Converts input distance to the total rotation of each wheel needed to cover that distance.
   * 
   * @param distance the input distance
   * @param direction the left wheel or the right wheel (0 is left and otherwise it's right)
   * @return the wheel rotations necessary to cover the distance
   */
  public static int ConvertDistance(double distance, int direction) {
    return 0;
  }
  
  /**
   * Stops both motors.
   */
  public static void StopMotors() {
  }
}
