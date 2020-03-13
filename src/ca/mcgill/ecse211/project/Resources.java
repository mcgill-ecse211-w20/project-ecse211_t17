package ca.mcgill.ecse211.project;


/**
 * This class is used to define static resources in one place for easy access and to avoid cluttering the rest of the
 * codebase. All resources can be imported at once like this:
 * 
 * <p>
 * {@code import static ca.mcgill.ecse211.lab3.Resources.*;}
 * 
 * @author Xinyue Chen
 * @author Zheng Yu Cui
 * @author Alixe Delabrousse
 * @author David Kronish
 * @author Ariane Leroux
 * 
 */
public class Resources {

  /**
   * Wheel ratio
   */
  private static final double RATIO = 0.9976;
  
  /**
   * The wheel radius in centimeters.
   */
  public static final double WHEEL_RAD_LEFT = 2.130;
  public static final double WHEEL_RAD_RIGHT = 2.130;

  /**
   * The robot width in centimeters.
   */
  public static final double BASE_WIDTH = 10.625;

  /**
   * The speed at which the robot moves forward in degrees per second right.
   */
  public static final int FORWARD_SPEED_RIGHT = 150;
  
  /**
   * The speed at which the robot moves forward in degrees par second left.
   */
  public static final int FORWARD_SPEED_LEFT = (int) (FORWARD_SPEED_RIGHT * RATIO);
 
  /**
   * The motor acceleration in degrees per second squared.
   */
  public static final int ACCELERATION = 1500;

  /**
   * Timeout period in milliseconds.
   */
  public static final int TIMEOUT_PERIOD = 3000;

  /**
   * The tile size in centimeters. Note that 30.48 cm = 1 ft.
   */
  public static final double TILE_SIZE = 30.48;
  
  /**
   * Wall distance
   */
  public static final double WALL_DISTANCE = 50;
  
  /**
   * Error margin
   */
  public static final double MARGIN = 2;
  
  /**
   * Distance between sensor and wheel axle
   */
  public static final double LIGHT_SENSOR_DISTANCE = 13.5;
  
  /**
   * Offset of the sensor
   */
  public static final double SENSOR_OFFSET = 14.7;

  /**
   * The odometer.
   */
  public static Odometer odometer = Odometer.getOdometer();
  
  /**
   * The Movement
   */
  public static Movement movement;
  
  public static void setMovement(UltrasonicSensor uss) { Resources.movement = Movement.GetMovement(uss); }
  
  
  
}
