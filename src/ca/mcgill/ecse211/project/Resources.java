package ca.mcgill.ecse211.project;

import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;

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
   * The speed at which the robot moves when rotating in degrees per second.
   */
  public static final int ROTATE_SPEED = 90;
 
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
   * Distance between us sensor and wheel axle
   */
  public static final double US_SENSOR_DISTANCE = 2.1;
  
  /**
   * Distance where an object will be detected
   */
  public static final double DETECTION_DISTANCE = 3.5;

  public enum Team { GREEN, RED };
  /**
   * Team storing details
   */
  public static Team team = null;
  
  /**
   * Starting corner number
   */
  public static int corner = -1;
  
  /**
   * Tunnel lower left coordinates
   */
  public static int TN_LL_x = -1;
  public static int TN_LL_y = -1;
  
  /**
   * Tunnel upper right coordinates
   */
  public static int TN_UR_x = -1;
  public static int TN_UR_y = -1;
  
  /**
   * Home area lower left coordinates
   */
  public static int HOME_LL_x = -1;
  public static int HOME_LL_y = -1;
  
  /**
   * Home area upper right coordinates
   */
  public static int HOME_UR_x = -1;
  public static int HOME_UR_y = -1;
  
  /**
   * Search area lower left coordinates
   */
  public static int SZ_LL_x = -1;
  public static int SZ_LL_y = -1;
  
  /**
   * Search area lower left coordinates
   */
  public static int SZ_UR_x = -1;
  public static int SZ_UR_y = -1;
  
  /**
   * Initial position coordinates
   */
  public static int INITPOS_x = -1;
  public static int INITPOS_y = -1;
  
  /**
   * The odometer.
   */
  public static Odometer odometer = Odometer.getOdometer();
  
  /**
   * Creates the robot instance
   */
  public static Robot robot = Robot.getRobot();
  
  
  /**
   * The left motor.
   */
  public static final EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(MotorPort.A);

  /**
   * The right motor.
   */
  public static final EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(MotorPort.D);
  
  /**
   * The right light sensor for localization
   */
  public static final EV3ColorSensor LIGHT_SENSOR_RIGHT =
      new EV3ColorSensor(LocalEV3.get().getPort("S3"));
  
  /**
   * The left light sensor for localization
   */
  public static final EV3ColorSensor LIGHT_SENSOR_LEFT = 
      new EV3ColorSensor(LocalEV3.get().getPort("S4"));
  
  /**
   * The light sensor for color detection
   */
  public static final EV3ColorSensor LIGHT_SENSOR_COLOR =
       new EV3ColorSensor(LocalEV3.get().getPort("S2"));
  
  /**
   * The ultrasonic sensor for localization
   */
  public static final EV3UltrasonicSensor US_SENSOR =
      new EV3UltrasonicSensor(LocalEV3.get().getPort("S3"));
  
  /**
   * Instantiation of sampler object for the right light sensor
   */
  public static final LightSensor rightLightSensor = new LightSensor(LIGHT_SENSOR_RIGHT, "Red");
  
  /**
   * Instantiation of sampler object for the left light sensor
   */
  public static final LightSensor leftLightSensor = new LightSensor(LIGHT_SENSOR_LEFT, "Red");
  
  /**
   * Instantiation of sampler object for the color detection light sensor
   */
  public static final LightSensor colorLightSensor = new LightSensor(LIGHT_SENSOR_COLOR, "RGB");
  
  /**
   * Convert a tile size into cm
   */
  public static double toCM (int tileSize) {
    return tileSize * TILE_SIZE;
  }
  
  public static void setInitPos(int x, int y) {
    INITPOS_x = x;
    INITPOS_y = y;
  }
  
}
