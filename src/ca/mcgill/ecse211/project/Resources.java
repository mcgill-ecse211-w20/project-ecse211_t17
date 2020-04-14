package ca.mcgill.ecse211.project;

import java.math.BigDecimal;
import java.util.Map;
import ca.mcgill.ecse211.wificlient.WifiConnection;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;

/**
 * This class is used to define static resources in one place for easy access and to avoid cluttering the rest of the
 * codebase. All values are either initialized here or in Robot.initialize(). 
 * All resources can be imported at once like this:
 * 
 * <p>
 * {@code import static ca.mcgill.ecse211.lab3.Resources.*;}
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
public class Resources {
  
  /**
   * The default server IP used by the profs and TA's.
   */
  public static final String DEFAULT_SERVER_IP = "192.168.2.3";
  
  /**
   * The IP address of the server that transmits data to the robot. For the beta demo and
   * competition, replace this line with
   * 
   * <p>{@code public static final String SERVER_IP = DEFAULT_SERVER_IP;}
   */
  public static final String SERVER_IP = "192.168.2.3"; // = DEFAULT_SERVER_IP;
  
  /**
   * Your team number.
   */
  public static final int TEAM_NUMBER = 17;
  
  /** 
   * Enables printing of debug info from the WiFi class. 
   */
  public static final boolean ENABLE_DEBUG_WIFI_PRINT = true;
  
  /**
   * Enable this to attempt to receive Wi-Fi parameters at the start of the program.
   */
  public static final boolean RECEIVE_WIFI_PARAMS = true;

  /**
   * Wheel ratio
   */
  private static final double RATIO = 0.9976;
  
  /**
   * The left wheel radius in centimeters.
   */
  public static final double WHEEL_RAD_LEFT = 2.130;
  /**
   * The right wheel radius in centimeters
   */
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

  /**
   * Enum to recognize team colours
   *
   */
  public enum Team { GREEN, RED };
  /**
   * Team storing details
   */
  public static Team team = null;
  
  /**
   * Starting corner number
   */
  public static double corner = -1;
  
  /**
   * Tunnel lower left x coordinate specific to team
   */
  public static double TN_LL_x = -1;
  /**
   * Tunnel lower left y coordinate specific to team
   */
  public static double TN_LL_y = -1;
  
  /**
   * Tunnel upper right x coordinate specific to team
   */
  public static double TN_UR_x = -1;
  /**
   * Tunnel upper right y coordinate specific to team
   */
  public static double TN_UR_y = -1;
  
  /**
   * Home area lower left x coordinate specific to team 
   */
  public static double HOME_LL_x = -1;
  /**
   * Home area lower left y coordinate specific to team 
   */
  public static double HOME_LL_y = -1;
  
  /**
   * Home area upper right x coordinate specific to team
   */
  public static double HOME_UR_x = -1;
  /**
   * Home area upper right y coordinate specific to team
   */
  public static double HOME_UR_y = -1;
  
  /**
   * Search area lower left x coordinate specific to team
   */
  public static double SZ_LL_x = -1;
  /**
   * Search area lower left y coordinate specific to team
   */
  public static double SZ_LL_y = -1;
  
  /**
   * Search area lower left x coordinate specific to team
   */
  public static double SZ_UR_x = -1;
  /**
   * Search area lower left y coordinate specific to team
   */
  public static double SZ_UR_y = -1;
  
  /**
   * Largest point on map x coordinate
   */
  public static double MAP_x = chooseLargest(Wifi.RED_UR_x, Wifi.GREEN_UR_x, Wifi.Island_UR_x);
  /**
   * Largest point on map y coordinate
   */
  public static double MAP_y = chooseLargest(Wifi.RED_UR_y, Wifi.GREEN_UR_y, Wifi.Island_UR_y);
  /**
   * Initial position coordinate x in tile size coordinate specific to team
   */
  public static double INITPOS_x = -1;
  /**
   * Initial position coordinate y in tile size coordinate specific to team
   */
  public static double INITPOS_y = -1;
  /**
   * Initial angle when the robot was facing north according to the actual north in degrees
   */
  public static double INIT_NORTH_ANGLE = -1;
  
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
   * The hooking motor.
   */
  public static final EV3LargeRegulatedMotor raiseMotor = new EV3LargeRegulatedMotor(MotorPort.B);
  
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
  public static final LightPoller rightLightPoller = new LightPoller(LIGHT_SENSOR_RIGHT, "Red");
  
  /**
   * Instantiation of sampler object for the left light sensor
   */
  public static final LightPoller leftLightPoller = new LightPoller(LIGHT_SENSOR_LEFT, "Red");
  
  /**
   * Instantiation of sampler object for the color detection light sensor
   */
  public static final LightPoller colorLightPoller = new LightPoller(LIGHT_SENSOR_COLOR, "RGB");
  
  /**
   * Convert a tile size into cm
   */
  public static double toCM (double tileSize) {
    return tileSize * TILE_SIZE;
  }
  
  /**
   * Setting the initial positions of the robot according after it reaches the nearest intersection
   * 
   * @param x in tile size coordinate 
   * @param y in tile size coordinate
   */
  public static void setInitPos(double x, double y) {
    INITPOS_x = x;
    INITPOS_y = y;
  }
  
  /**
   * Choose the largest value among the parameters and return it
   * 
   * @param rCoord coordinate from the red area in tile size coordinate
   * @param gCoord coordinate from the green area in tile size coordinate
   * @param iCoord coordinate from the island area in tile size coordinate
   * 
   * @return the largest coordinate from the parameter in tile size coordinate
   */
  private static int chooseLargest(int rCoord, int gCoord, int iCoord) {
    int temp = (rCoord > gCoord) ? rCoord : gCoord;
    temp = (temp > iCoord) ? temp : iCoord;
    return temp;
  }
  
  /**
   * Container for the Wi-Fi parameters.
   */
  public static Map<String, Object> wifiParameters;
  
  // This static initializer MUST be declared before any Wi-Fi parameters.
  static {
    receiveWifiParameters();
  }
  
  /** Red team number. */
  public static int redTeam = getWP("RedTeam");

  /** Red team's starting corner. */
  public static int redCorner = getWP("RedCorner");

  /** Green team number. */
  public static int greenTeam = getWP("GreenTeam");

  /** Green team's starting corner. */
  public static int greenCorner = getWP("GreenCorner");

  /** The Red Zone. */
  public static Region red = makeRegion("Red");

  /** The Green Zone. */
  public static Region green = makeRegion("Green");

  /** The Island. */
  public static Region island = makeRegion("Island");

  /** The red tunnel footprint. */
  public static Region tnr = makeRegion("TNR");

  /** The green tunnel footprint. */
  public static Region tng = makeRegion("TNG");

  /** The red search zone. */
  public static Region szr = makeRegion("SZR");

  /** The green search zone. */
  public static Region szg = makeRegion("SZG");
  
  /**
   * Receives Wi-Fi parameters from the server program.
   */
  public static void receiveWifiParameters() {
    // Only initialize the parameters if needed
    if (!RECEIVE_WIFI_PARAMS || wifiParameters != null) {
      return;
    }
    System.out.println("Waiting to receive Wi-Fi parameters.");

    // Connect to server and get the data, catching any errors that might occur
    try (WifiConnection conn =
        new WifiConnection(SERVER_IP, TEAM_NUMBER, ENABLE_DEBUG_WIFI_PRINT)) {
      /*
       * getData() will connect to the server and wait until the user/TA presses the "Start" button
       * in the GUI on their laptop with the data filled in. Once it's waiting, you can kill it by
       * pressing the back/escape button on the EV3. getData() will throw exceptions if something
       * goes wrong.
       */
      wifiParameters = conn.getData();
    } catch (Exception e) {
      System.err.println("Error: " + e.getMessage());
    }
  }
  
  /**
   * Returns the Wi-Fi parameter int value associated with the given key.
   * 
   * @param key the Wi-Fi parameter key
   * @return the Wi-Fi parameter int value associated with the given key
   */
  public static int getWP(String key) {
    if (wifiParameters != null) {
      return ((BigDecimal) wifiParameters.get(key)).intValue();
    } else {
      return 0;
    }
  }
  
  /** 
   * Makes a point given a Wi-Fi parameter prefix.
   */
  public static Point makePoint(String paramPrefix) {
    return new Point(getWP(paramPrefix + "_x"), getWP(paramPrefix + "_y"));
  }
  
  /**
   * Makes a region given a Wi-Fi parameter prefix.
   */
  public static Region makeRegion(String paramPrefix) {
    return new Region(makePoint(paramPrefix + "_LL"), makePoint(paramPrefix + "_UR"));
  }
  
}
