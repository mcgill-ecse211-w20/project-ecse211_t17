package ca.mcgill.ecse211.project;

import static ca.mcgill.ecse211.project.Resources.*;
import lejos.hardware.Sound;

/**
 * Intermediate between the main class and the other classes.
 * Implemented using a state machine to determine which methods to call in what state
 * 
 * @author Xinyue Chen
 * @author Zheng Yu Cui
 * @author Alixe Delabrousse
 * @author David Kronish
 * @author Ariane Leroux
 *
 */
public class Robot {
  
  /**
   * Enumeration class listing the five states the robot can be ins
   *
   */
  public enum State {
    IDLE,
    TRAVELING,
    CROSSING,
    ONISLAND,
    SEARCHING,
    IDENTIFICATION
  }
  
  // booleans to regulate the running of Threads
  /**
   * Set to true when the robot is in the TRAVELING and SEARCHING states
   */
  public static boolean isNavigating;
  
  /**
   * Set to true when the robot is in the CROSSING state
   */
  public static boolean crossBridge;
  
  /**
   * Set to true when the robot is in the SEARCHING and IDENTIFICATION states
   */
  public static boolean isSearching;
  
  /**
   * Current state of the robot
   */
  public static State robotStatus;
  
  /**
   * Set to true if it's looking for obstacles
   */
  public static boolean isLookingObstacles;
  
  /**
   * orientation of bridge
   */
  private enum Direction { Vertical, Horizontal };
  
  /**
   * actual orientation of bridge
   */
  public static Direction direction;

  /**
   * Initializing the state machine and the static references to various detection and movement objects
   */
  public Robot() {
    Robot.updateState(State.IDLE);
  }
  
  public static synchronized Robot getRobot() {
    return new Robot();
  }
  
  private static boolean foundCart;
  
//in coordinate system
  private static int bridgeSize;
  
  private static double xBeginCoord;
  private static double yBeginCoord;
  
  /**
   * Updates the current state of the robot object as well as the corresponding booleans
   * 
   * @param newState - the new state of the device
   */
  public static void updateState(State newState){
    robotStatus = newState;
    switch (newState) {
      case IDLE:
        isNavigating = false;
        isSearching = false;
        crossBridge = false;
        isLookingObstacles = false;
        break;
      case TRAVELING:
        isNavigating = true;
        isSearching = false;
        crossBridge = false;
        isLookingObstacles = false;
        break;
      case CROSSING:
        isNavigating = false;
        isSearching = false;
        crossBridge = true;
        isLookingObstacles = false;
        break;
      case ONISLAND:
        isNavigating = true;
        isSearching = false;
        isLookingObstacles = true;
        crossBridge = false;
        break;
      case SEARCHING:
        isNavigating = true;
        isSearching = true;
        crossBridge = false;
        isLookingObstacles = true;
        break;
      case IDENTIFICATION:
        isNavigating = false;
        isSearching = true;
        crossBridge = false;
        isLookingObstacles = true;
        break;
    }   
  }
  
  
  /**
   * Initializes the robot's position and orienting it so it faces 0deg
   */
  public static void initialize() {
    
    int colour;
    
    do {
      colour = ColorDetector.DetectColor();
      if (colour == 0) {
        
        team = Team.RED;
        corner = Wifi.RedCorner;
        
        TN_LL_x = Wifi.TNR_LL_x;
        TN_LL_y = Wifi.TNR_LL_y;
        TN_UR_x = Wifi.TNR_UR_x;
        TN_UR_y = Wifi.TNR_UR_y;
        
        HOME_LL_x = Wifi.RED_LL_x;
        HOME_LL_y = Wifi.RED_LL_y;
        HOME_UR_x = Wifi.RED_UR_x;
        HOME_UR_y = Wifi.RED_UR_y;
        
        SZ_LL_x = Wifi.SZR_LL_x;
        SZ_LL_y = Wifi.SZR_LL_y;
        SZ_UR_x = Wifi.SZR_UR_x;
        SZ_UR_y = Wifi.SZR_UR_y;
      } else if (colour == 1) {
        
        team = Team.GREEN;
        corner = Wifi.GreenCorner;
        
        TN_LL_x = Wifi.TNG_LL_x;
        TN_LL_y = Wifi.TNG_LL_y;
        TN_UR_x = Wifi.TNG_UR_x;
        TN_UR_y = Wifi.TNG_UR_y;
        
        HOME_LL_x = Wifi.RED_LL_x;
        HOME_LL_y = Wifi.RED_LL_y;
        HOME_UR_x = Wifi.RED_UR_x;
        HOME_UR_y = Wifi.RED_UR_y;
        
        SZ_LL_x = Wifi.SZR_LL_x;
        SZ_LL_y = Wifi.SZR_LL_y;
        SZ_UR_x = Wifi.SZR_UR_x;
        SZ_UR_y = Wifi.SZR_UR_y;
      }
      
    } while (team == null);
    
    UltrasonicSensor.reorient();
    UltrasonicSensor.moveToOrigin();
    updateState(State.TRAVELING);
  }
  
  /**
   * Movement to search using the data from the Wifi class
   */
  public static void toSearch() {
    
    
    double xCenterHome = (double)(HOME_LL_x + HOME_UR_x)/2.0;
    double yCenterHome = (double)(HOME_LL_y + HOME_UR_y)/2.0;
    double xCenterIsland = (double)(Wifi.Island_LL_x + Wifi.Island_UR_x)/2.0;
    double yCenterIsland = (double)(Wifi.Island_LL_y + Wifi.Island_UR_y)/2.0;
    
    double xDiff = xCenterHome - xCenterIsland;
    double yDiff = yCenterHome - yCenterIsland;
    
    if (xDiff > yDiff) {
      direction = Direction.Horizontal;
      bridgeSize = TN_UR_x - TN_LL_x;
    } else { //both when the xDiff is smaller and when its equal the bridge is vertical
      direction = Direction.Vertical;
      bridgeSize = TN_UR_y - TN_LL_y;
    } 
    
    if (direction == Direction.Horizontal) {
      //Starting corner is on the right
      if (corner == 1 || corner == 2) {
        
        Movement.travelTo(TN_UR_x + 1, TN_LL_y, isLookingObstacles);
        Movement.turnTo(0.0);
        Movement.goForward(TILE_SIZE/2);
        Movement.turnTo(-90.0);
        updateState(State.CROSSING);
        Movement.goForward(toCM(bridgeSize + 2));
        Movement.turnTo(180.0);
        Movement.goForward(TILE_SIZE/2);
        
        localize();
        odometer.setXyt(toCM(TN_LL_x -1), toCM(TN_LL_y), 0.0);
        
        updateState(State.ONISLAND);
        
        
      } else { //Starting corner is on the left
        Movement.travelTo(TN_LL_x - 1, TN_LL_y, isLookingObstacles);
        Movement.turnTo(0.0);
        Movement.goForward(TILE_SIZE/2);
        Movement.turnTo(90.0);
        updateState(State.CROSSING);
        Movement.goForward(toCM(bridgeSize + 2));
        Movement.turnTo(180.0);
        Movement.goForward(TILE_SIZE/2);
        
        localize();
        odometer.setXyt(toCM(TN_UR_x  + 1), toCM(TN_LL_y), 0.0);
        
        updateState(State.ONISLAND);
        
      }
      
      
    } else if (direction == Direction.Vertical) {
      
    //Starting corner is above
      if (corner == 2 || corner == 3) {
        
        Movement.travelTo(TN_UR_x, TN_UR_y + 1, isLookingObstacles);
        Movement.turnTo(-90.0);
        Movement.goForward(TILE_SIZE/2);
        Movement.turnTo(180.0);
        updateState(State.CROSSING);
        Movement.goForward(toCM(bridgeSize + 2));
        Movement.turnTo(90.0);
        Movement.goForward(TILE_SIZE/2);
        
        localize();
        odometer.setXyt(toCM(TN_UR_x), toCM(TN_LL_y - 1), 0.0);
        
        updateState(State.ONISLAND);
        
      } else { //Starting corner is below
        Movement.travelTo(TN_UR_x, TN_LL_y - 1, isLookingObstacles);
        Movement.turnTo(-90.0);
        Movement.goForward(TILE_SIZE/2);
        Movement.turnTo(0.0);
        updateState(State.CROSSING);
        Movement.goForward(toCM(bridgeSize + 2));
        Movement.turnTo(90.0);
        Movement.goForward(TILE_SIZE/2);
        
        localize();
        odometer.setXyt(toCM(TN_UR_x), toCM(TN_UR_y + 1), 0.0);
        
        updateState(State.ONISLAND);
        
      }
    }
    
    Movement.travelTo(SZ_LL_x, SZ_LL_y, isLookingObstacles);
    updateState(State.SEARCHING);
    beeps(3);
    //Movement.stopMotors();
  }
  
  /**
   * Only localize according to the lightsensor
   * Reason why not two same time from lightsensor is bc too many threads
   */
  public static void localize() {
    Movement.turnTo(45);
    
    float rightValue = rightLightPoller.getColor();
    float leftValue = leftLightPoller.getColor();
    
    boolean rPassed = false;
    boolean lPassed = false;
    
    double[] rAngles = {-1, -1, -1, -1};
    double[] lAngles = {-1, -1, -1, -1};
    double[] avgAngles = new double[4];
    
    //Counters counting how many lines are stored in the arrays
    int r = 0;
    int l = 0;
    
    //start rotation and clock all 4 gridlines until all 4 are obtained
    while (rAngles[3] == -1 && lAngles[3] == -1) {
      //Get sensor values
      rightValue = rightLightPoller.getColor();
      leftValue = leftLightPoller.getColor();
      
      //when right sensor first encounters the line
      if (rightValue > 0.4) {
        rPassed = false;
        //if it passed a line then set the passed to true and store data in array
      } else if (rightValue < 0.35 && !rPassed && rAngles[r] == -1) {
        rAngles[r] = odometer.getTheta() + 52.0; // adjustment because it is on the right
        rPassed = true;
        r++;
      }
      
      //when left sensor first encounters the line
      if (leftValue > 0.4) {
        lPassed = false;
        //if it passed a line then set the passed to true and store data in array
      } else if (leftValue < 0.35 && !lPassed && lAngles[l] == -1) {
        lAngles[l] = odometer.getTheta() - 53.0; // adjustment because it is on the right
        lPassed = true;
        l++;
      }
      
      //Rotate as long as not all four lines are found
      Movement.rotateCounterClockwise();
    }
    
    /**
     * ========================CALCULATIONS==============================
     */
    //obtaining average angle for each 
    for (int i = 0; i<4; i++) {
      avgAngles[i] = (makePositive(rAngles[i]) + makePositive(lAngles[i]))/2.0;
    }
    
    double thetaY = makeObtuse(Math.abs(avgAngles[0] - avgAngles[2]));
    double thetaX = makeObtuse(Math.abs(rAngles[1] - rAngles[3]));
    double x= LIGHT_SENSOR_DISTANCE * Math.cos(Math.toRadians(thetaY/2));
    double y = LIGHT_SENSOR_DISTANCE * Math.cos(Math.toRadians(thetaX)/2);
    
    odometer.setXyt(x, y, odometer.getTheta());
    //Move to the crossing of lines and rotate back to 0deg
    Movement.travelTo(0.0, 0.0);
    Movement.turnTo(fixAngle(avgAngles[0], avgAngles[2]) +180.0);
  }
  
  private static double makePositive(double angle) {
    return (angle < 0) ? angle + 360 : angle;
  }
  private static double makeObtuse(double angle) {
    return (angle > 180) ? 360-angle : angle;
  }
  private static double fixAngle(double a, double b) {
    double diff = a - b;
    diff = (diff < 0) ? (diff + 360.0) : diff;
    if (diff > 180) {
      diff = 360.0 - diff;
    }
    diff = (180 - diff)/2.0;
    diff = diff + a;
    diff = (diff > 180) ? (diff - 360) : diff;
    return diff;
  }
  /**
   * Search the area with the data from wifi class
   */
  public static void search() {
    
    int xLength = SZ_UR_x - SZ_LL_x;
    int yLength = SZ_UR_y - SZ_LL_y - 1;
    
    Movement.turnTo(0.0);
    Movement.goForward(TILE_SIZE/2);
    
    int yCounter = 0;
    double angle;
    while (yCounter <= yLength && !foundCart) {
      angle = (yCounter % 2 == 0) ? 90.0 : -90.0;
      Movement.turnTo(angle);
      Movement.moveForwardSearch(toCM(xLength));
      Movement.turnTo(0.0);
      Movement.moveForwardSearch(TILE_SIZE/2);
    }
    
    if (foundCart) {
      orientPerpendicular();
      Movement.raiseCart();
    } else {
      //sad
    }
    updateState(State.ONISLAND);
  }
  
  /**
   * Movement back to home area
   */
  public static void toHome() {
    
    if (direction == Direction.Horizontal) {
      //Island is on the left
      if (corner == 1 || corner == 2) {
        
        Movement.travelTo(TN_LL_x - 1, TN_LL_y, isLookingObstacles);
        Movement.turnTo(0.0);
        Movement.goForward(TILE_SIZE/2);
        Movement.turnTo(90.0);
        updateState(State.CROSSING);
        Movement.goForward(toCM(bridgeSize + 2));
        Movement.turnTo(180.0);
        Movement.goForward(TILE_SIZE/2);
        
        localize();
        odometer.setXyt(toCM(TN_UR_x + 1), toCM(TN_LL_y), 0.0);
        
        updateState(State.TRAVELING);
        
      } else { //Island is on the right
        Movement.travelTo(TN_LL_x + 1, TN_LL_y, isLookingObstacles);
        Movement.turnTo(0.0);
        Movement.goForward(TILE_SIZE/2);
        Movement.turnTo(-90.0);
        updateState(State.CROSSING);
        Movement.goForward(toCM(bridgeSize + 2));
        Movement.turnTo(180.0);
        Movement.goForward(TILE_SIZE/2);
        
        localize();
        odometer.setXyt(toCM(TN_LL_x  - 1), toCM(TN_LL_y), 0.0);
        
        updateState(State.TRAVELING);
        
      }
      
      
    } else if (direction == Direction.Vertical) {
      
    //Island is below
      if (corner == 2 || corner == 3) {
        
        Movement.travelTo(TN_UR_x, TN_LL_y - 1, isLookingObstacles);
        Movement.turnTo(-90.0);
        Movement.goForward(TILE_SIZE/2);
        Movement.turnTo(0.0);
        updateState(State.CROSSING);
        Movement.goForward(toCM(bridgeSize + 2));
        Movement.turnTo(90.0);
        Movement.goForward(TILE_SIZE/2);
        
        localize();
        odometer.setXyt(toCM(TN_UR_x), toCM(TN_UR_y + 1), 0.0);
        
        updateState(State.TRAVELING);
        
      } else { //Island is above
        Movement.travelTo(TN_UR_x, TN_UR_y + 1, isLookingObstacles);
        Movement.turnTo(-90.0);
        Movement.goForward(TILE_SIZE/2);
        Movement.turnTo(180.0);
        updateState(State.CROSSING);
        Movement.goForward(toCM(bridgeSize + 2));
        Movement.turnTo(90.0);
        Movement.goForward(TILE_SIZE/2);
        
        localize();
        odometer.setXyt(toCM(TN_UR_x), toCM(TN_LL_y - 1), 0.0);
        
        updateState(State.TRAVELING);
        
      }
    }
    /**The arrow is pointing towards the INIT_NORTH_ANGLE
     * 3-->      |         2
     *           |         |
     *           |         V
     *           |
     * __________|__________
     *           |          
     *           |
     * ^         |
     * |         |
     * 0         |     <-- 1       
     */
    Movement.travelTo(INITPOS_x, INITPOS_y, isLookingObstacles);
    
    Movement.turnTo(INIT_NORTH_ANGLE);
    Movement.turnBy(-90.0);
    Movement.goForward(TILE_SIZE - xBeginCoord);
    Movement.turnBy(-90.0);
    Movement.goForward(TILE_SIZE - yBeginCoord);
    //Movement.stopMotors();
  }
  
  /**
   * Beeps specific amount times
   */
  public static void beeps(int times) {
    for (int i = 0; i<times; i++) {
      Sound.beep();
    }
  }
  
  public static void orientPerpendicular() {
    double initDist;
    double finalDist;
    double diff;
    double angle;
    
    initDist = UltrasonicSensor.getDistance();
    Movement.turnBy(-90.0);
    Movement.goForward(8.0);
    Movement.turnBy(90.0);
    finalDist = UltrasonicSensor.getDistance();
    
    if (finalDist < TILE_SIZE) {
      diff = finalDist - initDist;
      angle = 90.0 + Math.toDegrees(Math.atan(diff/8.0));
      if (angle < 0) {
        Movement.turnBy(90.0);
        Movement.goForward(8.0);
      }
      
    } else {
      Movement.turnBy(90.0);
      Movement.goForward(16.0);
      finalDist = UltrasonicSensor.getDistance();
      
      diff = finalDist - initDist;
      angle = 90.0 + Math.atan(diff/8.0);
      if (angle > 0) {
        Movement.turnBy(90.0);
        Movement.goForward(8.0);
      }
      
    }

    Movement.turnTo(angle);
    
  }
  
  /**
   * Setting the foundCart boolean
   * 
   * @param flag the boolean to which foundCart needs to be set to
   */
  public static void setFoundCart(boolean flag) {
    foundCart = flag;
  }
  
  /**
   * Getting the foundCart boolean
   * 
   * @return foundCart, true if robot found the cart, false if it hasn't
   */
  public static boolean getFoundCart() {
    return foundCart;
  }
  
  public static void setXBeginCoord(double x) {
    xBeginCoord = x;
  }
  
  public static double getXBeginCoord() {
    return xBeginCoord;
  }
  
  public static void setYBeginCoord(double y) {
    yBeginCoord = y;
  }
  
  public static double getYBeginCoord() {
    return yBeginCoord;
  }
}
