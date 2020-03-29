package ca.mcgill.ecse211.project;

import ca.mcgill.ecse211.project.Resources.Team;
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
    TOSEARCH,
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
  private static boolean isLookingObstacles;
  
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
      case TOSEARCH:
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
    
    Team colour;
    
    do {
      colour = ColorDetector.DetectColor();
      if (colour == Team.RED) {
        
        Resources.team = Resources.Team.RED;
        Resources.corner = Wifi.RedCorner;
        
        Resources.TN_LL_x = Wifi.TNR_LL_x;
        Resources.TN_LL_y = Wifi.TNR_LL_y;
        Resources.TN_UR_x = Wifi.TNR_UR_x;
        Resources.TN_UR_y = Wifi.TNR_UR_y;
        
        Resources.HOME_LL_x = Wifi.RED_LL_x;
        Resources.HOME_LL_y = Wifi.RED_LL_y;
        Resources.HOME_UR_x = Wifi.RED_UR_x;
        Resources.HOME_UR_y = Wifi.RED_UR_y;
      } else if (colour == Team.GREEN) {
        
        Resources.team = Resources.Team.GREEN;
        Resources.corner = Wifi.GreenCorner;
        
        Resources.TN_LL_x = Wifi.TNG_LL_x;
        Resources.TN_LL_y = Wifi.TNG_LL_y;
        Resources.TN_UR_x = Wifi.TNG_UR_x;
        Resources.TN_UR_y = Wifi.TNG_UR_y;
        
        Resources.HOME_LL_x = Wifi.RED_LL_x;
        Resources.HOME_LL_y = Wifi.RED_LL_y;
        Resources.HOME_UR_x = Wifi.RED_UR_x;
        Resources.HOME_UR_y = Wifi.RED_UR_y;
      }
      
    } while (Resources.team == null);
    
    UltrasonicSensor.reorient();
    UltrasonicSensor.moveToOrigin();
    updateState(State.TRAVELING);
  }
  
  /**
   * Movement to search using the data from the Wifi class
   */
  public static void toSearch() {
    
    
    double xCenterHome = (double)(Resources.HOME_LL_x + Resources.HOME_UR_x)/2.0;
    double yCenterHome = (double)(Resources.HOME_LL_y + Resources.HOME_UR_y)/2.0;
    double xCenterIsland = (double)(Wifi.Island_LL_x + Wifi.Island_UR_x)/2.0;
    double yCenterIsland = (double)(Wifi.Island_LL_y + Wifi.Island_UR_y)/2.0;
    
    double xDiff = xCenterHome - xCenterIsland;
    double yDiff = yCenterHome - yCenterIsland;
    
    if (xDiff > yDiff) {
      direction = Direction.Horizontal;
      bridgeSize = Resources.TN_UR_x - Resources.TN_LL_x;
    } else { //both when the xDiff is smaller and when its equal the bridge is vertical
      direction = Direction.Vertical;
      bridgeSize = Resources.TN_UR_y - Resources.TN_LL_y;
    } 
    
    if (direction == Direction.Horizontal) {
      //Starting corner is on the right
      if (Resources.corner == 1 || Resources.corner == 2) {
        
        Movement.travelTo(Resources.TN_UR_x + 1, Resources.TN_LL_y, isLookingObstacles);
        Movement.turnTo(0.0);
        Movement.goForward(Resources.TILE_SIZE/2);
        Movement.turnTo(-90.0);
        updateState(State.CROSSING);
        Movement.goForward(Resources.toCM(bridgeSize + 2));
        Movement.turnTo(180.0);
        Movement.goForward(Resources.TILE_SIZE/2);
        
        double fixedAngle = localize();
        Resources.odometer.setXyt(Resources.toCM(Resources.TN_LL_x -1), Resources.toCM(Resources.TN_LL_y), fixedAngle);
        
        updateState(State.TOSEARCH);
        
        
      } else { //Starting corner is on the left
        Movement.travelTo(Resources.TN_LL_x - 1, Resources.TN_LL_y, isLookingObstacles);
        Movement.turnTo(0.0);
        Movement.goForward(Resources.TILE_SIZE/2);
        Movement.turnTo(90.0);
        updateState(State.CROSSING);
        Movement.goForward(Resources.toCM(bridgeSize + 2));
        Movement.turnTo(180.0);
        Movement.goForward(Resources.TILE_SIZE/2);
        
        double fixedAngle = localize();
        Resources.odometer.setXyt(Resources.toCM(Resources.TN_UR_x  + 1), Resources.toCM(Resources.TN_LL_y), fixedAngle);
        
        updateState(State.TOSEARCH);
        
      }
      
      
    } else if (direction == Direction.Vertical) {
      
    //Starting corner is above
      if (Resources.corner == 2 || Resources.corner == 3) {
        
        Movement.travelTo(Resources.TN_UR_x, Resources.TN_UR_y + 1, isLookingObstacles);
        Movement.turnTo(-90.0);
        Movement.goForward(Resources.TILE_SIZE/2);
        Movement.turnTo(180.0);
        updateState(State.CROSSING);
        Movement.goForward(Resources.toCM(bridgeSize + 2));
        Movement.turnTo(90.0);
        Movement.goForward(Resources.TILE_SIZE/2);
        
        double fixedAngle = localize();
        Resources.odometer.setXyt(Resources.toCM(Resources.TN_UR_x), Resources.toCM(Resources.TN_LL_y - 1), fixedAngle);
        
        updateState(State.TOSEARCH);
        
      } else { //Starting corner is below
        Movement.travelTo(Resources.TN_UR_x, Resources.TN_LL_y - 1, isLookingObstacles);
        Movement.turnTo(-90.0);
        Movement.goForward(Resources.TILE_SIZE/2);
        Movement.turnTo(0.0);
        updateState(State.CROSSING);
        Movement.goForward(Resources.toCM(bridgeSize + 2));
        Movement.turnTo(90.0);
        Movement.goForward(Resources.TILE_SIZE/2);
        
        double fixedAngle = localize();
        Resources.odometer.setXyt(Resources.toCM(Resources.TN_UR_x), Resources.toCM(Resources.TN_UR_y + 1), fixedAngle);
        
        updateState(State.TOSEARCH);
        
      }
    }
    
    Movement.travelTo(Resources.SZ_LL_x, Resources.SZ_LL_y, isLookingObstacles);
    updateState(State.SEARCHING);
    beeps(3);
    //Movement.stopMotors();
  }
  
  /**
   * Only localize according to the lightsensor
   */
  public static double localize() {
    return 0.0;
  }
  
  /**
   * Search the area with the data from wifi class
   */
  public static void search() {
    
    int xLength = Resources.SZ_UR_x - Resources.SZ_LL_x;
    int yLength = Resources.SZ_UR_y - Resources.SZ_LL_y - 1;
    
    Movement.turnTo(0.0);
    Movement.goForward(Resources.TILE_SIZE/2);
    
    int yCounter = 0;
    double angle;
    while (yCounter <= yLength && !foundCart) {
      angle = (yCounter % 2 == 0) ? 90.0 : -90.0;
      Movement.turnTo(angle);
      Movement.moveForwardSearch(Resources.toCM(xLength));
      Movement.turnTo(0.0);
      Movement.moveForwardSearch(Resources.TILE_SIZE/2);
    }
    
    if (foundCart) {
      Movement.hookCart();
    } else {
      //sad
    }
    updateState(State.TOSEARCH);
  }
  
  /**
   * Movement back to home area
   */
  public static void toHome() {
    
    if (direction == Direction.Horizontal) {
      //Island is on the left
      if (Resources.corner == 1 || Resources.corner == 2) {
        
        Movement.travelTo(Resources.TN_LL_x - 1, Resources.TN_LL_y, isLookingObstacles);
        Movement.turnTo(0.0);
        Movement.goForward(Resources.TILE_SIZE/2);
        Movement.turnTo(90.0);
        updateState(State.CROSSING);
        Movement.goForward(Resources.toCM(bridgeSize + 2));
        Movement.turnTo(180.0);
        Movement.goForward(Resources.TILE_SIZE/2);
        
        double fixedAngle = localize();
        Resources.odometer.setXyt(Resources.toCM(Resources.TN_UR_x + 1), Resources.toCM(Resources.TN_LL_y), fixedAngle);
        
        updateState(State.TRAVELING);
        
      } else { //Island is on the right
        Movement.travelTo(Resources.TN_LL_x + 1, Resources.TN_LL_y, isLookingObstacles);
        Movement.turnTo(0.0);
        Movement.goForward(Resources.TILE_SIZE/2);
        Movement.turnTo(-90.0);
        updateState(State.CROSSING);
        Movement.goForward(Resources.toCM(bridgeSize + 2));
        Movement.turnTo(180.0);
        Movement.goForward(Resources.TILE_SIZE/2);
        
        double fixedAngle = localize();
        Resources.odometer.setXyt(Resources.toCM(Resources.TN_LL_x  - 1), Resources.toCM(Resources.TN_LL_y), fixedAngle);
        
        updateState(State.TRAVELING);
        
      }
      
      
    } else if (direction == Direction.Vertical) {
      
    //Island is below
      if (Resources.corner == 2 || Resources.corner == 3) {
        
        Movement.travelTo(Resources.TN_UR_x, Resources.TN_LL_y - 1, isLookingObstacles);
        Movement.turnTo(-90.0);
        Movement.goForward(Resources.TILE_SIZE/2);
        Movement.turnTo(0.0);
        updateState(State.CROSSING);
        Movement.goForward(Resources.toCM(bridgeSize + 2));
        Movement.turnTo(90.0);
        Movement.goForward(Resources.TILE_SIZE/2);
        
        double fixedAngle = localize();
        Resources.odometer.setXyt(Resources.toCM(Resources.TN_UR_x), Resources.toCM(Resources.TN_UR_y + 1), fixedAngle);
        
        updateState(State.TRAVELING);
        
      } else { //Island is above
        Movement.travelTo(Resources.TN_UR_x, Resources.TN_UR_y + 1, isLookingObstacles);
        Movement.turnTo(-90.0);
        Movement.goForward(Resources.TILE_SIZE/2);
        Movement.turnTo(180.0);
        updateState(State.CROSSING);
        Movement.goForward(Resources.toCM(bridgeSize + 2));
        Movement.turnTo(90.0);
        Movement.goForward(Resources.TILE_SIZE/2);
        
        double fixedAngle = localize();
        Resources.odometer.setXyt(Resources.toCM(Resources.TN_UR_x), Resources.toCM(Resources.TN_LL_y - 1), fixedAngle);
        
        updateState(State.TRAVELING);
        
      }
    }
    
    Movement.travelTo(Resources.INITPOS_x, Resources.INITPOS_y, isLookingObstacles);
    Movement.turnTo(90);
    Movement.goForward(xBeginCoord);
    Movement.turnTo(180);
    Movement.goForward(yBeginCoord);
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
