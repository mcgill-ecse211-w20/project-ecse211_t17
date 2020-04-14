package ca.mcgill.ecse211.project;

import static ca.mcgill.ecse211.project.Resources.*;
import lejos.hardware.Sound;

/**
 * Static class that coordinates all actions by using the other resources.
 * Implemented using a state machine to help the pollers determining when they should stop.
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
public class Robot {
  
  /**
   * Enumeration class listing the five states the robot can be in
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
  
  /**
   * Current state of the robot
   */
  public static State robotStatus;
  
  /**
   * orientation of bridge
   */
  private enum Direction { Vertical, Horizontal };
  
  /**
   * actual orientation of bridge
   */
  public static Direction direction;
  
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
   * Set to true if it's looking for obstacles
   */
  public static boolean isLookingObstacles;
  
  /**
   * Set to true once the robot found the cart
   */
  private static boolean foundCart;
  
  /**
   * Size of the bridge in tile size
   */
  private static double bridgeSize;
  
  /**
   * Robot's initial x coordinate relative to the nearest corner
   */
  private static double xBeginCoord;
  
  /**
   * Robot's initial y coordinate relative to the nearest corner
   */
  private static double yBeginCoord;

  /**
   * Initializing the state machine and the static references to various detection and movement objects
   */
  public Robot() {
    Robot.updateState(State.IDLE);
  }
  
  public static synchronized Robot getRobot() {
    return new Robot();
  }
  
  
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
   * Initializes the robot's position and imports all the wifi data
   * <p>
   * Verifies which team we are and initializes the general parameter in Resources accordingly.
   * <ul>
   *    <li>Tunnel LL and UR</li>
   *    <li>Home LL and UR</li>
   *    <li>Search Zone LL and UR</li>
   * </ul>
   */
  public static void initialize() {
    
    do {
      if (redTeam == TEAM_NUMBER) {
        
        team = Team.RED;
        corner = redCorner;
        
        TN_LL_x = tnr.ll.x;
        TN_LL_y = tnr.ll.y;
        TN_UR_x = tnr.ur.x;
        TN_UR_y = tnr.ur.y;
        
        HOME_LL_x = red.ll.x;
        HOME_LL_y = red.ll.y;
        HOME_UR_x = red.ur.x;
        HOME_UR_y = red.ur.y;
        
        SZ_LL_x = szr.ll.x;
        SZ_LL_y = szr.ll.y;
        SZ_UR_x = szr.ur.x;
        SZ_UR_y = szr.ur.y;
      } else if (greenTeam == TEAM_NUMBER) {
        
        team = Team.GREEN;
        corner = Wifi.GreenCorner;
        
        TN_LL_x = tng.ll.x;
        TN_LL_y = tng.ll.y;
        TN_UR_x = tng.ur.x;
        TN_UR_y = tng.ur.y;
        
        HOME_LL_x = green.ll.x;
        HOME_LL_y = green.ll.y;
        HOME_UR_x = green.ur.x;
        HOME_UR_y = green.ur.y;
        
        SZ_LL_x = szg.ll.x;
        SZ_LL_y = szg.ll.y;
        SZ_UR_x = szg.ur.x;
        SZ_UR_y = szg.ur.y;
      }
      
    } while (team == null);
    
    UltrasonicSensor.reorient();
    UltrasonicSensor.moveToOrigin();
    updateState(State.TRAVELING);
  }
  
  /**
   * Moves the robot to the search area by taking into consideration the orientation of the bridge and from which entrance
   * it should enter according to the imports from the Wifi class to the Resources class.
   * After exiting the bridge, localizes itself using the lightsensors
   * <p>
   * Finds the orientation of the bridge using the deltaX and deltaY of the segment connecting the the centre of the island and the centre of HOME.
   * Although it works for most layouts of the map, the drawback remains that if the regions are marginal, this method only works if one side is longer than the other by 
   * no more than 6 units. (Explained in more details in the software document) 
   * <p>
   * According to the starting corner, determine from which side the robot should approach the bridge. 
   * <br>
   * <ul>
   *    <li>If vertical: approach from lower right OR upper right</li>
   *    <li>If horizontal: approach from lower right OR lower left</li>
   * </ul>
   * <p>
   * After going through the tunnel, the robot light localizes by using the turning on a corner method. The live localization
   * during navigation is then turned off. However, the obstacle detection is on (without cart detection). The robot then moves to the lower left corner of the
   * search zone to begin search.
   */
  public static void toSearch() {
    
    double bridgeDiffX = TN_UR_x - TN_LL_x;
    double bridgeDiffY = TN_UR_y - TN_LL_y;
    
    
    if (bridgeDiffX == 1 && bridgeDiffY == 1 ) {
      
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
    } else if (bridgeDiffX > bridgeDiffY) {
      direction = Direction.Horizontal;
      bridgeSize = TN_UR_x - TN_LL_x;
    } else {
      direction = Direction.Vertical;
      bridgeSize = TN_UR_y - TN_LL_y;
    }
    
    
    leftLightPoller.start();
    rightLightPoller.start();
    
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
                
        updateState(State.ONISLAND);

        localize();
        odometer.setXyt(toCM(TN_LL_x -1), toCM(TN_LL_y), 0.0);
        isNavigating = false;
        
      } else { //Starting corner is on the left
        Movement.travelTo(TN_LL_x - 1, TN_LL_y, isLookingObstacles);
        Movement.turnTo(0.0);
        Movement.goForward(TILE_SIZE/2);
        Movement.turnTo(90.0);
        updateState(State.CROSSING);
        Movement.goForward(toCM(bridgeSize + 2));
        Movement.turnTo(180.0);
        Movement.goForward(TILE_SIZE/2);
                
        updateState(State.ONISLAND);
        

        localize();
        odometer.setXyt(toCM(TN_UR_x  + 1), toCM(TN_LL_y), 0.0);
        isNavigating = false;
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
        
        
        updateState(State.ONISLAND);

        localize();
        odometer.setXyt(toCM(TN_UR_x), toCM(TN_LL_y - 1), 0.0);
        isNavigating = false;
      } else { //Starting corner is below
        Movement.travelTo(TN_UR_x, TN_LL_y - 1, isLookingObstacles);
        Movement.turnTo(-90.0);
        Movement.goForward(TILE_SIZE/2);
        Movement.turnTo(0.0);
        updateState(State.CROSSING);
        Movement.goForward(toCM(bridgeSize + 2));
        Movement.turnTo(90.0);
        Movement.goForward(TILE_SIZE/2);
        
        
        updateState(State.ONISLAND);

        localize();
        odometer.setXyt(toCM(TN_UR_x), toCM(TN_UR_y + 1), 0.0);
        isNavigating = false;
      }
    }
    
    UltrasonicSensor.usSensorPoller();
    Movement.travelTo(SZ_LL_x, SZ_LL_y, isLookingObstacles);
    updateState(State.SEARCHING);
    beeps(3);
    //Movement.stopMotors();
  }
  
  /**
   * Helper method used after crossing the bridge to make sure the odometer is accurate.
   * Uses the two lightsensors to detect the lines while rotating.
   * 
   * <b>Note: The reason why this method wasn't implemented in the LighPoller class to have a nonstatic method is because
   * of the issue of too many threads</b>
   */
  private static void localize() {
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
    double thetaX = makeObtuse(Math.abs(avgAngles[1] - avgAngles[3]));
    double x= LIGHT_SENSOR_DISTANCE * Math.cos(Math.toRadians(thetaY/2));
    double y = LIGHT_SENSOR_DISTANCE * Math.cos(Math.toRadians(thetaX)/2);
    
    odometer.setXyt(x, y, odometer.getTheta());
    //Move to the crossing of lines and rotate back to 0deg
    Movement.travelTo(0.0, 0.0);
    Movement.turnTo(fixAngle(avgAngles[0], avgAngles[2]) +180.0);
  }
  
  /**
   * Helper method used by localize() to make any angle positive to facilitate calculations
   * 
   * @param angle to make positive in degrees
   * @return the angle by making it positive in degrees
   */
  private static double makePositive(double angle) {
    return (angle < 0) ? angle + 360 : angle;
  }
  
  /**
   * Helper method used by localize() to make an angle smaller than 180
   * 
   * @param angle in degrees that needs to be checked
   * @return the angle unmodified if it is already under 180, otherwise substract this angle from 360
   */
  private static double makeObtuse(double angle) {
    return (angle > 180) ? 360-angle : angle;
  }
  
  /**
   * Helper method used by localize() to calculate the exact angle needed to return the error
   * 
   * @param a the angle that is supposed to be at 0deg
   * @param b the angle that is supposed to be at 180deg
   * @return the error of the real reference frame and the wanted reference frame
   */
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
   * Search the area for the cart
   * <p>
   * Search the area corresponding to the right team to find the cart by going through the middle of the tiles and zigzagging
   * by reaching the end of each row. i.e. Robot goes to the end of every row then moves up by one TILE_SIZE then moves to the end of the second row and so on.
   * <p>
   * Reorients the robot so it is perpendicular to the cart (more details at the method 
   * orientPerpendicular())
   */
  public static void search() {
    
    double xLength = SZ_UR_x - SZ_LL_x;
    double yLength = SZ_UR_y - SZ_LL_y - 1;
    
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
   * Goes back home by traveling to the bridge, through the bridge, back to the initial corner, then back to the initial position.
   * Decides on how to go back towards the bridge according to where the initial corner is.
   * Localizes with the light sensors after coming out of the bridge
   * <p>
   * Depending if the bridge is vertical or horizontal AND the HOME corner, determine from which side to approach the bridge.
   * The chosen algorithm is moving the robot to the left corner so,  lower left or upper right.
   */
  public static void toHome() {
    
    rightLightPoller.start();
    leftLightPoller.start();
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
    updateState(State.IDLE);
  }
  
 /**
  * Makes the robot beep a certain number of times
  * 
  * @param times number of beeps
  */
  public static void beeps(int times) {
    for (int i = 0; i<times; i++) {
      Sound.beep();
    }
  }
  
  /**
   * Calculates the slope of the cart according to the robot's current orientation and moves the robot accordingly.
   * <p>
   * Determines current distance to the cart, move perpendicularly up from the current orientation for 8cm, and measure the distance to the cart again.
   * Using arctan(diff/8.0) determine the angle at which the robot should turn to
   * <p>
   * If the distance is infinity, then goes 16.0cm in the opposite direction from the current position and measure the distance there.
   * Similarly, using arctan, determine the angle
   * <p>
   * After finding the angle, the robot chooses from which height to backup. This is to avoid bumping into the cart while backing up, so the height will be where there is the biggest distance from the two measurements
   */
  private static void orientPerpendicular() {
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
  
  /**
   * Setting the beginning x coordinate
   * 
   * @param x in cm the x distance from the wall
   */
  public static void setXBeginCoord(double x) {
    xBeginCoord = x;
  }
  
  /**
   * Getting the beginning x coordinate
   * 
   * @return the beginning x distance from the wall
   */
  public static double getXBeginCoord() {
    return xBeginCoord;
  }
  
  /**
   * Setting the beginning y coordinate
   * 
   * @param y in cm the x distance from the wall
   */
  public static void setYBeginCoord(double y) {
    yBeginCoord = y;
  }
  
  /**
   * Getting the beginning y coordinate
   * 
   * @return the beginning y distance from the wall
   */
  public static double getYBeginCoord() {
    return yBeginCoord;
  }
}
