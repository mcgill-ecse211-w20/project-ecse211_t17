package ca.mcgill.ecse211.project;

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
    SEARCHING,
    IDENTIFICATION
  }
  
  // booleans to regulate the running of Threads
  /**
   * Set to true when the robot is in the TRAVELING and SEARCHING states
   */
  public boolean isNavigating;
  
  /**
   * Set to true when the robot is in the CROSSING state
   */
  public boolean crossBridge;
  
  /**
   * Set to true when the robot is in the SEARCHING and IDENTIFICATION states
   */
  public boolean isSearching;
  
  /**
   * Current state of the robot
   */
  public State robotStatus;
  
  /**
   * Initializing the state machine and the static references to various detection and movement objects
   */
  public Robot() {
    this.updateState(State.IDLE);
  }
  
  public static synchronized Robot getRobot() {
    return new Robot();
  }
  
  /**
   * Updates the current state of the robot object as well as the corresponding booleans
   * 
   * @param newState - the new state of the device
   */
  public void updateState(State newState){
    this.robotStatus = newState;
    switch (newState) {
      case IDLE:
        this.isNavigating = false;
        this.isSearching = false;
        this.crossBridge = false;
        break;
      case TRAVELING:
        this.isNavigating = true;
        this.isSearching = false;
        this.crossBridge = false;
        break;
      case CROSSING:
        this.isNavigating = false;
        this.isSearching = false;
        this.crossBridge = true;
      case SEARCHING:
        this.isNavigating = true;
        this.isSearching = true;
        this.crossBridge = false;
      case IDENTIFICATION:
        this.isNavigating = false;
        this.isSearching = true;
        this.crossBridge = false;
    }   
  }
  
  
  /**
   * Initializes the robot's position and orienting it so it faces 0deg
   */
  public static void initialize() {
    
  }
  
  /**
   * Movement to search using the data from the Wifi class
   */
  public static void toSearch() {
    
  }
  
  /**
   * Only localize according to the lightsensor
   */
  public static void localize() {
    
  }
  
  /**
   * Search the area with the data from wifi class
   */
  public static void search() {
    
  }
  
  /**
   * Movement back to home area
   */
  public static void toHome() {
    
  }
  
  /**
   * Beeps 3 times
   */
  public static void threeBeeps() {
    
  }
  
  /**
   * Beeps 5 times
   */
  public static void fiveBeeps() {
    
  }
}
