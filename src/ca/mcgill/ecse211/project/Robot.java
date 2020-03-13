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
   * Enumerations to define the states from the state machine
   */
  enum RobotStatus { localizing, travelling, searching, returning }
  private static RobotStatus robotStatus;
  
  /**
   * Initializing the state machine and the static references to various detection and movement objects
   */
  public Robot() {
    
  }
  
  /**
   * Initializes the robot's position and orienting it so it faces 0deg
   */
  public static void Initialize() {
    
  }
  
  /**
   * Movement to search using the data from the Wifi class
   */
  public static void ToSearch() {
    
  }
  
  /**
   * Only localize according to the lightsensor
   */
  public static void Localize() {
    
  }
  
  /**
   * Search the area with the data from wifi class
   */
  public static void Search() {
    
  }
  
  /**
   * Movement back to home area
   */
  public static void ToHome() {
    
  }
  
  /**
   * Beeps 3 times
   */
  public static void ThreeBeeps() {
    
  }
  
  /**
   * Beeps 5 times
   */
  public static void FiveBeeps() {
    
  }
}
