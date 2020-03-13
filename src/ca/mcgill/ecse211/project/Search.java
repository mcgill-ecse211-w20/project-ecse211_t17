package ca.mcgill.ecse211.project;

/**
 * Intermediate between the WIFI class and the Movement class
 * Performs all actions when in the search area
 * 
 * @author Xinyue Chen
 * @author Zheng Yu Cui
 * @author Alixe Delabrousse
 * @author David Kronish
 * @author Ariane Leroux
 *
 */
public class Search {
  
  //References to odometer and movement
  private static Odometer odometer;
  private static ColorDetector colorDetector;
  private static UltrasonicSensor usSensor_Donut;
  
  /**
   * Thread Manipulation
   */
  private static boolean collision;
  
  /**
   * Initializes the static references
   * @param cd ColorDetector object which has a reference to the right sensor
   * @param uss UltrasonicSensor object that sees where the objects are
   * 
   */
  public Search(ColorDetector cd, UltrasonicSensor uss) {
    
  }
  
  /**
   * Search around the area while iteratively looking for the cart and avoiding collision
   */
  public static void Search() {
    //while loop as long as collision == false
    
    //Search for color sticker
    //After some time if no specific sticker then move on 
  }
  
  /**
   * Using the ultrasonic sensor to detect objects in front of the robot
   */
  public static void DetectCollision() {
    //thread.start
    collision = false;
    //while loop to check the distances
    collision = true;
  }
  
  /**
   * Moves around according to the input map which represents the search area
   * @param map 2D array that represents the corners of the search area
   */
  public static void SearchInArea(int[][] map) {
    
  }
  
  

}
