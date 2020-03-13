package ca.mcgill.ecse211.project;

//import static ca.mcgill.ecse211.project.Resources.*;

/**
 * Main class where the main thread will be started. All actions will be done via the Main class,
 * with the robot object as intermediate
 * 
 * @author Xinyue Chen
 * @author Zheng Yu Cui
 * @author Alixe Delabrousse
 * @author David Kronish
 * @author Ariane Leroux
 *
 */
public class Main {
  
  /**
   * Main thread will start here and sequentially start and stop new threads as needed.
   * This method calls the robot object to perform specific tasks
   * 
   */
  public static void main(String[] args) {
    
  }
  
  /**
   * Method to initially start the robot with input checking to make sure the right button is pressed.
   * Different methods useful for debugging: testing specific parts of the code
   * 
   * @return Button_ID according to what was clicked
   */
  private static int waitForInitialInput() {
    return 1;
  }
  
  /**
   * Make the current thread sleep for an amount of time
   *
   * @param time in ms for how long the Thread should sleep for
   */
  public static void SleepFor(double time) {
    
  }
}
