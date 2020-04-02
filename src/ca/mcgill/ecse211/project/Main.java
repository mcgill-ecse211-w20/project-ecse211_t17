package ca.mcgill.ecse211.project;

import lejos.hardware.Button;

import static ca.mcgill.ecse211.project.Resources.*;

/**
 * Main class is the main entry point of the robot. All it does is calling the Robot static class to perform all actions
 * like moving around or beeping.
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
   * This method calls robot to perform specific tasks
   * 
   */
  public static void main(String[] args) {
        
    //start the odometer thread
    odometer.start();
    
    //storing the of the button
    int click = Button.waitForAnyPress();
    
    //Set for debugging purposes
    while (click != Button.ID_ESCAPE) {
      if (click == Button.ID_ENTER) {
        Robot.initialize();
        Robot.beeps(3);
        Robot.toSearch();
        Robot.beeps(3);
        Robot.search();
        Robot.toHome();
        Robot.beeps(5);
      }
    }
    System.exit(0);
  }
  
  
  /**
   * Make the current thread sleep for an amount of time
   *
   * @param time in ms for how long the Thread should sleep for
   */
  public static void sleepFor(long time) {
   try {
     Thread.sleep(time);
   } catch (InterruptedException e) {
     
   }
  }
}
