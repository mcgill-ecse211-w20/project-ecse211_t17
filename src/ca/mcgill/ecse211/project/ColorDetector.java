package ca.mcgill.ecse211.project;

import ca.mcgill.ecse211.project.Resources.Team;

/**
 * Class that has a reference to the colour light sensor to detect colour and return the normalized value
 * 
 * @author Xinyue Chen
 * @author Zheng Yu Cui
 * @author Alixe Delabrousse
 * @author David Kronish
 * @author Ariane Leroux
 *
 */
public class ColorDetector {
  //RGB indexes
  private static final int RED_INDEX = 0;
  private static final int GREEN_INDEX = 1;
  private static final int BLUE_INDEX = 2;

  // rgb color data in the order of {R,G,B}

  public static double[] RED_COLOR = {0.959771, 0.26671, 0.08776}; // value of red color
  public static double[] GREEN_COLOR = {0.398574, 0.904758, 0.15017}; // value of green color
  public static double[] BLUE_COLOR = {0.149069, 0.780432, 0.60721}; // value of blue color
  public static double[] YELLOW_COLOR = {0.82158, 0.55636, 0.1244}; // value of yellow color


  public static final float[] sampleColor = new float[100]; // create an array for the sensor


  private static double smallest = 1; // the max possible value for a normalized reading is 1    
  private static double colorThreshold = 0.2; // all correct readings are smaller than thus thresold, obtained in the
                                             // color sampling process
  
  /**
   * Calculates the mean and standard deviation for the initial readings and call color() method for
   * telling what the color is where 0=red, 1=green, 2=blue, 3=yellow, and -1 for everything else
   * 
   * @return int color
   */
  public static Team DetectColor() {
    
    int colour;
    Team colorT;
    
    colour = 0;
    
    if (colour == 0) {
      colorT = Team.RED;
    } else {
      colorT = Team.GREEN;
    }
    
    return colorT;
  }
  
  /**
   * calculates the euclidean distance in RGB space
   * 
   * @param rN normalized red reading
   * @param gN normalized green reading
   * @param bN normalized blue reading
   * @param rM mean red value for a color
   * @param gM mean green value for a color
   * @param bM mean blue value for a color
   * @return the euclidean distance in double
   */
  public static double euclidean(double rN, double gN, double bN, double rM, double gM, double bM) {
    return Math.sqrt(Math.pow((rN - rM), 2) + Math.pow((gN - gM), 2) + Math.pow((bN - bM), 2));
  }
  
  /**
   * this is used for normalizing the r value of a new reading
   * 
   * @param r red reading
   * @param g green reading
   * @param b blue reading
   * @return normalized reading
   */
  public static double rNormalize(float r, float g, float b) {
    return r / (Math.sqrt(Math.pow(r, 2) + Math.pow(g, 2) + Math.pow(b, 2)));
  }
  
  /**
   * this is used for normalizing the g value of a new reading
   * 
   * @param r red reading
   * @param g green reading
   * @param b blue reading
   * @return normalized reading
   */
  public static double gNormalize(float r, float g, float b) {
    return g / (Math.sqrt(Math.pow(r, 2) + Math.pow(g, 2) + Math.pow(b, 2)));
  }

  /**
   * this is used for normalizing the b value of a new reading
   * 
   * @param r red reading
   * @param g green reading
   * @param b blue reading
   * @return normalized reading
   */
  public static double bNormalize(float r, float g, float b) {
    return b / (Math.sqrt(Math.pow(r, 2) + Math.pow(g, 2) + Math.pow(b, 2)));
  }
  
  /**
   * Calibrate the values for the colour red
   */
  public static void rCalibrate() {
    
  }
  
  /**
   * Calibrate the values for the colour green
   */
  public static void gCalibrate() {
    
  }
  
  /**
   * Calibrate the values for the colour blue
   */
  public static void bCalibrate() {
    
  }
}
