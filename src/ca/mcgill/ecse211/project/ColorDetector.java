package ca.mcgill.ecse211.project;

import static ca.mcgill.ecse211.project.Resources.*;

/**
 * Class that has a reference to the colour light sensor to calibrate the initial colours and then detect which colours
 * it is using the mean and standard deviation for initial colours
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
public class ColorDetector {
  /**
   * Red index in the values array
   */
  private static final int RED_INDEX = 0;
  /**
   * Green index in the values array
   */
  private static final int GREEN_INDEX = 1;
  /**
   * Blue index in the values array
   */
  private static final int BLUE_INDEX = 2;

  /**
   * RGB values for the colour red
   */
  public static float[] RED_COLOR = {(float) 0.959771, (float) 0.26671, (float) 0.08776};
  /**
   * RGB values for the colour green
   */
  public static float[] GREEN_COLOR = {(float) 0.398574, (float) 0.904758, (float) 0.15017}; 
  /**
   * RGB values for the colour blue
   */
  public static float[] BLUE_COLOR = {(float) 0.149069, (float) 0.780432, (float) 0.60721};
  /**
   * RGB values for the colour yellow
   */
  public static float[] YELLOW_COLOR = {(float) 0.82158, (float)0.55636, (float)0.1244};
  
  /**
   * Storage for the current detected colour
   */
  public static float[] sampleColor = new float[100];

  /**
   * the max possible value for a normalized reading is 1  
   */
  private static double smallest = 1;
  /**
   * all correct readings are smaller than thus thresold, obtained in the
   * color sampling process
   */
  private static double colorThreshold = 0.2;
  
  /**
   * Calculates the mean and standard deviation for the initial readings returns what the color is detected
   * 
   * @return int where 0=red, 1=green, 2=blue, 3=yellow, and -1 for everything else
   */
  public static int DetectColor() {
    
 // obtain reading from sensor and normalize them
    sampleColor = colorLightPoller.getColours();
    double r = rNormalize(sampleColor[0] * 1000, sampleColor[1] * 1000, sampleColor[2] * 1000);
    double g = gNormalize(sampleColor[0] * 1000, sampleColor[1] * 1000, sampleColor[2] * 1000);
    double b = bNormalize(sampleColor[0] * 1000, sampleColor[1] * 1000, sampleColor[2] * 1000);

    // get euclidean distance for each color
    double dBlue = euclidean(r, g, b, BLUE_COLOR[RED_INDEX], BLUE_COLOR[GREEN_INDEX], BLUE_COLOR[BLUE_INDEX]);
    double dGreen = euclidean(r, g, b, GREEN_COLOR[RED_INDEX], GREEN_COLOR[GREEN_INDEX], GREEN_COLOR[BLUE_INDEX]);
    double dYellow = euclidean(r, g, b, YELLOW_COLOR[RED_INDEX], YELLOW_COLOR[GREEN_INDEX], YELLOW_COLOR[BLUE_INDEX]);
    double dRed = euclidean(r, g, b, RED_COLOR[RED_INDEX], RED_COLOR[GREEN_INDEX], RED_COLOR[BLUE_INDEX]);

    // rank the distances and choose the color --smallest euclidean
    double[] d = {dBlue, dGreen, dYellow, dRed};
    for (int i = 0; i < 4; i++) {
      if (i == 0)
        smallest = d[i];
      else if (d[i] < smallest)
        smallest = d[i];
    }

    /// return the TR value
    if (smallest <= colorThreshold) {
      if (smallest == dRed) {
        return 0; // return red
      }
      if (smallest == dGreen) {
        return 1; // return green
      }
      if (smallest == dBlue) {
        return 2; // return blue
      }
      if (smallest == dYellow) {
        return 3; // return yellow
      }
    }
    return -1;
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
   * Calibrate the RGB values for the colour red. This is done just before starting the competition
   */
  public static void rCalibrate() {
    RED_COLOR = colorLightPoller.getColours();
  }
  
  /**
   * Calibrate the RGB values for the colour green. This is done just before starting the competition
   */
  public static void gCalibrate() {
    GREEN_COLOR = colorLightPoller.getColours();
  }
  
  /**
   * Calibrate the RGB values for the colour yellow. This is done just before starting the competition
   */
  public static void yCalibrate() {
    YELLOW_COLOR = colorLightPoller.getColours();
  }
  
  /**
   * Calibrate the RGB values for the colour blue. This is done just before starting the competition
   */
  public static void bCalibrate() {
    BLUE_COLOR = colorLightPoller.getColours();
  }
}
