package ca.mcgill.ecse211.project;

import static ca.mcgill.ecse211.project.Resources.*;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

/**
 * The LightPoller class has access to a lightsensor sample provider and is able to poll the current colour value.
 * Each LightPoller object is initialized with a sample provider and a sample mode and outputs the values accordingly.
 * 
 * @author Xinyue Chen
 * @author Zheng Yu Cui
 * @author Alixe Delabrousse
 * @author David Kronish
 * @author Ariane Leroux
 *
 */
public class LightPoller extends Thread {
  
  /**
   * Sample provider of the sensor referenced by the LightPoller object
   */
  public SampleProvider sampleProvider;
  /**
   * Colour sensor linked with the LightPoller object
   */
  public EV3ColorSensor sensor;
  /**
   * Sensor linked witht he LightPoller object
   */
  public SensorModes sensorMode;
  /**
   * Array where values are stored for rgb values
   */
  private float[] colourSensorValues;
  /**
   * Value of the colour stored when the mode is simply "RED"
   */
  private float color;
  
  /**
   * LightPoller constructor takes the correct sensor and its sensor more to initialize all values to be able to call 
   * the right polling method
   * 
   * @param sensor reference to the physical sensor on the brick
   * @param mode how the sensor is used. e.g. "Red", "RGB"
   */
  public LightPoller(EV3ColorSensor sensor, String mode) {
    this.sensor = sensor;
    this.sensorMode = this.sensor;
    this.sampleProvider = this.sensorMode.getMode(mode);
    this.colourSensorValues = new float[this.sensorMode.sampleSize()];
  }
  
  /**
   * Runs the polling thread continuously as long as the robot is navigating for side light pollers and as long as the 
   * robot is searching for colour poller. The side pollers are for line detections while the colour poller is for
   * colour detection.
   */
  public void run() {
    if (this == rightLightPoller || this == leftLightPoller) {
      while (true) {
        while(Robot.isNavigating) {
          this.readColorSampleRed();
        }
        this.sleepFor(200);
      }
      
    } else if (this == colorLightPoller) {
      while (true) {
        while(Robot.isSearching) {
          this.readColorSample();
        }
      }
    } else {
      System.out.println("No thread is associated with this sensor");
    }
    
  }
  
  /**
   * Reads the color from the side sensors so this is for line detection and stores it in color
   */
  private void readColorSampleRed() {
    this.sampleProvider.fetchSample(this.colourSensorValues, 0);
    this.color = this.colourSensorValues[0] * 100;
   }
  
  /**
   * Reads the RGB values from the colour sensor and stores it in colourSensorValues
   */
  private void readColorSample() {
    this.sampleProvider.fetchSample(this.colourSensorValues, 0);
  }
  
  
  /**
   * Returns the latest value of the colour polled
   * 
   * @return a value between 0 and 1, 0 being white and 1 being black
   */
  public float getColor() {
    return this.color;
  }
  
  /**
   * Returns the latest array of the colours being polled if the mode is RGB
   * 
   * @return a value between 0 and 1, 0 being while adn 1 being saturated colour
   */
  public float[] getColours() {
    return this.colourSensorValues;
  }
  
  /**
   * Check whether a line is detected or not given a float sensor value
   * 
   * @return true if the value is larger than 0.35
   */
  public boolean lineDetected() {
    return this.color > 0.35;
  }
  
  /**
   * Sleeps current thread for the specified duration.
   * 
   * @param duration sleep duration in milliseconds
   */
  public void sleepFor(long duration) {
    try {
      this.sleep(duration);
    } catch (InterruptedException e) {
      // There is nothing to be done here
    }
  }

  

}
