package ca.mcgill.ecse211.project;

import static ca.mcgill.ecse211.project.Resources.*;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

public class LightPoller extends Thread {
  
  public SampleProvider sampleProvider;
  public EV3ColorSensor sensor;
  public SensorModes sensorMode;
  public float[] colourSensorValues;
  

  private float color;

  public LightPoller(EV3ColorSensor sensor, String mode) {
    this.sensor = sensor;
    this.sensorMode = this.sensor;
    this.sampleProvider = this.sensorMode.getMode(mode);
    this.colourSensorValues = new float[this.sensorMode.sampleSize()];
  }
  
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
   * Reads the color from the Color Sensor
   */
  private void readColorSampleRed() {
    this.sampleProvider.fetchSample(this.colourSensorValues, 0);
    this.color = this.colourSensorValues[0] * 100;
   }
  
  private void readColorSample() {
    this.sampleProvider.fetchSample(this.colourSensorValues, 0);
  }
  
  
  public float getColor() {
    return this.color;
  }
  
  public float[] getColours() {
    return this.colourSensorValues;
  }
  
  /**
   * Check whether a line is detected or not given a float sensor value
   * 
   * @param colorSensorValue from a EV3ColorSensor
   */
  public boolean lineDetected() {
    return this.color < 0.35;
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
