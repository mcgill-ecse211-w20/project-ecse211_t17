package ca.mcgill.ecse211.project;

public class LightPoller extends Thread {
  
  public LightSensor sensor;
  

  private float color;

  public LightPoller(LightSensor sensor) {
    this.sensor = sensor;
  }
  
  public void run() {
    if (this.sensor == Resources.rightLightSensor || this.sensor == Resources.leftLightSensor) {
      while (true) {
        while(Robot.isNavigating) {
          this.readColorSampleRed();
        }
        this.sleepFor(200);
      }
      
    } else {
      System.out.println("No thread is associated with this sensor");
    }
    
  }
  
  /**
   * Reads the color from the Color Sensor
   */
  public void readColorSampleRed() {
    this.sensor.sampleProvider.fetchSample(this.sensor.colourSensorValues, 0);
    this.color = this.sensor.colourSensorValues[0] * 100;
   }
  
  /**
   * Check whether a line is detected or not given a float sensor value
   * 
   * @param colorSensorValue from a EV3ColorSensor
   */
  public boolean lineDetected() {
    return this.color < 0.35;
  }
  
  public double getColor() {
    return this.color;
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
