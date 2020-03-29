package ca.mcgill.ecse211.project;

public class LightPoller extends Thread {
  
  public LightSensor sensor;
  

  private double value;

  public LightPoller(LightSensor sensor) {
    this.sensor = sensor;
  }
  
  public void run() {
    if (this.sensor == Resources.rightLightSensor || this.sensor == Resources.leftLightSensor) {

      while (Robot.isNavigating) {

        this.sensor.readColorSampleRed();
      }
    } else {
      System.out.println("No thread is associated with this sensor");
    }
    
  }
  
  public double getValue() {
    return value;
  }
  public boolean detected() {
    return value >= 0.35;
  }

}
