package ca.mcgill.ecse211.project;

public class LightPoller extends Thread {
  
  public LightSensor sensor;
  
  
  
  public LightPoller(LightSensor sensor) {
    this.sensor = sensor;
  }
  
  public void run() {
    if (this.sensor == Resources.rightLightSensor || this.sensor == Resources.leftLightSensor) {
      while (Resources.robot.isNavigating) {
        this.sensor.readColorSampleRed();
      }
    } else {
      System.out.println("No thread is associated with this sensor");
    }
    
  }

}
