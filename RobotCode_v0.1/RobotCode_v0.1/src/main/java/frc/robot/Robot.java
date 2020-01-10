/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2020 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.vision.VisionThread;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PWMVictorSPX;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final int IMG_WIDTH = 320;
	private static final int IMG_HEIGHT = 240;
	
  private VisionThread visionThread;
  private VisionThread visionThread_Green;
  

  private double centerX = 0.0;
  private final Object imgLock = new Object();
  
  private final CANSparkMax m_leftMotor = new CANSparkMax(10, MotorType.kBrushless);
  private final CANSparkMax m_rightMotor = new CANSparkMax(11, MotorType.kBrushless);
  private final DifferentialDrive m_myRobot = new DifferentialDrive(m_leftMotor, m_rightMotor);
  private final Joystick m_leftStick = new Joystick(0);
  private final Joystick m_rightStick = new Joystick(1);

  private final Timer m_timer = new Timer();

  private boolean foundBlobs = false;
  private boolean foundGreen = false;


  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    
    
  }

  /**
   * This function is called periodically during autonomous.
   */
  
  @Override
  public void autonomousPeriodic() {
    //System.out.println("Starting Autonomous");

    //The robot should not move if it finds blue
    if (foundGreen) {
      m_myRobot.tankDrive(0, 0);
      return;
    }



    double centerX;
    synchronized (imgLock) {
        centerX = this.centerX;
    }
    double distanceFromCenter = centerX - (IMG_WIDTH / 2);
    if (distanceFromCenter < 0)
      distanceFromCenter *= -1;

    // distance from center as a percentage, it will be used for determining if it is centered enough
    double percentDistance = distanceFromCenter / IMG_WIDTH;
    //the weighted distance is a multiplier to the turning speed, it should be turning slower as it gets closer to the center
    //so that it doesn't overshoot, but still can turn fast initially
    double weightedDistance = distanceFromCenter / IMG_WIDTH;
    // anything over 50% is full turning speed (will be doubled from .5 to 1.0 soon)
    if (weightedDistance > .5)
      weightedDistance = .5;
    //scale the weight to have a more gradual drop 
    weightedDistance *= 2;
    //slowest speed it can go is 25% of the turning speeds
    if (weightedDistance < .25)
      weightedDistance = .25;
    System.out.println(distanceFromCenter);
    System.out.println(weightedDistance);
    //m_myRobot.arcadeDrive(.1, turn * 0.002);
    if (foundBlobs){
      //if it is within a 30% distance from the center it will count as forward (15% both sides)
      if (percentDistance < .15){
        System.out.println("Moving Forwards");
        m_myRobot.tankDrive(.3, .3);
      } else if (IMG_WIDTH / 2 < centerX){
        System.out.println("Turning Right");
        m_myRobot.tankDrive(.4 * weightedDistance, .1 * weightedDistance);
      } else{
        System.out.println("Turning Left");
        m_myRobot.tankDrive(.1 * weightedDistance, .4 * weightedDistance);
      }
    } else {
      System.out.println("Not Moving");
      m_myRobot.tankDrive(.0, .0);
    }
    
    
    /*Drive for 2 seconds
    if (m_timer.get() < 2.0){
      m_myRobot.arcadeDrive(0.5, 0.0); //drive forwards half speed
    } else {
      m_myRobot.stopMotor(); //then stop
    }
    */
  }

  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and uncomment the
   * getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to
   * the switch structure below with additional strings. If using the
   * SendableChooser make sure to add them to the chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    System.out.println("autonomousInit has started");
    m_timer.reset();
    m_timer.start();

    UsbCamera countourVideo;
    UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
    camera.setResolution(IMG_WIDTH, IMG_HEIGHT);
    System.out.println("Camera set, Robot Starting...");

    visionThread = new VisionThread(camera, new BlobPipeline(), pipeline -> {     
      //System.out.println("Inside Thread");
      if (!pipeline.findContoursOutput().isEmpty()) {
        System.out.println("Contours found");
        Rect r = Imgproc.boundingRect(pipeline.findContoursOutput().get(0));
        System.out.println(r.x);
        foundBlobs = true;
        synchronized (imgLock) {
            centerX = r.x + (r.width / 2);
            System.out.println("detected "+ pipeline.findBlobsOutput().get(0,0));
        }
      } else{
        //System.out.println("Blobs not found");
        foundBlobs = false;
      }
    });

    visionThread_Green = new VisionThread(camera, new GreenPipeline(), pipeline -> {     
      //System.out.println("Inside Thread");
      if (!pipeline.findContoursOutput().isEmpty()) {
        System.out.println("Freen found");
        Rect r = Imgproc.boundingRect(pipeline.findContoursOutput().get(0));
        System.out.println(r.x);
        foundGreen = true;
        synchronized (imgLock) {
            centerX = r.x + (r.width / 2);
            System.out.println("detected "+ pipeline.findBlobsOutput().get(0,0));
        }
      } else{
        //System.out.println("Blobs not found");
        foundGreen = false;
      }
    });

    System.out.println("Starting Threads");
    visionThread.start();
    visionThread_Green.start();
    System.out.println("Thread Starteds");
    System.out.println("autonomousInit has ended");
  }

  /*
  @Override
  public void disabledInit() {
    try {
    if (visionThread.isAlive())
      visionThread.stop();
    } catch (Exception e) {

    }
  }
  */

  @Override
  public void teleopInit() {
    try {
      if (visionThread.isAlive())
        visionThread.stop();
        visionThread_Green.stop();
      } catch (Exception e) {
      }
  } 

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    //m_myRobot.arcadeDrive(m_leftStick.getY(), m_rightStick.getY());

    /*This code has the robot move with a separate left and right stick
    ** The line directly below these comments is the actual movement of the motors
    * the rest is about the buttons
    *
    ** For the buttons, it first loops through all the possible buttons that could be clicked 
    * (assuming more buttons on the right joystick: those won't work, change as necessary) 
    ** After it detects that a button has been pressed, it checks which one was specifically pressed
    * based on a switch-case system. Decide which buttons you want to program something for, and 
    * make something for them. There is no break statement due to the fact that multiple buttons could be pressed at once
    ** NOTE: If it is a button that has to be pressed continuously, have it do the continous thing in the case
    * If it only has to be a click you have to implement a stick.getRawButtonPressed and/or stick.getRawButtonReleased 
    * system, though this could be a bit annoying. Such as a general stick.getRawButtonReleased(i) 
    * at the end to have it stop all the possible combinations of clicks
    */
    m_myRobot.tankDrive(- m_leftStick.getY() / 2, - m_rightStick.getY() /2);

    //this is to avoid index error on two joysticks with more or less buttons than another
    int maxButtons;
    if (m_leftStick.getButtonCount() > m_rightStick.getButtonCount() ){
      maxButtons = m_rightStick.getButtonCount();
    } else {
      maxButtons = m_leftStick.getButtonCount();
    }
    //indexes start at 1 for joysticks
    for (int i = 1; i < maxButtons; i++) {
      if (m_leftStick.getRawButton(i)){
        System.out.println("Button" + i + "L pressed");
        switch (i) {
          case (1):
            System.out.println("Example of a hold");
          case (2):
            if (m_leftStick.getRawButtonPressed(2)) {
              System.out.println("Example of a click");
            }
        }
            
      }
      if (m_rightStick.getRawButton(i)){
        System.out.println("Button" + i + "R pressed");
        switch (i) {
          case (1):
            System.out.println("Example of a Hold");
          case (2):
          if (m_rightStick.getRawButtonPressed(2)) {
            System.out.println("Example of a click");
          }
        }
      }
    }
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
