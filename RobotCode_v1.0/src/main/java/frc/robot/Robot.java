/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

//default imports
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

//Driving Imports 
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;

//Motor Imports
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.VictorSP;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

//Camera Imports
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.vision.VisionThread;
import edu.wpi.first.wpilibj.SerialPort;

//The Gyroscope Imports
import com.analog.adis16448.frc.ADIS16448_IMU;

//Other
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;
import java.util.Map;
//import java.util.Timer;
import edu.wpi.first.wpilibj.Timer;
import java.util.ArrayList;
import java.util.HashMap;
import java.lang.Math;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  // Camera Resolution
  // Note that pixel 0,0 is top left, and 315,207 is bottom right
  private static final int IMG_WIDTH = 315;
  private static final int IMG_HEIGHT = 207;

  private SerialPort arduino;
  public String[] ballLocations = { "Left", "Right", "Center" };
  public String ballLocation;

  // The Base of the Vision Threads which detect Color
  /*
   * //This would repeat for all colors
   * 
   * private VisionThread visionThread_COLOR; private double centerX_COLOR = 0.0;
   * private double centerY_COLOR = 0.0; privatefinal Object imgLock_COLOR = new
   * Object();
   * 
   * visionThread_COLOR = new VisionThread(camera, new COLORPipeline(), pipeline
   * -> { if (!pipeline.findContoursOutput().isEmpty()) { Rect r_COLOR =
   * Imgproc.boundingRect(pipeline.findContoursOutput().get(0)); found_COLOR =
   * true; synchronized (imgLock_COLOR) { centerX_COLOR = r_COLOR.x +
   * (r_COLOR.width / 2); } } else{ found_COLOR = false; } });
   * 
   * ############ WIP, might not use //HELP:
   * https://stackoverflow.com/questions/19602601/create-an-arraylist-with-
   * multiple-object-types //Array holding all the vision threads //NO USE OF IT
   * YET, BUT A SIMILAR THING COULD BE APPLIED FOR OTHER INFO
   * 
   * Map<String, VisionThread> visionThreads = new HashMap<String, VisionThread>()
   * {{ put("COLOR", visionThread_COLOR); }};
   * 
   * //OR
   * 
   * public ObservableList[] colors = { {"COLOR", visionThread_COLOR, }, {"COLOR",
   * visionThread_COLOR, }, {"COLOR", visionThread_COLOR, }; }; ###########
   */

  /*
   * FRONT (OMNIS) _____5_____ | 9 | 1 2 | 8 | | | 3 4 |___ 7 ____|
   */
  // change number depending on what they are entered to
  // dont use front back, use left right
  private final VictorSPX frontLeftMotor = new VictorSPX(4);
  private final VictorSPX backLeftMotor = new VictorSPX(2);
  // private SpeedControllerGroup leftMotors = new SpeedControllerGroup(frontLeftMotor, backLeftMotor);
  private final VictorSPX frontRightMotor = new VictorSPX(1);
  private final VictorSPX backRightMotor = new VictorSPX(3);
  // private SpeedControllerGroup rightMotors = new SpeedControllerGroup(frontRightMotor, backRightMotor);
  private final VictorSP swerveMotor1 = new VictorSP(8);
  private final VictorSP swerveMotor2 = new VictorSP(9);
  // private SpeedControllerGroup frontMotors = new SpeedControllerGroup(frontMotor1, frontMotor2);

  private final SpeedController liftMotor1 = new CANSparkMax(6, MotorType.kBrushless);
  private final SpeedController liftMotor2 = new CANSparkMax(5, MotorType.kBrushless);
  private SpeedControllerGroup liftMotors = new SpeedControllerGroup(liftMotor1, liftMotor2);
  private final SpeedController shootMotor1 = new CANSparkMax(10, MotorType.kBrushless);
  private final SpeedController shootMotor2 = new CANSparkMax(11, MotorType.kBrushless);
  private SpeedControllerGroup shootMotors = new SpeedControllerGroup(shootMotor1, shootMotor2);

  private final SpeedController winch = new CANSparkMax(7, MotorType.kBrushless);
  private SpeedControllerGroup winchMotor = new SpeedControllerGroup(winch);
  //-private final SpeedController piston = new CANSparkMax(11, MotorType.kBrushless);


  // movement by .set(speed)
  // 2 neo shooter
  // redline intake

  // 2 Joysticks for TankDrive
  private final Joystick leftJoystick = new Joystick(0);
  private final Joystick rightJoystick = new Joystick(1);

  // Incase a timer is needed, from java.util (normal java one)
  private final Timer timer = new Timer();

  /*
   * getAngle() is for the angle ==== NOTE: Uses 360 degrees and overdoes it aka
   * 361 degrees getRate() method to obtain the current rotation rate being
   * measured getGyroInstantX is for the X getGyroInstantY is for the Y
   */
  public static final ADIS16448_IMU imu = new ADIS16448_IMU();

  // Bot Information. The X and Y are for the grid system. the botAngle is for
  // orientation. 0-360 where 90 is the right, and 270 is left
  public double botX = 0.0;
  public double botY = 0.0;
  public double botAngle = 0.0; // radians
  public int ballCount = 0;
  public int[] ballPos = { 1, 1, 1 };
  public Boolean lookingForBall = false;

  //misc
  public Boolean initial = true;

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {
    arduino = new SerialPort(9600, SerialPort.Port.kUSB1);
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for
   * items like diagnostics that you want ran during disabled, autonomous,
   * teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable chooser
   * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
   * remove all of the chooser code and uncomment the getString line to get the
   * auto name from the text box below the Gyro
   *
   * <p>
   * You can add additional auto modes by adding additional comparisons to the
   * switch structure below with additional strings. If using the SendableChooser
   * make sure to add them to the chooser code above as well.
   */
  @Override
  public void autonomousInit() {

  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    /*
     * Step1: side step to shoot
     */
    if (initial) {
      findShortcut(30, 0);
      findShortcut(30, 300);
      findShortcut(0, 300);
      try {
        shootAutonomous();
      } catch (InterruptedException e) {
        e.printStackTrace();
      }
      lookingForBall = true;
      initial = false;
    }
    
    if (lookingForBall) {
      try {
        ballLocation = ballLocations[Integer.parseInt(arduino.readString())];
      } catch (Exception e) {
        ballLocation = "none";
      }
      switch (ballLocation) {
        case "Forward":
          forwards(.2);
          break;
        case "Left":
          backLeftMotor.set(ControlMode.PercentOutput,-1*-0.3);
          frontLeftMotor.set(ControlMode.PercentOutput,-1*-0.3);
          backRightMotor.set(ControlMode.PercentOutput,0.3);
          frontRightMotor.set(ControlMode.PercentOutput,0.3);
        case "Right":
          backLeftMotor.set(ControlMode.PercentOutput,-1*0.3);
          frontLeftMotor.set(ControlMode.PercentOutput,-1*0.3);
          backRightMotor.set(ControlMode.PercentOutput,-0.3);
          frontRightMotor.set(ControlMode.PercentOutput,-0.3);
      }
    }
    
    
  }

  public void forwardsTime(double seconds, double speed){
    long endTime = System.currentTimeMillis() + ((int) seconds*1000);
    while(System.currentTimeMillis() < endTime) {
      backLeftMotor.set(ControlMode.PercentOutput,-1*speed);
      frontLeftMotor.set(ControlMode.PercentOutput,-1*speed);
      backRightMotor.set(ControlMode.PercentOutput,speed);
      frontRightMotor.set(ControlMode.PercentOutput,speed);
      // pause to avoid churning
      //Thread.sleep(1);
      //calculateGridSwerve(speed, 0);
    }
  }
  public void backwardsTime(double seconds, double speed){
    long endTime = System.currentTimeMillis() + ((int) seconds*1000);
    while(System.currentTimeMillis() < endTime) {
      backLeftMotor.set(ControlMode.PercentOutput,-1*-speed);
      frontLeftMotor.set(ControlMode.PercentOutput,-1*-speed);
      backRightMotor.set(ControlMode.PercentOutput,-speed);
      frontRightMotor.set(ControlMode.PercentOutput,-speed);
      //calculateGridSwerve(-speed, 0);
    }
  }
  public void leftTime(double seconds, double speed){
    long endTime = System.currentTimeMillis() + ((int) seconds*1000);
    while(System.currentTimeMillis() < endTime) {
      swerveMotor1.set(speed);
      swerveMotor2.set(speed);
      //calculateGridSwerve(0, -speed);
    }
  }
  public void rightTime(double seconds, double speed){
    long endTime = System.currentTimeMillis() + ((int) seconds*1000);
    while(System.currentTimeMillis() < endTime) {
      swerveMotor1.set(-speed);
      swerveMotor2.set(-speed);
      //calculateGridSwerve(0, speed);
    }
  }

  public void forwards(double speed){
    backLeftMotor.set(ControlMode.PercentOutput,-1*speed);
    frontLeftMotor.set(ControlMode.PercentOutput,-1*speed);
    backRightMotor.set(ControlMode.PercentOutput,speed);
    frontRightMotor.set(ControlMode.PercentOutput,speed);
    //calculateGridSwerve(speed, 0);
  }
  public void backwards(double speed){
    backLeftMotor.set(ControlMode.PercentOutput,-1*-speed);
    frontLeftMotor.set(ControlMode.PercentOutput,-1*-speed);
    backRightMotor.set(ControlMode.PercentOutput,-speed);
    frontRightMotor.set(ControlMode.PercentOutput,-speed);
    //calculateGridSwerve(-speed, 0);
  }
  public void left(double speed){
    swerveMotor1.set(speed);
    swerveMotor2.set(speed);
    //calculateGridSwerve(0, -speed);
  }
  public void right(double speed){
    swerveMotor1.set(-speed);
    swerveMotor2.set(-speed);
    //calculateGridSwerve(0, speed);
  }


  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    /*This code has the robot move with a separate left and right stick (tank drive)
    * alternatively, if the right button is held down, it enters a single joystick "glide motion"
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
    int rightButton = getRightJoystickButton();
    int leftButton = getLeftJoystickButton();
    boolean isSwerve = (rightButton == 2);

    movementOfWheels(isSwerve);

    if (leftButton == 1){liftMotors.set(0.3);}
    if (rightButton == 1){shootMotors.set(0.3);}
    if (rightButton == 3){winch.set(0.3);}
    if (rightButton == 2){winch.set(0.3);}
  }

  public int getRightJoystickButton() {
    //this is to avoid index error on two joysticks with more or less buttons than another
    int maxButtonsRight;
    maxButtonsRight = rightJoystick.getButtonCount();

    //indexes start at 1 for joysticks
    //this holds a case and switch depending on a button
    //if case (1), then do something when button 1 pressed. 
    //change the buttons as needed
    /* Hold runs every tick, click only runs once per press
      case (X):
         System.out.println("Example of a Hold");
         return X;
      case (Y):
        if (rightJoystick.getRawButtonPressed(Y)) {
          System.out.println("Example of a click");
          return Y;
        }
    */
    for (int i = 1; i < maxButtonsRight; i++) {
      if (rightJoystick.getRawButton(i)){
        System.out.println("Button" + i + "R pressed");
        switch (i) {
          case (1):
            System.out.println("Example of a Hold");
            return 1;
          case (2):
            System.out.println("Example of a Hold");
            return 2;
          case (3):
            System.out.println("Example of a Hold");
            return 3;
        }
      }
    }
    //if it reaches here, no button was pressed
    return 0;
  }

  //look at function for getRightJoystickButton() above this to understand,
  //it works the same but for the left joystick
  public int getLeftJoystickButton() {
    //this is to avoid index error on two joysticks with more or less buttons than another
    int maxButtonsLeft;
    maxButtonsLeft = leftJoystick.getButtonCount();
    for (int i = 1; i < maxButtonsLeft; i++) {
      if (leftJoystick.getRawButton(i)){
        System.out.println("Button" + i + "R pressed");
        switch (i) {
          case (1):
            System.out.println("Example of a Hold");
            return 1;
          case (2):
            if (leftJoystick.getRawButtonPressed(2)) {
              System.out.println("Example of a click");
              return 2;
            }
        }
      }
    }
    //if it reaches here, no button was pressed
    return 0;
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }

  public void Half_Turn(){
     //it will loop until it reached 4% from angle 0
     while (true){
      double rotationSpeed = .5; //idk about best speed, Trial an Error sort of thing?

      if (imu.getAngle() > 200 || imu.getAngle() < 160){ 
      //if (botAngle > 1.3*Math.PI){ 
        //the weighted angle makes it move slower as it gets closer
        //leftMotors.set(rotationSpeed);
        //rightMotors.set(-rotationSpeed);
        backLeftMotor.set(ControlMode.PercentOutput,rotationSpeed);
        frontLeftMotor.set(ControlMode.PercentOutput,rotationSpeed);
        backRightMotor.set(ControlMode.PercentOutput,-rotationSpeed);
        frontRightMotor.set(ControlMode.PercentOutput,-rotationSpeed);
      } 
      else if (imu.getAngle() > 185 || imu.getAngle() < 175){ 
      //else if (botAngle > Math.PI*1.1){ 
        //the weighted angle makes it move slower as it gets closer
        //leftMotors.set(rotationSpeed * .25);
        //rightMotors.set(-rotationSpeed * .25);
        backLeftMotor.set(ControlMode.PercentOutput,rotationSpeed* .25);
        frontLeftMotor.set(ControlMode.PercentOutput,rotationSpeed* .25);
        backRightMotor.set(ControlMode.PercentOutput,-rotationSpeed* .25);
        frontRightMotor.set(ControlMode.PercentOutput,-rotationSpeed* .25);
      } 
      else if (imu.getAngle() > 181.5 || imu.getAngle() < 178.5){
      //else if (botAngle > Math.PI*1.01){ 
        //the weighted angle makes it move slower as it gets closer
        //leftMotors.set(rotationSpeed * .1);
        //rightMotors.set(-rotationSpeed * .1);
        backLeftMotor.set(ControlMode.PercentOutput,rotationSpeed* .1);
        frontLeftMotor.set(ControlMode.PercentOutput,rotationSpeed* .1);
        backRightMotor.set(ControlMode.PercentOutput,-rotationSpeed* .1);
        frontRightMotor.set(ControlMode.PercentOutput,-rotationSpeed* .1);
      } //otherwise move counter clockwise
      else {
        backLeftMotor.set(ControlMode.PercentOutput,-rotationSpeed);
        frontLeftMotor.set(ControlMode.PercentOutput,-rotationSpeed);
        backRightMotor.set(ControlMode.PercentOutput,rotationSpeed);
        frontRightMotor.set(ControlMode.PercentOutput,rotationSpeed);
      }
    }
  }

  String hewwo = "👌😂😂😜"; //OwO wot's this

  public void movementOfWheels(boolean isSwerve){
    double speedLeft = leftJoystick.getY();
    double speedRight = rightJoystick.getY(); 
    double swerveSpeedX = rightJoystick.getX(); 
    double swerveSpeedY = rightJoystick.getY(); 
    if (!isSwerve){
      //DifferentialDrive m_myRobot = new DifferentialDrive(m_leftMotor, m_rightMotor);
      //leftMotors.set(speedLeft);
      //rightMotors.set(speedRight);
      backLeftMotor.set(ControlMode.PercentOutput,-1* speedLeft);
      frontLeftMotor.set(ControlMode.PercentOutput,-1* speedLeft);
      backRightMotor.set(ControlMode.PercentOutput,speedRight);
      frontRightMotor.set(ControlMode.PercentOutput,speedRight);
      //calculateGridNormal(speedLeft, speedRight);
      // assuming no friction for front Motors: (change as moving) frontMotors();
    } else {
      //the right joystick is the one for swerve
      //frontMotors.set(swerveSpeedX);
      swerveMotor1.set(swerveSpeedX);
      swerveMotor2.set(swerveSpeedX);

      //get rid of /2 if too slow
      //leftMotors.set(swerveSpeedY / 2);
      //rightMotors.set(swerveSpeedY / 2);
      backLeftMotor.set(ControlMode.PercentOutput,-1* swerveSpeedY / 2);
      frontLeftMotor.set(ControlMode.PercentOutput,-1* swerveSpeedY / 2);
      backRightMotor.set(ControlMode.PercentOutput,swerveSpeedY / 2);
      frontRightMotor.set(ControlMode.PercentOutput,swerveSpeedY / 2);

      //TODO: add a thing to calculate the movement for Autonomous
      //^ but would that be better found in the autonomous periodic Function?
      //TODO the back motors of Right and Left might need to have a slightly faster speed to compensate
      //for their Angle. this a T&E thing
      //calculateGridSwerve(swerveSpeedY, swerveSpeedX);
    }
  }

  //TODO change as speed needed
  public void shootContinous() {
    liftMotors.set(10);
    shootMotors.set(10);
  }

  //TODO set speed and time as necessary
  public void shootAutonomous() throws InterruptedException {
    while (ballPos[0] != 1) {
      moveBallsUp();
    }

    //shoots at time for 3 balls
    liftMotors.set(10);
    shootMotors.set(10);
    //shoot at 10 speed for 1 second (1000ms)
    try {
    Thread.sleep(1000);
    } catch (InterruptedException e) {}

    ballCount = 0;
    ballPos = new int[] {0,0,0};
    stopShoot();
    stopLift();
  }

  public void moveBallsUp(){
    liftMotors.set(0);
    try {
      Thread.sleep(200);
    } catch (InterruptedException e) {}
    liftMotors.set(0);
    //TODO when bot picks up ball have it move balls then set last one to 1 manually
    ballPos[0] = ballPos[1];
    ballPos[1] = ballPos[2];
    ballPos[2] = 0;
  }
  public void stopShoot() {
    shootMotors.set(0);
  }

  public void stopLift() {
    liftMotors.set(0);
  }

  //TODO change values as necessary
  public void doFirstShot() throws InterruptedException {
    findShortcut(20, 0);
    findShortcut(20, 200);
    findShortcut(0, 200);

    setOrigin();
    shootAutonomous();
    stopShoot();
  }

  public void goToShoot() throws InterruptedException {
    findShortcut(0, 0);
    resetOrientation();
    shootAutonomous();
  }

  public void MoveWinch() {
    winchMotor.set(50);
  }
  public void StopWinch() {
    winchMotor.set(0);
  }

  public void calculateGridNormal(double speedLeft, double speedRight) {

  /* There is a True X,Y which is the grid
        Y
        |
        |
    ----------- X
        |
        |
    
    and an internal X,Y in which the reference frame is the bot iself
    these will be called initial X and Y, they will then be translated into the true grid
    NOTE: the initial X and Y are speed vectors, what this does is change the speed
        into the translation of the bot in the true grid
  */

    double radius = 0.346; //the radius cause centripetal force, this was measured from Fer (in m. in cm is 34.6)

      
    //I sent a phys paper explaining my calculations for explanations: Vl = speedLeft Vr = speedRight
      /* Visualizing the Equation
          sqrt(Vl^2+Vr^2) 
      Vx = --------------- t
                  r

      Note: removing t for reasons explained in paper
      */
    double angleChange = (Math.sqrt(speedRight*speedRight + speedLeft*speedLeft)) / (radius);

    //the change will be positive if both rotate in same way, so if its not, then make negative
    if ( !((speedRight > 0 && speedLeft > 0) || (speedRight > 0 && speedLeft > 0)) ) {
      angleChange = -angleChange;
    }
    botAngle += angleChange;

    //if outside of the domain, then fix. [0-2pi]
    if (botAngle > 2*Math.PI) { botAngle -= 2*Math.PI; };
    if (botAngle < 0) { botAngle += 2*Math.PI; }

    //the speed of the bot moving forward/backward is just the difference of the speeds.
    //the variable of forward/backwards is Y
    //if one/both is moving backwards, then the speed would reflect that
    double initialY = speedLeft + speedRight;
    botY += initialY * Math.cos(botAngle); //forward is θ = 0. in which the change is 1:1

    /* Visualizing the Equation
         (Vl^2-Vr^2) 
    Vx = ----------- t
              r

    Note: removing t for reasons explained in paper
    */
    double initialX = (speedLeft*speedLeft - speedRight*speedRight) / radius;
    botX += initialX*Math.sin(botAngle); //forward is θ = 0. in which X wouldn't change
  }

//this is a mode where botAngle stays a constant
  public void calculateGridSwerve(double swerveSpeedY, double swerveSpeedX){
    //for this one, the Y would be exactly the same as normal, as the motors that
    //move it in the Y direction are not being changed
    //there is no need to calcualte initialY as that is already a given
    botY += swerveSpeedY * Math.cos(botAngle);

    //on the other hand, the way it moved in X in normal was due to the difference in the motors
    //since here that difference is the same that would e 0, it instead calculates it based on the 
    //speed of the forward controller. which in this case is calculated like Y
    //this is because the angle of those controlls are the same
    botX += swerveSpeedX * Math.sin(botAngle);
  }

  public int getBallCount(){
    return ballCount;
  }

  public void setOrigin(){
    botX = 0;
    botY = 0;
    //CAREFUL WITH THIS, dependin on initial orientation, this could mean a flip on X and Y,
    //which doesn't change calculations, but it makes it hard to visualize it moving
    botAngle = 0; 
    //remember this is the important: i just didnt want to break previously wrtten code
    imu.reset();
  }

  //this one moves the bot until the angle is 0
  public void resetOrientation(){
    //it will loop until it reached 4% from angle 0
    while (true){
      double rotationSpeed = .5; //idk about best speed, Trial an Error sort of thing?
      //if angle - pi is positive, it means the closest path to orientation is clockwise
      //else it is counter clockwise
      double angleFromZero = imu.getAngle() - 180;
      //double angleFromZero = botAngle - Math.PI;

      //pato, ik you dont know what this means, but it just makes the absolute distance positive
      double absoluteAngleFromZero = angleFromZero > 0 ? angleFromZero : -angleFromZero; 
      double percentDistance = absoluteAngleFromZero / 180;
      //double percentDistance = absoluteAngleFromZero / Math.PI;
      
      //the weighted distance is a multiplier to the turning speed, it should be turning slower as it gets closer to the center
      //so that it doesn't overshoot, but still can turn fast initially
      //it starts the same as the percent distance
      double weightedAngle = absoluteAngleFromZero / 180;
      //double weightedAngle = absoluteAngleFromZero / Math.PI;

      // anything over 50% is full turning speed (will be doubled from .5 to 1.0 soon)
      if (weightedAngle > .5)
        weightedAngle = .5;

      //scale the weight to have a more gradual drop 
      weightedAngle *= 2;

      //slowest speed it can go is 25% of the turning speeds
      if (weightedAngle < .25)
        weightedAngle = .25;

      //if within 4% of the center, it is fine
      if (percentDistance < .02){
        return;
      }//turn clockwise if closer from left angle 
      else if (imu.getAngle() - 180 > 0){ 
      //else if (botAngle - Math.PI > 0){ 
        //the weighted angle makes it move slower as it gets closer
        //leftMotors.set(rotationSpeed * weightedAngle);
        //rightMotors.set(-rotationSpeed * weightedAngle);

        backLeftMotor.set(ControlMode.PercentOutput,-1*rotationSpeed * weightedAngle);
        frontLeftMotor.set(ControlMode.PercentOutput,-1*rotationSpeed * weightedAngle);
        backRightMotor.set(ControlMode.PercentOutput,-rotationSpeed * weightedAngle);
        frontRightMotor.set(ControlMode.PercentOutput,-rotationSpeed * weightedAngle);
      } //otherwise move counter clockwise
      else {
        //leftMotors.set(-rotationSpeed * weightedAngle);
        //rightMotors.set(rotationSpeed * weightedAngle);

        backLeftMotor.set(ControlMode.PercentOutput,-1*-rotationSpeed * weightedAngle);
        frontLeftMotor.set(ControlMode.PercentOutput,-1*-rotationSpeed * weightedAngle);
        backRightMotor.set(ControlMode.PercentOutput, rotationSpeed * weightedAngle);
        frontRightMotor.set(ControlMode.PercentOutput, rotationSpeed * weightedAngle);
      }
    }
  }

  //moves the bot throuh the shortest path
  //TODO have a detect obstacles thing here that can divert its movement
  public void findShortcut(double targetX, double targetY){
    //careful with the while true here
    while (true) {
      //this will also follow a similar method to the resetOrientaton, just that it used values instead of percents
      double movementSpeed = 1; //change with trial and error
      double distanceFromX = targetX - imu.getMagX(); //imu.getGyroInstantX();
      double distanceFromY = targetY - imu.getMagX(); //imu.getGyroInstantY();
      //double distanceFromX = targetX - botX;
      //double distanceFromY = targetY - botY;

      double weightedDistanceX = (distanceFromX > 20) ? 1 : distanceFromX / 20; //20 can be changed
      double weightedDistanceY = (distanceFromY > 20) ? 1 : distanceFromY / 20; //percent distance from 20 units

      double speedX = movementSpeed * weightedDistanceX;
      double speedY = movementSpeed * weightedDistanceY;

      if (distanceFromX > 2 || distanceFromY > 2){
        //frontMotors.set(speedX);

        swerveMotor1.set(speedX);
        swerveMotor2.set(speedX);
        

        //get rid of /2 if too slow
        //leftMotors.set(speedY / 2);
        //rightMotors.set(speedY / 2);
        backLeftMotor.set(ControlMode.PercentOutput, -1*speedY / 2);
        frontLeftMotor.set(ControlMode.PercentOutput, -1*speedY / 2);
        backRightMotor.set(ControlMode.PercentOutput, speedY / 2);
        frontRightMotor.set(ControlMode.PercentOutput, speedY / 2);


        //TODO: add a thing to calculate the movement for Autonomous
        //^ but would that be better found in the autonomous periodic Function?
        //TODO the back motors of Right and Left might need to have a slightly faster speed to compensate
        //for their Angle. this a T&E thing
        //calculateGridSwerve(speedX, speedY);
      } else {
        return; //ends the function as it is already close enough
      }

    }
    
  }
}
