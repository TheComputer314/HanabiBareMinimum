// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.swerve.Swerve;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {
  private static final String DEFAULT_AUTO = "Default";
  private static final String CUSTOM_AUTO = "My Auto";
  private String autoSelected;
  private final SendableChooser<String> chooser = new SendableChooser<>();

  private final Swerve swerve;

  private final XboxController driverController;

  /**
   * This method is run when the robot is first started up and should be used for any initialization
   * code.
   */
  public Robot() {
    chooser.setDefaultOption("Default Auto", DEFAULT_AUTO);
    chooser.addOption("My Auto", CUSTOM_AUTO);
    SmartDashboard.putData("Auto choices", chooser);

    swerve = new Swerve();
    driverController = new XboxController(0);
  }

  /**
   * This method is called every 20 ms, no matter the mode. Use this for items like diagnostics that
   * you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic methods, but before LiveWindow and SmartDashboard
   * integrated updating.
   */
  @Override
  public void robotPeriodic() {}

  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    autoSelected = chooser.getSelected();
    // autoSelected = SmartDashboard.getString("Auto Selector", DEFAULT_AUTO);
    System.out.println("Auto selected: " + autoSelected);
  }

  /** This method is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    switch (autoSelected) {
      case CUSTOM_AUTO:
        // Put custom auto code here
        break;
      case DEFAULT_AUTO:
      default:
        // Put default auto code here
        break;
    }
  }

  /** This method is called once when teleop is enabled. */
  @Override
  public void teleopInit() {}

  /** This method is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    // WPILibの数学は+X=前、+Y=左なんだけど、コントローラーのスティックは+X=右、+Y=下なの。
    // あとスティックドリフトはありだからMathUtil.applyDeadbandで小さい入力は無視
    // ドライバーの入力は+-1だからそれ x 最大速度 = ターゲットの速度
    swerve.drive(
        new ChassisSpeeds(
            MathUtil.applyDeadband(-driverController.getLeftY(), .05)
                * Swerve.maxLinearVelocityMetersPerSec,
            MathUtil.applyDeadband(-driverController.getLeftX(), .05)
                * Swerve.maxLinearVelocityMetersPerSec,
            MathUtil.applyDeadband(-driverController.getRightX(), .05)
                * Swerve.maxAngularVelocityRadiansPerSec));
  }

  /** This method is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This method is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}
}
