/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019-2020 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;

/**
 * Represents a differential drive style drivetrain.
 */
public class Drivetrain {

  //Constants
  //Ports
  public static final int leftMasterPort = 0;
  public static final int leftFollowerPort = 0;
  public static final int rightMasterPort = 0;
  public static final int rightFollowerPort = 0;
  public static final int gyroPort = 0;

  //Numerical Constants
  public static final double kMaxSpeed = 3.0; // meters per second->needs to be inputted
  public static final double kMaxAngularSpeed = 2 * Math.PI; // one rotation per second
  private static final double kTrackWidth = 0.381 * 2; // meters->needs to be inputted
  private static final double kWheelRadius = 0.0762; // meters->needs to be inputted
  private static final int kEncoderResolution = 2048; // <-precision for integrated talon FX encoder
  public static final double distancePerPulseFactor = (2 * Math.PI * kWheelRadius)/ kEncoderResolution;


  //Motors/Sensors
  private final TalonFX leftMaster = new TalonFX(leftMasterPort);
  private final TalonFX leftFollower = new TalonFX(leftFollowerPort);
  private final TalonFX rightMaster = new TalonFX(rightMasterPort);
  private final TalonFX rightFollower = new TalonFX(rightFollowerPort);
  // private final SpeedController leftMaster = (SpeedController) new TalonFX(leftMasterPort);
  // private final SpeedController leftFollower = (SpeedController) new TalonFX(leftFollowerPort);
  // private final SpeedController rightMaster = (SpeedController) new TalonFX(rightMasterPort);
  // private final SpeedController rightFollower = (SpeedController) new TalonFX(rightFollowerPort);

  //Defintion of SpeedController Group
  // private final SpeedControllerGroup m_leftGroup = new SpeedControllerGroup(leftMaster, leftFollower);
  // private final SpeedControllerGroup m_rightGroup = new SpeedControllerGroup(rightMaster, rightFollower);


  //Setup for Talons/Gyro
  final TalonFXInvertType kFxInvertType = TalonFXInvertType.Clockwise;
  final NeutralMode kBrakeDurNeutral = NeutralMode.Brake;
  private final AnalogGyro m_gyro = new AnalogGyro(gyroPort);

  
  //PID/Kinematics/Odometery
  private final PIDController m_leftPIDController = new PIDController(1, 0, 0);
  private final PIDController m_rightPIDController = new PIDController(1, 0, 0);

  private final DifferentialDriveKinematics m_kinematics
      = new DifferentialDriveKinematics(kTrackWidth);

  private final DifferentialDriveOdometry m_odometry;

  // Gains are for example purposes only - must be determined for your own robot!
  private final SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(1, 3);

  
  


  /**
   * Constructs a differential drive object.
   * Sets the encoder distance per pulse and resets the gyro.
   */
  public Drivetrain() {
    m_gyro.reset();

    TalonFXConfiguration configs = new TalonFXConfiguration();

    configs.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;

    //Left side
    leftMaster.configAllSettings(configs);
    leftMaster.setInverted(false);
    leftMaster.setNeutralMode(kBrakeDurNeutral);

    leftFollower.configAllSettings(configs);
    leftFollower.setInverted(false);
    leftFollower.setNeutralMode(kBrakeDurNeutral);
    leftFollower.follow(leftMaster);

    //Right side
    rightMaster.configAllSettings(configs);
    rightMaster.setInverted(false);
    rightMaster.setNeutralMode(kBrakeDurNeutral);

    
    rightFollower.configAllSettings(configs);
    rightFollower.setInverted(false);
    rightFollower.setNeutralMode(kBrakeDurNeutral);
    rightFollower.follow(rightMaster);


    //reset(); -> Zero the encoders

    // Set the distance per pulse for the drive encoders. We can simply use the
    // distance traveled for one rotation of the wheel divided by the encoder
    // resolution.

    // int selSenPos = talon10.getSelectedSensorPosition(0); /* position units */
    // int selSenVel = talon10.getSelectedSensorVelocity(0);

    //Setting distance/pulse -> need to figure out 
    // m_leftEncoder.setDistancePerPulse(2 * Math.PI * kWheelRadius / kEncoderResolution);
    // m_rightEncoder.setDistancePerPulse(2 * Math.PI * kWheelRadius / kEncoderResolution);

    // m_leftEncoder.reset();
    // m_rightEncoder.reset();

    leftMaster.setSelectedSensorPosition(0); //setting initial gyro positions to 0
    rightMaster.setSelectedSensorPosition(0);

    m_odometry = new DifferentialDriveOdometry(getAngle());
  }





  
  /**
   * Returns the angle of the robot as a Rotation2d.
   *
   * @return The angle of the robot. -> Pigeon IMU
   */
  public Rotation2d getAngle() {
    // Negating the angle because WPILib gyros are CW positive.
    return Rotation2d.fromDegrees(-m_gyro.getAngle()); //-> need to replace w/ the Pigeon IMU
  }

  /**
   * Sets the desired wheel speeds.
   *
   * @param speeds The desired wheel speeds.
   */

  public void setSpeeds(DifferentialDriveWheelSpeeds speeds) {
    final double leftFeedforward = m_feedforward.calculate(speeds.leftMetersPerSecond);
    final double rightFeedforward = m_feedforward.calculate(speeds.rightMetersPerSecond);

    final double leftOutput = m_leftPIDController.calculate(leftMaster.getSelectedSensorVelocity() * distancePerPulseFactor,
        speeds.leftMetersPerSecond);
    final double rightOutput = m_rightPIDController.calculate(rightMaster.getSelectedSensorVelocity() * distancePerPulseFactor,
        speeds.rightMetersPerSecond);

    leftMaster.set(ControlMode.Current, leftOutput + leftFeedforward);
    rightMaster.set(ControlMode.Current, rightOutput + rightFeedforward);
  }

  /**
   * Drives the robot with the given linear velocity and angular velocity.
   *
   * @param xSpeed Linear velocity in m/s.
   * @param rot    Angular velocity in rad/s.
   */
  @SuppressWarnings("ParameterName")
  public void drive(double xSpeed, double rot) {
    var wheelSpeeds = m_kinematics.toWheelSpeeds(new ChassisSpeeds(xSpeed, 0.0, rot));
    setSpeeds(wheelSpeeds);
  }

  /**
   * Updates the field-relative position.
   */
  public void updateOdometry() {
    m_odometry.update(getAngle(), leftMaster.getSelectedSensorPosition()*distancePerPulseFactor, rightMaster.getSelectedSensorPosition()*distancePerPulseFactor);
  }
}
