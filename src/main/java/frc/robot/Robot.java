// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Robot extends TimedRobot {

    // Constants
    private static final double DEADZONE = 0.1;
    private static final double SPEED_MOD = 1;
    private static final int TRIGGER = 1;
    private static final int THUMBBUTTON = 2;

    // Joysticks
    private final Joystick joyE = new Joystick(0);
    private final Joystick joyL = new Joystick(2);
    private final Joystick joyR = new Joystick(1);

    // Motor Controllers
    private final WPI_VictorSPX frontLeftMotor = new WPI_VictorSPX(9);
    private final WPI_VictorSPX backLeftMotor = new WPI_VictorSPX(5);
    private final WPI_VictorSPX frontRightMotor = new WPI_VictorSPX(2);
    private final WPI_VictorSPX backRightMotor = new WPI_VictorSPX(3);
    private final WPI_VictorSPX topCollectorMotor = new WPI_VictorSPX(6);
    private final WPI_VictorSPX bottomCollectorMotor = new WPI_VictorSPX(10);
    private final WPI_VictorSPX topArticulatingClimber = new WPI_VictorSPX(7);
    private final WPI_VictorSPX bottomArticulatingClimber = new WPI_VictorSPX(11);
    private final WPI_VictorSPX extendingClimber = new WPI_VictorSPX(4);
    private final WPI_VictorSPX hoodMotor = new WPI_VictorSPX(8);
    private final CANSparkMax shooterMotor = new CANSparkMax(1, MotorType.kBrushless);

    // Motor Controller Groups
    private final MotorControllerGroup leftMotorGroup = new MotorControllerGroup(frontLeftMotor, backLeftMotor);
    private final MotorControllerGroup rightMotorGroup = new MotorControllerGroup(frontRightMotor, backRightMotor);

    private final DifferentialDrive robotDrive = new DifferentialDrive(leftMotorGroup, rightMotorGroup);

    @Override
    public void robotInit() {
        leftMotorGroup.setInverted(true);
        rightMotorGroup.setInverted(false);
    }

    @Override
    public void teleopPeriodic() {

        // Variables
        boolean initialCollectionL = joyL.getRawButton(TRIGGER);
        boolean initialCollectionR = joyR.getRawButton(TRIGGER);
        boolean spinShooter = joyE.getRawButton(TRIGGER);

        boolean ejectButtonL = joyL.getRawButton(THUMBBUTTON);
        boolean ejectButtonR = joyR.getRawButton(THUMBBUTTON);
        boolean feederCollection = joyE.getRawButton(THUMBBUTTON);

        boolean topClimber = joyE.getRawButton(11);
        boolean topClimberOtherway = joyE.getRawButton(7);
        boolean bottomClimber = joyE.getRawButton(12);
        boolean bottomClimberOtherway = joyE.getRawButton(8);
        boolean releaseClimber = joyE.getRawButton(6);
        boolean windClimber = joyE.getRawButton(5);

        double shooterSpeed = (joyE.getRawAxis(4) / 4) + 0.75; // converts [-1, 1] to [-1/4, 1/4] to [0.5, 1]

        // Drivetrain Controls: left and right joysticks
        if (Math.abs(joyL.getY()) > DEADZONE || Math.abs(joyR.getY()) > DEADZONE) {
            robotDrive.tankDrive(joyL.getY() * SPEED_MOD, joyR.getY() * SPEED_MOD);
        } else {
            robotDrive.tankDrive(0, 0);
        }

        // Collection Controls: driver's trigger + thumb button for intake/ejection,
        // operator's thumb button for feeding to spinning shooter wheel
        if (initialCollectionL || initialCollectionR) {
            topCollectorMotor.set(-1);
            bottomCollectorMotor.set(1);
        } else if (feederCollection) {
            topCollectorMotor.set(1);
            bottomCollectorMotor.set(1);
        } else if (ejectButtonL || ejectButtonR) {
            topCollectorMotor.set(-1);
            bottomCollectorMotor.set(-1);
        } else {
            topCollectorMotor.set(0);
            bottomCollectorMotor.set(0);
        }

        // Shooting Motor: controlled by operator's trigger
        shooterMotor.set(spinShooter ? shooterSpeed : 0);
        SmartDashboard.putNumber("Shooter Motor Percentage", shooterSpeed);

        // Articulating Climber Controls: operator buttons 11 & 7
        // TODO: test bottom articulating climber motor!! add to code
        if (topClimber) {
            topArticulatingClimber.set(0.5);
        } else if (topClimberOtherway) {
            topArticulatingClimber.set(-0.5);
        } else {
            topArticulatingClimber.set(0);
        }

        // Extending Climber Winch Controls: operator buttons 5 & 6
        // TODO: does the winch have enough torque to safely lift the robot?
        if (windClimber) {
            extendingClimber.set(1);
        } else if (releaseClimber) {
            extendingClimber.set(-1);
        } else {
            extendingClimber.set(0.0);
        }

    }

}