// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
//import frc.robot.Instrum;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;

public class Arm_MM extends SubsystemBase {
	
	// private static double GEAR_RATIO = 20;
	// private static double MOTOR_COUNTS_PER_REV = 2048;
	private double ForwardSoftLimitThreshold = inches_To_Raw_Sensor_Counts(42);
	private double ReverseSoftLimitThreshold = inches_To_Raw_Sensor_Counts(-0.125);

	private DigitalInput _retractSensor;
	
	WPI_TalonFX _talon = new WPI_TalonFX(Constants.ARM_MOTOR, "rio"); // Rename "rio" to match the CANivore device name if using a
													// CANivore


	/* Used to build string throughout loop */
	//StringBuilder _sb = new StringBuilder();

	/** Creates a new Arm_MM. */
	public Arm_MM() {
		/* Factory default hardware to prevent unexpected behavior */
		_talon.configFactoryDefault();

		/* Configure Sensor Source for Pirmary PID */
		_talon.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, Constants.kPIDLoopIdx,
				Constants.kTimeoutMs);

		/*
		 * set deadband to super small 0.001 (0.1 %).
		 * The default deadband is 0.04 (4 %)
		 */
		_talon.configNeutralDeadband(0.001, Constants.kTimeoutMs);

		/**
		 * Configure Talon FX Output and Sensor direction accordingly Invert Motor to
		 * have green LEDs when driving Talon Forward / Requesting Postiive Output Phase
		 * sensor to have positive increment when driving Talon Forward (Green LED)
		 */
		_talon.setSensorPhase(false);
		_talon.setInverted(true);
		/*
		 * Talon FX does not need sensor phase set for its integrated sensor
		 * This is because it will always be correct if the selected feedback device is
		 * integrated sensor (default value)
		 * and the user calls getSelectedSensor* to get the sensor's position/velocity.
		 * 
		 * https://phoenix-documentation.readthedocs.io/en/latest/ch14_MCSensor.html#
		 * sensor-phase
		 */
		// _talon.setSensorPhase(true);

		/* Set relevant frame periods to be at least as fast as periodic rate */
		_talon.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, Constants.kTimeoutMs);
		_talon.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, Constants.kTimeoutMs);

		/* Set the peak and nominal outputs */
		_talon.configNominalOutputForward(0, Constants.kTimeoutMs);
		_talon.configNominalOutputReverse(0, Constants.kTimeoutMs);
		_talon.configPeakOutputForward(1, Constants.kTimeoutMs);
		_talon.configPeakOutputReverse(-1, Constants.kTimeoutMs);

		/* Set Motion Magic gains in slot0 - see documentation */
		_talon.selectProfileSlot(Constants.kSlotIdx, Constants.kPIDLoopIdx);
		_talon.config_kF(Constants.kSlotIdx, 0.00, Constants.kTimeoutMs);
		_talon.config_kP(Constants.kSlotIdx, 0.1, Constants.kTimeoutMs);
		_talon.config_kI(Constants.kSlotIdx, 0.0, Constants.kTimeoutMs);
		_talon.config_kD(Constants.kSlotIdx, 0.05, Constants.kTimeoutMs);

		/* Set acceleration and vcruise velocity - see documentation */
		_talon.configMotionCruiseVelocity(18000, Constants.kTimeoutMs);
		_talon.configMotionAcceleration(12000, Constants.kTimeoutMs);

		/* Zero the sensor once on robot boot up */
		_talon.setSelectedSensorPosition(0, Constants.kPIDLoopIdx, Constants.kTimeoutMs);

		_talon.configForwardSoftLimitThreshold(ForwardSoftLimitThreshold, Constants.kTimeoutMs);
		_talon.configForwardSoftLimitEnable(true, 0);
		
		_talon.configReverseSoftLimitThreshold(ReverseSoftLimitThreshold, Constants.kTimeoutMs);
		_talon.configReverseSoftLimitEnable(true, 0);
		// _talon.configClearPositionOnLimitR(true, 0);

		// Assign optical sensor to digital channel 0 
		_retractSensor = new DigitalInput(0);
	}

	@Override
	public void periodic() {
		double vel;
		double pos;
		vel = _talon.getSelectedSensorVelocity();
		pos = _talon.getSelectedSensorPosition();
		if (DisableRetractMotion()) {
			my_PercentOutput_Run(0);
		}
		SmartDashboard.putNumber("Current Inches", my_getInches());
		SmartDashboard.putNumber("Arm Encoder", pos);
		SmartDashboard.putBoolean("Rectraction Allowed", retractAllowed());
		SmartDashboard.putNumber("Arm Velocity", vel);
		//double motorOutput = _talon.getMotorOutputPercent();

		/* Prepare line to print */
	

		/* Instrumentation */
		//Instrum.Process(_talon, _sb);
	}

	double m_targetPos = 0.0;

	/**
	 * Input in deg
	 * 
	 * @param deg
	 */
	public void my_motionMagic_Run(double inches) {
		/* Motion Magic */
		m_targetPos = inches;
		
		/* 2048 ticks/rev * 10 Rotations in either direction */
		double targetPos = inches_To_Raw_Sensor_Counts(inches);// / 360 * MOTOR_COUNTS_PER_REV * GEAR_RATIO;
		_talon.set(TalonFXControlMode.MotionMagic, targetPos);
	}

	/**
	 * input -1 to 1
	 * 
	 * @param speed
	 */
	public void my_PercentOutput_Run(double speed) {
		_talon.set(TalonFXControlMode.PercentOutput, speed);
	}

	public void my_resetEncoder() {
		_talon.setSelectedSensorPosition(0, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
	}

	public double my_getInches() {
		return raw_Sensor_Counts_To_Inches(_talon.getSelectedSensorPosition());// * 360 / (GEAR_RATIO * MOTOR_COUNTS_PER_REV);
	}

	/**
	 * Test if in position
	 * 
	 * @param setpoint // in inches
	 * 
	 * @return
	 */
	public boolean my_get_PositionLock(double setpoint) {
		double positionLockTollerence = 1;
		double error = my_getInches() - setpoint;

		if (Math.abs(error) <= positionLockTollerence) {
			return true;
		} else {
			return false;
		}
	}

	private double raw_Sensor_Counts_To_Inches(double counts){
		return (0.0000790526*counts + 0.320624629);
		//return counts * 360 / (GEAR_RATIO * MOTOR_COUNTS_PER_REV);

	}

	private double inches_To_Raw_Sensor_Counts(double inches){
		return inches * 12635.203 - 3861;
		//return inches / 360 * MOTOR_COUNTS_PER_REV * GEAR_RATIO;
	}

	public boolean DisableRetractMotion()  {
		double sensorvelocity;
		sensorvelocity = _talon.getSelectedSensorVelocity();
		return (!_retractSensor.get() && sensorvelocity < 0);
	}

	public boolean retractAllowed()  {
		return _retractSensor.get();
	}
}