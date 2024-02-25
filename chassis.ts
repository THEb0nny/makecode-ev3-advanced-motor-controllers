enum MeasurementUnit {
    //% block="cm"
    Centimeters,
    //% block="mm"
    Millimeters
}

/**
 * A differential drive robot
 */
//% color="#00751B" weight=89 icon="\uf1b9"
namespace chassis {

    let chassisMotors: motors.SynchedMotorPair; // The motors pair
    let chassisLeftMotor: motors.MotorBase; // The left motor in chassis
    let chassisRightMotor: motors.MotorBase; // The right motor in chassis

    let motorMaxRPM: number = 0; // Motor maximum rpm
    let wheelRadius: number = 0; // The radius of the wheel (cm)
    let baseLength: number = 0; // The distance between the wheels (cm)

    let syncKp: number = 0.03;
    let syncKi: number = 0;
    let syncKd: number = 0.5;

    const pidChassisSync = new automation.PIDController(); // PID for sync loop

    /**
     * Sets the motors used by the chassis, default is large B+C.
     * @param motors motors pair, eg: motors.largeBC
     **/
    //% blockId=chassisSetMotors block="set motors to chassis $motors"
    //% motors.fieldEditor="motors"
    //% motors.fieldOptions.decompileLiterals=1
    //% inlineInputMode=inline
    //% weight=89
    //% group="Properties"
    export function setChassisMotors(motorsPair: motors.SynchedMotorPair) {
        chassisMotors = motorsPair;
        const motorsType = chassisMotors.toString().split(" ")[0];
        if (motorsType == "large") {
            motorMaxRPM = 170;
            //chassisLeftMotor = motors.largeB;
            //chassisRightMotor = motors.largeC;
        } else {
            motorMaxRPM = 250;
        }
        //motorMaxRPM = (motorsType == "large" ? 170 : 250);
        console.log(`motorsType: ${motorsType}`);
    }

    /**
     * Sets the wheel radius.
     * @param radius the radius of a wheel, eg: 56 (mm)
     * @param unit dimension of the unit of radius, eg: MeasurementUnit.Millimeters
     */
    //% blockId=chassisSetWheelRadius block="set wheel radius to $radius $unit"
    //% weight=88 blockGap=8
    //% group="Properties"
    export function setWheelRadius(radius: number, unit: MeasurementUnit = MeasurementUnit.Millimeters) {
        if (unit == MeasurementUnit.Centimeters) wheelRadius = radius;
        else if (unit == MeasurementUnit.Millimeters) wheelRadius = radius / 10;
        else return;
    }

    /**
     * Gets the wheel radius.
     * @param unit dimension of the unit of length, eg: MeasurementUnit.Millimeters
     */
    //% blockId=chassisGetWheelRadius block="get wheel radius $unit"
    //% weight=87
    //% group="Properties"
    export function getWheelRadius(unit: MeasurementUnit): number {
        if (unit == MeasurementUnit.Centimeters) return wheelRadius;
        else if (unit == MeasurementUnit.Millimeters) return wheelRadius * 10;
        else return 0;
    }

    /**
     * Sets the base length.
     * @param length the base length, eg: 130 (mm)
     * @param unit dimension of the unit of length, eg: MeasurementUnit.Millimeters
     */
    //% blockId=chassisSetBaseLength block="set base length to $length $unit"
    //% weight=86 blockGap=8
    //% group="Properties"
    export function setBaseLength(length: number, unit: MeasurementUnit = MeasurementUnit.Millimeters) {
        if (unit == MeasurementUnit.Centimeters) baseLength = length;
        else if (unit == MeasurementUnit.Millimeters) baseLength = length / 10;
        else return;
    }

    /**
     * Gets the base length.
     * @param unit dimension of the unit of length, eg: MeasurementUnit.Millimeters
     */
    //% blockId=chassisGetBaseLength block="get base length $unit"
    //% weight=85
    //% group="Properties"
    export function getBaseLength(unit: MeasurementUnit = MeasurementUnit.Millimeters) {
        if (unit == MeasurementUnit.Centimeters) return baseLength;
        else if (unit == MeasurementUnit.Millimeters) return baseLength * 10;
        else return 0;
    }

    /**
        Set the chassis timing control values.
        @param syncKp kp input value, eg. 1
        @param syncKi ki input value, eg. 0
        @param syncKd kd input value, eg. 0
    **/
    //% blockId=SetRegulatorGains
    //% block="set chassis sync pid gains kp = $Kp|ki = $Ki|kd = $Kd"
    //% group="Properties"
    //% inlineInputMode=inline
    export function SetRegulatorGains(Kp: number, Ki: number, Kd: number) {
        syncKp = Kp;
        syncKi = Ki;
        syncKd = Kd;
    }

    /**
     * Makes a differential drive robot move with a given speed (cm/s) and rotation rate (deg/s) using a unicycle model.
     * @param speed speed of the center point between motors, eg: 10
     * @param rotationSpeed rotation of the robot around the center point, eg: 30
     * @param distance driving distance, eg: 150 (cm)
     * @param unit dimension of the unit of movement, eg: MeasurementUnit.Millimeters
     **/
    //% blockId=chassisDrive block="drive at $speed cm/s turning $rotationSpeed deg/s for $distance|$unit"
    //% inlineInputMode=inline
    //% weight=99 blockGap=8
    //% rotationSpeed.min=-3200 rotationSpeed.max=3200
    //% group="Move"
    export function drive(speed: number, rotationSpeed: number, distance: number = 0, unit: MeasurementUnit = MeasurementUnit.Millimeters) {
        if (!chassisMotors || wheelRadius == 0 || baseLength == 0 || motorMaxRPM == 0) return;
        if (!speed) {
            chassisMotors.stop();
            return;
        }

        // speed is expressed in %
        const R = wheelRadius; // cm
        const L = baseLength; // cm
        const maxw = motorMaxRPM / 60 * 2 * Math.PI; // rad/s
        const maxv = maxw * R; // cm/s

        const v = speed; // speed is cm/s
        const w = rotationSpeed / 360 * 2 * Math.PI; // rad/s

        const vr = (2 * v + w * L) / (2 * R); // rad/s
        const vl = (2 * v - w * L) / (2 * R); // rad/s

        const sr = vr / maxw * 100; // %
        const sl = vl / maxw * 100; // %

        if (distance != 0 && unit == MeasurementUnit.Millimeters) distance / 10; // mm to cm
        const seconds = distance / speed; // cm / (cm/s) = s

        chassisMotors.tank(sr, sl, seconds, MoveUnit.Seconds);
    }

    /**
        Synchronization of motors in the chassis with setting speeds for each motor. No acceleration or deceleration support.
        @param vLeft left motor speed input value, eg. 50
        @param vRight right motor speed input value, eg. 50
        @param vRight right motor speed input value, eg. 50
        @param value move duration or rotation
        @param unit unit of the value, eg. MoveUnit.Degrees
    **/
    //% blockId=SyncChassisMovement
    //% block="sync chassis movement at $vLeft=motorSpeedPicker|\\%|$vRight=motorSpeedPicker|\\%|for value = $value $unit"
    //% inlineInputMode=inline
    //% weight=98 blockGap=8
    //% group="Move"
    export function SyncChassisMovement(vLeft: number, vRight: number, value: number, unit: MoveUnit = MoveUnit.Degrees) {
        vLeft = Math.clamp(-100, 100, vLeft >> 0); // We limit the speed of the left motor from -100 to 100 and cut off the fractional part
        vRight = Math.clamp(-100, 100, vRight >> 0); // We limit the speed of the right motor from -100 to 100 and cut off the fractional part
        if (unit == MoveUnit.Rotations) value /= 360; // Convert degrees to revolutions if the appropriate mode is selected

        advmotctrls.SyncMotorsConfig(vLeft, vRight); // Set motor speeds for subsequent regulation

        pidChassisSync.setGains(syncKp, syncKi, syncKd); // Setting the regulator coefficients
        pidChassisSync.setControlSaturation(-100, 100); // Regulator limitation
        pidChassisSync.reset(); // Reset pid controller

        let prevTime = 0; // Last time time variable for loop
        const startTime = control.millis() * (unit == MoveUnit.Seconds ? 0.001 : 1); // We fix the time before the start of the regulation cycle
        const endTime = (unit == MoveUnit.MilliSeconds || unit == MoveUnit.Seconds ? startTime + value : 0); // We record the end time of the regulation cycle if the appropriate mode is selected
        while (true) { // Synchronized motion control cycle
            let currTime = control.millis();
            let dt = currTime - prevTime;
            prevTime = currTime;

            let encB = motors.mediumB.angle(); // Get left motor encoder current value
            let encC = motors.mediumC.angle(); // Get right motor encoder current value

            if (unit == MoveUnit.Degrees) {
                if ((encB + encC) / 2 >= value) break;
            } else if (unit == MoveUnit.Rotations) {
                if (((encB + encC) / 2) / 360 >= value) break;
            } else if (unit == MoveUnit.MilliSeconds) {
                if (control.millis() >= endTime) break;
            } else if (unit == MoveUnit.Seconds) {
                if (control.millis() * 0.001 >= endTime) break;
            }

            let error = advmotctrls.GetErrorSyncMotors(encB, encC); // Find out the error in motor speed control
            pidChassisSync.setPoint(error); // Transfer control error to controller
            let U = pidChassisSync.compute(dt, 0); // Find out and record the control action of the regulator
            let powers = advmotctrls.GetPwrSyncMotors(U); // Find out the power of motors for regulation
            motors.mediumB.run(powers.pwrLeft); // Set power/speed left motor
            motors.mediumC.run(powers.pwrRight); // Set power/speed right motor
            control.PauseUntilTime(currTime, 5); // Wait until the control cycle reaches the set amount of time passed
        }
        motors.mediumB.stop(); // Stop left motor
        motors.mediumC.stop(); // Stop right motor
        //motors.mediumBC.stop(); // Остановить моторы
    }
    
}

namespace control {

    export function PauseUntilTime(startTime: number, ms: number) {
        if (startTime == 0) startTime = control.millis();
        while (control.millis() < startTime + ms);
    }

}