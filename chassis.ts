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

    export let motorsPair: motors.SynchedMotorPair; // The motors pair
    export let leftMotor: motors.Motor; // The left motor in chassis
    export let rightMotor: motors.Motor; // The right motor in chassis

    let motorMaxRPM: number = 0; // Motor maximum rpm
    let wheelRadius: number = 0; // The radius of the wheel (cm)
    let baseLength: number = 0; // The distance between the wheels (cm)

    let syncKp: number = 0.03; // Proportional synchronization gain
    let syncKi: number = 0; // Integral synchronization gain
    let syncKd: number = 0.5; // Differential synchronization gain

    export const pidChassisSync = new automation.PIDController(); // PID for sync motors chassis loop

    /**
     * Sets the motors used by the chassis. If necessary, you can immediately set the reverse properties.
     * @param newMotorsPair motors pair, eg: motors.largeBC
     * @param setLeftMotReverse left motor reverse property, eg: false
     * @param setRightMotReverse right motor reverse property, eg: true
     */
    //% blockId=ChassisSetMotors
    //% block="set motors to chassis $newMotorsPair||at reverse property $setLeftMotReverse|$setRightMotReverse"
    //% block.loc.ru="установить моторы шасси $newMotorsPair||с свойством реверса $setLeftMotReverse|$setRightMotReverse"
    //% newMotorsPair.fieldEditor="motors"
    //% newMotorsPair.fieldOptions.decompileLiterals=1
    //% setLeftMotReverse.shadow="toggleOnOff"
    //% setRightMotReverse.shadow="toggleOnOff"
    //% inlineInputMode=inline
    //% expandableArgumentMode="toggle"
    //% weight=89
    //% group="Properties"
    export function setChassisMotors(newMotorsPair: motors.SynchedMotorPair, setLeftMotReverse?: boolean, setRightMotReverse?: boolean) {
        motorsPair = newMotorsPair;
        const motorsName = motorsPair.toString();
        const motorsType = motorsName.split(" ")[0];
        const motorsNameArr = motorsName.split(" ")[1].split("+");
        const allUsedSingleMotors = motors.Motor.getAllInstances(); // Get all motors instances
        // console.log(`allUseMotors: ${allUsedSingleMotors.length}`);
        if (allUsedSingleMotors.length >= 2) { // Ищем из существующих моторов
            leftMotor = allUsedSingleMotors.filter((motor) => motor.toString().split(" ")[1] == motorsNameArr[0])[0]; // Set left motor instance
            rightMotor = allUsedSingleMotors.filter((motor) => motor.toString().split(" ")[1] == motorsNameArr[1])[0]; // Set right motor instance
        }
        if (!leftMotor || !rightMotor) { // Если моторы не были найдены, тогда уже создать свои классы
            const motorsOut = motors.splitDoubleOutput(strNameToOutput(motorsName));
            const isLargeMotor = (motorsType == "large" ? true : false);
            if (!leftMotor) leftMotor = new motors.Motor(motorsOut[0], isLargeMotor);
            if (!rightMotor) rightMotor = new motors.Motor(motorsOut[1], isLargeMotor);
        }
        console.log(`reverse ${setLeftMotReverse}, ${setRightMotReverse}`);
        if (setLeftMotReverse != undefined) {
            leftMotor.setInverted(setLeftMotReverse);
            console.log(`reverse ${setLeftMotReverse}`);
        }
        if (setRightMotReverse != undefined) {
            rightMotor.setInverted(setRightMotReverse);
            console.log(`reverse ${setRightMotReverse}`);
        }
        // console.log(`leftMotor: ${leftMotor}`);
        // console.log(`rightMotor: ${rightMotor}`);
        if (motorsType == "large") motorMaxRPM = 170;
        else if (motorsType == "medium") motorMaxRPM = 250;
    }

    function strNameToOutput(outStr: string): Output {
        if (outStr == "B+C") return Output.BC;
        else if (outStr == "A+B") return Output.AB;
        else if (outStr == "C+D") return Output.CD;
        else if (outStr == "A+D") return Output.AD;
        return Output.ALL;
    }

    /*
    // Only a double output at a time
    function splitDoubleOutput(out: Output): Output[] {
        if (out == Output.BC) return [Output.B, Output.C];
        else if (out == Output.AB) return [Output.A, Output.B];
        else if (out == Output.CD) return [Output.C, Output.D];
        else if (out == Output.AD) return [Output.A, Output.D];
        return [];
    }
    */

    /**
     * Sets the wheel radius.
     * @param radius the radius of a wheel, eg: 56 (mm)
     * @param unit dimension of the unit of radius, eg: MeasurementUnit.Millimeters
     */
    //% blockId=ChassisSetWheelRadius
    //% block="set wheel radius = $radius|$unit"
    //% block.loc.ru="установить радиус колёс шасси $radius|$unit"
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
    //% blockId=ChassisGetWheelRadius
    //% block="get wheel radius $unit"
    //% block.loc.ru="получить радиус колёс шасси в $unit"
    //% weight=87
    //% group="Properties"
    export function getWheelRadius(unit: MeasurementUnit = MeasurementUnit.Millimeters): number {
        if (unit == MeasurementUnit.Centimeters) return wheelRadius;
        else if (unit == MeasurementUnit.Millimeters) return wheelRadius * 10;
        else return 0;
    }

    /**
     * Sets the base length.
     * @param length the base length, eg: 130 (mm)
     * @param unit dimension of the unit of length, eg: MeasurementUnit.Millimeters
     */
    //% blockId=ChassisSetBaseLength
    //% block="set base length = $length|$unit"
    //% block.loc.ru="установить размер коллеи шасси = $length|$unit"
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
    //% blockId=ChassisGetBaseLength
    //% block="get base length $unit"
    //% block.loc.ru="получить размер коллеи шасси в $unit"
    //% weight=85
    //% group="Properties"
    export function getBaseLength(unit: MeasurementUnit = MeasurementUnit.Millimeters) {
        if (unit == MeasurementUnit.Centimeters) return baseLength;
        else if (unit == MeasurementUnit.Millimeters) return baseLength * 10;
        else return 0;
    }

    /**
        Set the chassis synchronization control values.
        @param kp sync kp input value, eg. 0.03
        @param ki sync ki input value, eg. 0
        @param kd sync kd input value, eg. 0.5
     */
    //% blockId=SetRegulatorGains
    //% block="set chassis sync pid gains kp = $Kp|ki = $Ki|kd = $Kd"
    //% block.loc.ru="установить коэффиценты синхронизации шасси kp = $Kp|ki = $Ki|kd = $Kd"
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
     */
    //% blockId=ChassisDrive
    //% block="drive at $speed cm/s turning $rotationSpeed deg/s for $distance|$unit"
    //% block.loc.ru="движение $speed см/с поворотом $rotationSpeed град/с на дистанцию $distance|$unit"
    //% inlineInputMode=inline
    //% weight=99 blockGap=8
    //% rotationSpeed.min=-3200 rotationSpeed.max=3200
    //% group="Move"
    export function Drive(speed: number, rotationSpeed: number, distance: number = 0, unit: MeasurementUnit = MeasurementUnit.Millimeters) {
        if (!motorsPair) return;
        if (!speed || wheelRadius == 0 || baseLength == 0 || motorMaxRPM == 0) {
            ChassisStop(true);
            return;
        }

        // Speed is expressed in %
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

        motorsPair.tank(sr, sl, seconds, MoveUnit.Seconds);
    }

    /**
        Synchronization of motors in chassis with setting speeds for each motor. No acceleration or deceleration support.
        @param vLeft left motor speed input value, eg. 50
        @param vRight right motor speed input value, eg. 50
        @param vRight right motor speed input value, eg. 50
        @param value move duration or rotation
        @param unit unit of the value, eg. MoveUnit.Degrees
     */
    //% blockId=SyncChassisMovement
    //% block="sync chassis movement at $vLeft=motorSpeedPicker|\\%|$vRight=motorSpeedPicker|\\%|for value = $value|$unit"
    //% block.loc.ru="синхронизированное управление шасси с $vLeft=motorSpeedPicker|\\%|$vRight=motorSpeedPicker|\\%|на $value|$unit"
    //% inlineInputMode=inline
    //% weight=98 blockGap=8
    //% group="Move"
    export function SyncChassisMovement(vLeft: number, vRight: number, value: number, unit: MoveUnit = MoveUnit.Degrees) {
        if (!motorsPair) return;
        if (vLeft == 0 && vRight == 0 || ((unit == MoveUnit.Rotations || unit == MoveUnit.Degrees) && value == 0) || ((unit == MoveUnit.Seconds || unit == MoveUnit.MilliSeconds) && value <= 0)) {
            ChassisStop(true);
            return;
        }
        vLeft = Math.clamp(-100, 100, vLeft >> 0); // We limit the speed of the left motor from -100 to 100 and cut off the fractional part
        vRight = Math.clamp(-100, 100, vRight >> 0); // We limit the speed of the right motor from -100 to 100 and cut off the fractional part
        const emlPrev = leftMotor.angle(); // We read the value from the encoder from the left motor before starting
        const emrPrev = rightMotor.angle(); // We read the value from the encoder from the right motor before starting
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
            let eml = leftMotor.angle() - emlPrev; // Get left motor encoder current value
            let emr = rightMotor.angle() - emrPrev; // Get right motor encoder current value
            if (unit == MoveUnit.Degrees || unit == MoveUnit.Rotations) {
                if (Math.abs(eml) >= Math.abs(value) && Math.abs(emr) >= Math.abs(value)) break;
            } else if (unit == MoveUnit.MilliSeconds) {
                if (control.millis() >= endTime) break;
            } else if (unit == MoveUnit.Seconds) {
                if (control.millis() * 0.001 >= endTime) break;
            }
            let error = advmotctrls.GetErrorSyncMotors(eml, emr); // Find out the error in motor speed control
            pidChassisSync.setPoint(error); // Transfer control error to controller
            let U = pidChassisSync.compute(dt, 0); // Find out and record the control action of the regulator
            let powers = advmotctrls.GetPwrSyncMotors(U); // Find out the power of motors for regulation
            leftMotor.run(powers.pwrLeft); // Set power/speed left motor
            rightMotor.run(powers.pwrRight); // Set power/speed right motor
            control.PauseUntilTime(currTime, 5); // Wait until the control cycle reaches the set amount of time passed
        }
        ChassisStop(true);
    }

    /**
        Synchronized rotation of the chassis relative to the center at the desired angle at a certain speed. For example, if degress > 0, then the robot will rotate to the right, and if degress < 0, then to the left.
        @param degress rotation value in degrees, eg. 90
        @param speed turning speed value, eg. 40
     */
    //% blockId=ChassisSpinTurn
    //% block="sync chassis spin turn at degress = $degress|°|for speed = $speed|\\%"
    //% block.loc.ru="синхронизированный поворот шасси на угол = $degress|°|со скоростью = $speed|\\%"
    //% inlineInputMode=inline
    //% weight=97 blockGap=8
    //% group="Move"
    export function ChassisSpinTurn(degress: number, speed: number) {
        if (!motorsPair) return;
        if (degress == 0 || speed <= 0) {
            ChassisStop(true);
            return;
        }
        speed = Math.clamp(-100, 100, speed >> 0); // We limit the speed of the motor from -100 to 100 and cut off the fractional part
        const emlPrev = leftMotor.angle(); // We read the value from the encoder from the left motor before starting
        const emrPrev = rightMotor.angle(); // We read the value from the encoder from the right motor before starting
        const calcMotRot = Math.round(degress * getBaseLength() / getWheelRadius()); // Расчёт угла поворота моторов для поворота
        if (degress > 0) advmotctrls.SyncMotorsConfig(speed, -speed);
        else if (degress < 0) advmotctrls.SyncMotorsConfig(-speed, speed);
        pidChassisSync.setGains(syncKp, syncKi, syncKd); // Setting the regulator coefficients
        pidChassisSync.setControlSaturation(-100, 100); // Regulator limitation
        pidChassisSync.reset(); // Reset pid controller
        let prevTime = 0;
        while (true) {
            let currTime = control.millis();
            let dt = currTime - prevTime;
            prevTime = currTime;
            let eml = chassis.leftMotor.angle() - emlPrev;
            let emr = chassis.rightMotor.angle() - emrPrev;
            if ((Math.abs(eml) + Math.abs(emr)) / 2 >= Math.abs(calcMotRot)) break;
            let error = advmotctrls.GetErrorSyncMotors(eml, emr);
            chassis.pidChassisSync.setPoint(error);
            let U = chassis.pidChassisSync.compute(dt, 0);
            let powers = advmotctrls.GetPwrSyncMotors(U);
            leftMotor.run(powers.pwrLeft);
            rightMotor.run(powers.pwrRight);
            control.PauseUntilTime(currTime, 5);
        }
        ChassisStop(true);
    }

    /**
        Stop the chassis motors.
        @param setBrake hold the motors when braking, eg. true
     */
    //% blockId=ChassisStop
    //% block="chassis stop||at hold $setBrake"
    //% block.loc.ru="остановить шасси||с удержанием $setBrake"
    //% setBrake.shadow="toggleOnOff"
    //% inlineInputMode=inline
    //% expandableArgumentMode=toggle
    //% weight=96 blockGap=8
    //% group="Move"
    export function ChassisStop(setBrake?: boolean) {
        if (!motorsPair) return;
        if (setBrake) {
            motorsPair.setBrake(setBrake);
            leftMotor.setBrake(setBrake);
            rightMotor.setBrake(setBrake);
        }
        motorsPair.stop();
        // leftMotor.setBrakeSettleTime(0);
        // leftMotor.stop();
        // rightMotor.stop();
        // leftMotor.setBrakeSettleTime(10);
    }
    
}

namespace control {

    /**
     * Function to wait for a loop to complete for a specified time.
     * @param startTime start time, eg: 0
     * @param delay waiting time, eg: 10
     */
    //% blockId=PauseUntilTime block="ждать $delay мс|при начале в $startTime"
    //% block.loc.ru="wait $delay ms|at start at $startTime"
    //% weight=6
    export function PauseUntilTime(startTime: number, ms: number) {
        if (startTime == 0) startTime = control.millis();
        const waitCompletionTime = startTime + ms;
        while (control.millis() < waitCompletionTime);
    }

}