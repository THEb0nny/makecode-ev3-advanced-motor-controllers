/**
 * A differential drive robot.
 * Робот с дифференциальным приводом.
 */
//% block="Chassis"
//% block.loc.ru="Шасси"
//% color="#02909D" weight="88" icon="\uf1b9"
namespace chassis {

    export let motorsPair: motors.SynchedMotorPair; // Моторная пара
    export let leftMotor: motors.Motor; // Левый двигатель в шасси
    export let rightMotor: motors.Motor; // Правый двигатель в шасси

    let motorMaxRPM: number = 0; // Максимальная частота (rpm) вращения двигателя
    let wheelDiametr: number = 0; // Радиус колеса (см)
    let baseLength: number = 0; // Расстояние между колесами (см)

    let syncKp: number = 0.03; // Пропорциональный коэффициент синхронизации
    let syncKi: number = 0; // Интегральный коэффициент синхронизации
    let syncKd: number = 0.5; // Дифференциальный коэффициент синхронизации
    let syncKf: number = 0; // Фильтр дифференциального регулятора синхронизации

    let brakeSettleTime = 10; // Время срабатывания тормоза шасси (мс)

    export const pidChassisSync = new automation.PIDController(); // PID для синхронизации двигателей шасси

    // Функция установки свойства удержания сразу для двух двигателей шасси
    export function setBrake(hold: boolean) {
        // motorsPair.setBrake(hold);
        leftMotor.setBrake(hold);
        rightMotor.setBrake(hold);
    }

    // Только для двух двигателей одновременно в моторной паре
    function splitDoubleOutput(out: Output): Output[] {
        if (out == Output.BC) return [Output.B, Output.C];
        else if (out == Output.AB) return [Output.A, Output.B];
        else if (out == Output.CD) return [Output.C, Output.D];
        else if (out == Output.AD) return [Output.A, Output.D];
        return [];
    }
    
    // Получить выход (Output) двигателей из входной строки
    function strNameToOutput(outStr: string): Output {
        if (outStr == "B+C") return Output.BC;
        else if (outStr == "A+B") return Output.AB;
        else if (outStr == "C+D") return Output.CD;
        else if (outStr == "A+D") return Output.AD;
        return Output.ALL;
    }

    /**
     * Устанавливает двигатели, используемые шасси. При необходимости вы можете сразу же установить реверс моторам.
     * @param newMotorsPair пара двигателей, eg: motors.largeBC
     * @param setLeftMotReverse свойство реверса левого двигателя, eg: false
     * @param setRightMotReverse свойство реверса правого двигателя, eg: false
     */
    //% blockId="ChassisSetChassis"
    //% block="set motors to chassis $newMotorsPair||at reverse $setLeftMotReverse $setRightMotReverse"
    //% block.loc.ru="установить моторы шасси $newMotorsPair||с реверсом $setLeftMotReverse $setRightMotReverse"
    //% newMotorsPair.fieldEditor="motors"
    //% newMotorsPair.fieldOptions.decompileLiterals="1"
    //% setLeftMotReverse.shadow="toggleOnOff"
    //% setRightMotReverse.shadow="toggleOnOff"
    //% inlineInputMode="inline"
    //% expandableArgumentMode="toggle"
    //% weight="99"
    //% subcategory="Свойства"
    //% group="Установить"
    //% blockHidden="true"
    function setChassis(newMotorsPair: motors.SynchedMotorPair, setLeftMotReverse?: boolean, setRightMotReverse?: boolean) {
        return;
        /*
        motorsPair = newMotorsPair;
        const motorsName = motorsPair.toString();
        const motorsType = motorsName.split(" ")[0];
        const motorsPort = motorsName.split(" ")[1];
        const motorsPortArr = motorsName.split(" ")[1].split("+");
        const allUsedSingleMotors = motors.Motor.getAllInstances(); // Get all motors instances
        // allUsedSingleMotors.forEach((motor) => {
        //     console.log(`motor: ${motor}`);
        // });
        if (allUsedSingleMotors.length >= 1) { // Ищем из существующих моторов
            leftMotor = allUsedSingleMotors.filter((motor) => motor.toString().split(" ")[1] == motorsPortArr[0])[0]; // Set left motor instance
            rightMotor = allUsedSingleMotors.filter((motor) => motor.toString().split(" ")[1] == motorsPortArr[1])[0]; // Set right motor instance
            // console.log(`leftMotor1: ${leftMotor}, rightMotor1: ${rightMotor}`);
        }
        if (!leftMotor || !rightMotor) { // Если моторы не были найдены, тогда уже создать свои классы
            const motorsOut = motors.splitDoubleOutput(strNameToOutput(motorsPort));
            // console.log(`motorsName: ${motorsName}, motorsName: ${motorsPort[0]}, motorsOut: ${motorsOut[0]}, ${motorsOut[1]}`);
            // motorsOut.forEach((port) => {
            //     console.log(`motorsOut: ${port}`);
            // });
            const isLargeMotor = (motorsType == "large" ? true : false);
            if (!leftMotor) {
                leftMotor = new motors.Motor(motorsOut[0], isLargeMotor);
                //console.log(`new leftMotor2: ${leftMotor}`);
            }
            if (!rightMotor) {
                rightMotor = new motors.Motor(motorsOut[1], isLargeMotor);
                //console.log(`new rightMotor2: ${rightMotor}`);
            }
        }
        //console.log(`reverse ${setLeftMotReverse}, ${setRightMotReverse}`);
        if (setLeftMotReverse != undefined) {
            leftMotor.setInverted(setLeftMotReverse);
            //console.log(`reverse leftMotor: ${setLeftMotReverse}`);
        }
        if (setRightMotReverse != undefined) {
            rightMotor.setInverted(setRightMotReverse);
            //console.log(`reverse rightMotor: ${setRightMotReverse}`);
        }
        if (motorsType == "large") motorMaxRPM = 170;
        else if (motorsType == "medium") motorMaxRPM = 250;
        */
    }

    /**
     * Устанавливает двигатели, используемые шасси. При необходимости вы можете сразу же установить реверс моторам.
     * @param newLeftMotors левый двигатель в шасси, eg: motors.largeB
     * @param newRightMotors правый двигатель в шасси, eg: motors.largeC
     * @param setLeftMotReverse свойство реверса левого двигателя, eg: false
     * @param setRightMotReverse свойство реверса правого двигателя, eg: false
     */
    //% blockId="ChassisSetChassisMotors"
    //% block="set motors to chassis $newLeftMotors $newRightMotors|at reverse $setLeftMotReverse $setRightMotReverse"
    //% block.loc.ru="установить моторы шасси $newLeftMotors $newRightMotors|с реверсом $setLeftMotReverse $setRightMotReverse"
    //% newLeftMotors.fieldEditor="motors"
    //% newLeftMotors.fieldOptions.decompileLiterals="1"
    //% newRightMotors.fieldEditor="motors"
    //% newRightMotors.fieldOptions.decompileLiterals="1"
    //% setLeftMotReverse.shadow="toggleOnOff"
    //% setRightMotReverse.shadow="toggleOnOff"
    //% inlineInputMode="inline"
    //% expandableArgumentMode="disabled"
    //% weight="98"
    //% group="Установить"
    export function setChassisMotors(newLeftMotors: motors.Motor, newRightMotors: motors.Motor, setLeftMotReverse: boolean, setRightMotReverse: boolean) {
        if (newLeftMotors == newRightMotors) {
            console.log("Error: the same motor is specified for the left and right motors!");
            control.assert(false, 99);
        }
        leftMotor = newLeftMotors; // Установите левый экземпляр двигателя
        rightMotor = newRightMotors; // Установите правильный экземпляр двигателя
        leftMotor.setInverted(setLeftMotReverse); // Установите свойство реверса левого двигателя
        rightMotor.setInverted(setRightMotReverse); // Установите правильное свойство реверса двигателя
        leftMotor.setRegulated(false); // Отключить регулирование скорости прошивки левого мотора
        rightMotor.setRegulated(false); // Отключить регулирование скорости прошивки правого мотора
        const motorLeftType = leftMotor.toString().split(" ")[0][0];
        const motorRightType = leftMotor.toString().split(" ")[0][0];
        if (motorLeftType === "M" && motorRightType === "M") motorMaxRPM = 250;
        else motorMaxRPM = 170;
    }

    /**
     * Установить регулирование скоростей для моторов шасси.
     * @param regulated регулирование скорости, eg: false
     */
    //% blockId="ChassisSetSpeedRegulated"
    //% block="set speed regulated motors to chassis $regulated"
    //% block.loc.ru="установить регулирование скоростей моторов шасси $regulated"
    //% regulated.shadow="toggleOnOff"
    //% inlineInputMode="inline"
    //% weight="97"
    //% group="Установить"
    export function setSpeedRegulated(regulated: boolean) {
        leftMotor.setRegulated(regulated);
        rightMotor.setRegulated(regulated);
    }

    /**
     * Установить время стабилизации тормоза шасси.
     * @param settleTime время стабилизации шасси в мсек, eg: 100
     */
    //% blockId="ChassisSetBrakeSettleTime"
    //% block="set settle time after break $settleTime"
    //% block.loc.ru="установить время стабилизации после остановки $settleTime"
    //% inlineInputMode="inline"
    //% weight="96"
    //% group="Установить"
    export function setBrakeSettleTime(settleTime: number) {
        brakeSettleTime = Math.max(0, settleTime);
    }

    /**
     * Установите управляющие значения синхронизации шасси.
     * @param Kp значение синхронизации Kp, eg: 0.03
     * @param Ki значение синхронизации Ki, eg: 0
     * @param Kd значение синхронизации Kd, eg: 0.5
     * @param Kf значение фильтра дифференциального регулятора синхронизации, eg: 0
    */
    //% blockId="ChassisSetSyncRegulatorGains"
    //% block="set chassis sync pid gains Kp = $Kp Ki = $Ki Kd = $Kd||Kf = $Kf"
    //% block.loc.ru="установить коэффиценты синхронизации шасси Kp = $Kp Ki = $Ki Kd = $Kd||Kf = $Kf"
    //% inlineInputMode="inline"
    //% expandableArgumentMode="toggle"
    //% weight="95"
    //% group="Установить"
    export function setSyncRegulatorGains(Kp: number, Ki: number, Kd: number, Kf?: number) {
        syncKp = Kp;
        syncKi = Ki;
        syncKd = Kd;
        if (Kf) syncKf = Kf;
    }

    export function getSyncRegulatorKp(): number {
        return syncKp;
    }

    export function getSyncRegulatorKi(): number {
        return syncKi;
    }

    export function getSyncRegulatorKd(): number {
        return syncKd;
    }

    export function getSyncRegulatorKf(): number {
        return syncKf;
    }

    /**
     * Задает диаметр колеса.
     * @param diametr диаметр колеса, eg: 56
     * @param unit размер единицы измерения диаметра, eg: MeasurementUnit.Millimeters
     */
    //% blockId="ChassisSetWheelВiametr"
    //% block="set wheel diametr = $diametr $unit"
    //% block.loc.ru="установить диаметр колёс шасси = $diametr $unit"
    //% weight="95" blockGap="8"
    //% group="Колёса"
    export function setWheelDiametr(diametr: number, unit: MeasurementUnit = MeasurementUnit.Millimeters) {
        if (unit == MeasurementUnit.Centimeters) wheelDiametr = diametr;
        else if (unit == MeasurementUnit.Millimeters) wheelDiametr = diametr / 10;
        else return;
    }

    /**
     * Возвращает диаметр колеса.
     * @param unit размерность единицы измерения длины, eg: MeasurementUnit.Millimeters
     */
    //% blockId="ChassisGetWheelDiametr"
    //% block="get wheel diametr $unit"
    //% block.loc.ru="диаметр колёс шасси в $unit"
    //% weight="94"
    //% group="Колёса"
    export function getWheelDiametr(unit: MeasurementUnit = MeasurementUnit.Millimeters): number {
        if (unit == MeasurementUnit.Centimeters) return wheelDiametr;
        else if (unit == MeasurementUnit.Millimeters) return wheelDiametr * 10;
        return 0;
    }

    /**
     * Устанавливает длину базы (колеи).
     * @param length расстояние между центрами колёс в мм, eg: 130
     * @param unit размерность единицы измерения расстояния, eg: MeasurementUnit.Millimeters
     */
    //% blockId="ChassisSetBaseLength"
    //% block="set base length = $length $unit"
    //% block.loc.ru="установить размер колеи шасси = $length $unit"
    //% weight="93" blockGap="8"
    //% group="Колея"
    export function setBaseLength(length: number, unit: MeasurementUnit = MeasurementUnit.Millimeters) {
        if (unit == MeasurementUnit.Centimeters) baseLength = length;
        else if (unit == MeasurementUnit.Millimeters) baseLength = length / 10;
        return;
    }

    /**
     * Получить длину базы (колеи).
     * @param unit размерность единицы измерения длины, eg: MeasurementUnit.Millimeters
     */
    //% blockId="ChassisGetBaseLength"
    //% block="get base length $unit"
    //% block.loc.ru="размер колеи шасси в $unit"
    //% weight="92"
    //% group="Колея"
    export function getBaseLength(unit: MeasurementUnit = MeasurementUnit.Millimeters) {
        if (unit == MeasurementUnit.Centimeters) return baseLength;
        else if (unit == MeasurementUnit.Millimeters) return baseLength * 10;
        return 0;
    }

    /**
     * Отключите двигатели шасси.
     * @param setBrake удерживайте двигатели при торможении, если не установить, то состояние торможение не меняется с прошлого раза, eg: true
     * @param settleTime время для стабилизации шасси после остановки, eg: 100
     */
    //% blockId="ChassisStop"
    //% block="chassis stop||hold $setBrake|settle time $settleTime"
    //% block.loc.ru="остановить шасси||удержание $setBrake|время стабилизации $settleTime"
    //% inlineInputMode="inline"
    //% expandableArgumentMode="toggle"
    //% setBrake.shadow="toggleOnOff"
    //% weight="99"
    //% group="Move"
    export function stop(setBrake?: boolean, settleTime?: number) {
        //if (!motorsPair) return;
        if (!settleTime) settleTime = brakeSettleTime;
        if (setBrake) {
            // motorsPair.setBrake(setBrake);
            leftMotor.setBrake(setBrake);
            rightMotor.setBrake(setBrake);
        }
        // motorsPair.setBrakeSettleTime(0);
        // motorsPair.stop();
        leftMotor.setBrakeSettleTime(0); rightMotor.setBrakeSettleTime(0); // Set the motors separately to wait for stabilization when stopping at 0
        leftMotor.stop(); rightMotor.stop(); // Motors stop command
        leftMotor.setBrakeSettleTime(10); rightMotor.setBrakeSettleTime(10); // Return the motors separately to waiting for stabilization when stopping at 10
        pause(Math.max(0, settleTime)); // Settle chassis delay
    }

    // Получить скорости моторов от рулевого параметра и скорости
    export function getSpeedsAtSteering(turnRatio: number, speed: number): { speedLeft: number, speedRight: number } {
        speed = Math.clamp(-100, 100, speed >> 0);
        turnRatio = Math.floor(turnRatio);
        turnRatio = Math.clamp(-200, 200, turnRatio >> 0);
        let speedLeft = 0, speedRight = 0;
        if (turnRatio > 0) { // Вправо
            if (turnRatio <= 100) {
                speedLeft = speed;
                speedRight = (100 - turnRatio) * speed / 100;
            } else if (turnRatio > 100) { // Более 100
                speedLeft = speed;
                //speedRight = Math.max(-speed, -(turnRatio - 100) * (speed / 100));
                speedRight = -(turnRatio - 100) * (speed / 100);
            }
        } else if (turnRatio < 0) { // Влево
            if (turnRatio >= -100) { // До -100 включительно
                speedLeft = (100 + turnRatio) * speed / 100;
                speedRight = speed;
            } else if (turnRatio < -100) { // Более -100
                //speedLeft = Math.max(-speed, (turnRatio + 100) * (speed / 100));
                speedLeft = (turnRatio + 100) * (speed / 100);
                speedRight = speed;
            }
        } else { // Если turnRatio = 0
            speedLeft = speed;
            speedRight = speed;
        }
        return { speedLeft, speedRight };
    }

    /**
     * Команда рулевого управления моторами шасси.
     * @param turnRatio рулевой параметр, если больше 0, то поворачиваем вправо, а если меньше, то влево, eg: 0
     * @param speed скорость (мощность) движения, eg: 50
     */
    //% blockId="ChassisSteeringCommand"
    //% block="steering command in direction $turnRatio at $speed\\%"
    //% block.loc.ru="рулевое управление по направлению $turnRatio на $speed\\%"
    //% inlineInputMode="inline"
    //% turnRatio.shadow="motorTurnRatioPicker"
    //% speed.shadow="motorSpeedPicker"
    //% weight="98"
    //% group="Move"
    export function steeringCommand(turnRatio: number, speed: number) {
        const { speedLeft, speedRight } = getSpeedsAtSteering(turnRatio, speed);
        setSpeedsCommand(speedLeft, speedRight);
    }

    // Команда установки моторам скоростей (мощностей)
    export function setSpeedsCommand(speedLeft: number, speedRight: number) {
        leftMotor.run(speedLeft); rightMotor.run(speedRight);
    }

    /**
     * Заставляет робота с дифференциальным приводом двигаться с заданной скоростью (см/с) и частотой вращения (град/с), используя модель одноколесного велосипеда.
     * @param speed скорость (мощность) центральной точки между двигателями, eg: 10
     * @param rotationSpeed вращение робота вокруг центральной точки, eg: 30
     * @param distance расстояние до места проезда, eg: 150
     * @param unit размерность единицы перемещения, eg: MeasurementUnit.Millimeters
     */
    //% blockId="ChassisDrive"
    //% block="drive at $speed cm/s turning $rotationSpeed deg/s for $distance $unit"
    //% block.loc.ru="движение $speed см/с поворотом $rotationSpeed град/с на дистанцию $distance $unit"
    //% inlineInputMode="inline"
    //% weight="99" blockGap="8"
    //% rotationSpeed.min="-3200" rotationSpeed.max="3200"
    //% group="Move"
    //% blockHidden="true"
    export function drive(speed: number, rotationSpeed: number, distance: number = 0, unit: MeasurementUnit = MeasurementUnit.Millimeters) {
        // if (!motorsPair) return;
        if (!speed || wheelDiametr == 0 || baseLength == 0) {
            stop(true);
            return;
        }

        // Speed is expressed in %
        const D = wheelDiametr; // cm
        const R = D / 2; // cm
        const L = baseLength; // cm

        const maxw = motorMaxRPM / 60 * 2 * Math.PI; // rad/s
        const maxv = maxw * R; // cm/s
        const v = speed; // speed is cm/s
        const w = rotationSpeed / 360 * 2 * Math.PI; // rad/s

        const vr = (2 * v + w * L) / (2 * R); // rad/s
        const vl = (2 * v - w * L) / (2 * R); // rad/seconds

        const sr = vr / maxw * 100; // %
        const sl = vl / maxw * 100; // %

        if (distance != 0 && unit == MeasurementUnit.Millimeters) distance / 10; // mm to cm
        const seconds = distance / speed; // cm / (cm/s) = s

        motorsPair.tank(sr, sl, seconds, MoveUnit.Seconds);
    }
    
    /**
     * Синхронизация двигателей в шасси с настройкой скоростей для каждого двигателя. Нет поддержки ускорения или замедления.
     * @param vLeft входное значение частоты вращения левого двигателя, eg: 50
     * @param vRight входное значение частоты вращения правого двигателя, eg: 50
     * @param value размерность вращения, eg: 500
     * @param unit значение единицы измерения, eg: MoveUnit.Degrees
     * @param braking тип торможения, eg: Braking.Hold
     */
    //% blockId="ChassisSyncMovement"
    //% block="sync chassis movement at $vLeft\\% $vRight\\% for value = $value $unit braking $braking"
    //% block.loc.ru="синхронизированное управление шасси с $vLeft\\% $vRight\\% на $value $unit торможение $braking"
    //% inlineInputMode="inline"
    //% vLeft.shadow="motorSpeedPicker"
    //% vRight.shadow="motorSpeedPicker"
    //% weight="98" blockGap="8"
    //% group="Синхронизированное движение"
    export function syncMovement(vLeft: number, vRight: number, value: number, unit: MoveUnit = MoveUnit.Degrees, braking: Braking = Braking.Hold) {
        // if (!motorsPair) return;
        if (vLeft == 0 && vRight == 0 ||
            ((unit == MoveUnit.Rotations || unit == MoveUnit.Degrees) && value == 0) ||
            ((unit == MoveUnit.Seconds || unit == MoveUnit.MilliSeconds) && value <= 0)) {
            stop(true);
            return;
        }
        vLeft = Math.clamp(-100, 100, vLeft >> 0); // We limit the speed of the left motor from -100 to 100 and cut off the fractional part
        vRight = Math.clamp(-100, 100, vRight >> 0); // We limit the speed of the right motor from -100 to 100 and cut off the fractional part
        const emlPrev = leftMotor.angle(), emrPrev = rightMotor.angle(); // We read the value from the encoder from the left and right motor before starting
        if (unit == MoveUnit.Rotations) value /= 360; // Convert degrees to revolutions if the appropriate mode is selected
        const emlValue = (Math.abs(vLeft) != 0 ? value : 0); // The value that the left and right motor must pass
        const emrValue = (Math.abs(vRight) != 0 ? value : 0); // The value that the right motor must pass

        advmotctrls.syncMotorsConfig(vLeft, vRight); // Set motor speeds for subsequent regulation
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
            let eml = leftMotor.angle() - emlPrev, emr = rightMotor.angle() - emrPrev; // Get left motor and right motor encoder current value
            if ((unit == MoveUnit.Degrees || unit == MoveUnit.Rotations) &&
                Math.abs(eml) >= Math.abs(emlValue) && Math.abs(emr) >= Math.abs(emrValue)) break;
            else if (unit == MoveUnit.MilliSeconds && control.millis() >= endTime) break;
            else if (unit == MoveUnit.Seconds && control.millis() * 0.001 >= endTime) break;
            let error = advmotctrls.getErrorSyncMotors(eml, emr); // Find out the error in motor speed control
            pidChassisSync.setPoint(error); // Transfer control error to controller
            let U = pidChassisSync.compute(dt, 0); // Find out and record the control action of the regulator
            let powers = advmotctrls.getPwrSyncMotors(U); // Find out the power of motors for regulation
            setSpeedsCommand(powers.pwrLeft, powers.pwrRight); // Set power/speed motors
            control.pauseUntilTime(currTime, 1); // Wait until the control cycle reaches the set amount of time passed
        }
        if (braking == Braking.Hold) stop(true); // Break at hold
        else if (braking == Braking.NoBreak) stop(false); // No hold break
        else setSpeedsCommand(vLeft, vRight); // Forward
    }

    /**
     * Синхронизация двигателей в шасси с настройкой скоростей (мощностей) для каждого двигателя. Нет поддержки ускорения или замедления.
     * @param turnRatio рулевой параметр, если больше 0, то поворачиваем вправо, а если меньше, то влево, eg: 0
     * @param speed входное значение скорости (мощности), eg: 50
     * @param value размерность вращения, eg: 500
     * @param unit значение единицы измерения, eg: MoveUnit.Degrees
     * @param braking тип торможения, eg: Braking.Hold
     */
    //% blockId="ChassisSyncSteeringMovement"
    //% block="sync chassis movement in direction $turnRatio at $speed\\% for value = $value $unit braking $braking"
    //% block.loc.ru="синхронизированное управление шасси по направлению $turnRatio с $speed\\% при $value $unit торможение $braking"
    //% inlineInputMode="inline"
    //% turnRatio.shadow="motorTurnRatioPicker"
    //% speed.shadow="motorSpeedPicker"
    //% weight="98" blockGap="8"
    //% group="Синхронизированное движение"
    export function syncSteeringMovement(turnRatio: number, speed: number, value: number, unit: MoveUnit = MoveUnit.Degrees, braking: Braking = Braking.Hold) {
        const { speedLeft, speedRight } = getSpeedsAtSteering(turnRatio, speed);
        syncMovement(speedLeft, speedRight, value, unit, braking);
    }

    // Функция выполнения синхронизированного движения с фазами
    export function executeRampMovement(minStartPwr: number, maxPwr: number, minEndPwr: number, accelDist: number, decelDist: number, totalDist: number) {
        advmotctrls.accTwoEncConfig(minStartPwr, maxPwr, minEndPwr, accelDist, decelDist, totalDist);
        pidChassisSync.setGains(syncKp, syncKi, syncKd);
        pidChassisSync.setControlSaturation(-100, 100);
        pidChassisSync.reset();

        const emlPrev = leftMotor.angle(), emrPrev = rightMotor.angle(); // We read the value from the encoder from the left motor and right motor before starting

        let prevTime = 0;
        while (true) {
            let currTime = control.millis();
            let dt = currTime - prevTime;
            prevTime = currTime;
            let eml = leftMotor.angle() - emlPrev, emr = rightMotor.angle() - emrPrev;
            let out = advmotctrls.accTwoEnc(eml, emr);
            if (out.isDone) break;
            let error = advmotctrls.getErrorSyncMotorsAtPwr(eml, emr, out.pwr, out.pwr);
            pidChassisSync.setPoint(error);
            let U = pidChassisSync.compute(dt, 0);
            let powers = advmotctrls.getPwrSyncMotorsAtPwr(U, out.pwr, out.pwr);
            setSpeedsCommand(powers.pwrLeft, powers.pwrRight);
            control.pauseUntilTime(currTime, 1);
        }
    }

    /**
     * Синхронизация с плавным ускорением и замедлением при прямолинейном движении. Значения расстояний устанавливается в тиках энкодера.
     * @param startSpeed начальная скорость (мощность) двигателя, eg: 20
     * @param maxSpeed максимальная скорость (мощность) двигателя, eg: 50
     * @param finishSpeed конечная скорость (мощность) двигателя, eg: 10
     * @param totalValue значение общей длины для энкодера, eg: 300
     * @param accelValue значение длины ускорения для энкодера, eg: 50
     * @param decelValue значение длины замедления для энкодера, eg: 100
     */
    //% blockId="ChassisSyncRampMovement"
    //% block="sync chassis ramp movement at start $startSpeed\\% max $maxSpeed\\% finish $finishSpeed\\% for distance $totalValue acceleration $accelValue deceleration $decelValue"
    //% block.loc.ru="синхронизированное управление шасси с ускорением на старте $startSpeed\\% макс $maxSpeed\\% финише $finishSpeed\\% на расстояние $totalValue ускорения $accelValue замедления $decelValue"
    //% inlineInputMode="inline"
    //% startSpeed.shadow="motorSpeedPicker"
    //% maxSpeed.shadow="motorSpeedPicker"
    //% finishSpeed.shadow="motorSpeedPicker"
    //% weight="99"
    //% group="Синхронизированное движение с ускорениями"
    export function syncRampMovement(startSpeed: number, maxSpeed: number, finishSpeed: number, totalValue: number, accelValue: number, decelValue: number) {
        //if (!motorsPair) return;
        if (maxSpeed == 0 || totalValue == 0) {
            stop(true);
            return;
        }

        executeRampMovement(startSpeed, maxSpeed, finishSpeed, accelValue, decelValue, totalValue); // Выполнение синхронизированного движения с фазами
        stop(true); // Break at hold
    }

    /*
    export function syncRampArcMovement(minSpeedLeft: number, maxSpeedLeft: number, minSpeedRight: number, maxSpeedRight: number, totalValue: number, accelValue: number, decelValue: number) {
        if (maxSpeedLeft == 0 && maxSpeedRight == 0 || totalValue == 0) {
            stop(true);
            return;
        }

        const emlPrev = leftMotor.angle(), emrPrev = rightMotor.angle();

        advmotctrls.accTwoEncConfig(minSpeedLeft, maxSpeedLeft, minSpeedLeft, minSpeedRight, maxSpeedRight, minSpeedRight, accelValue, decelValue, totalValue);

        let prevTime = control.millis();
        while (true) {
            let currTime = control.millis();
            let dt = currTime - prevTime;
            prevTime = currTime;

            let eml = leftMotor.angle() - emlPrev;
            let emr = rightMotor.angle() - emrPrev;

            let powers = advmotctrls.accTwoEnc(eml, emr);
            if (powers.isDone) break;

            setSpeedsCommand(powers.pwrLeft, powers.pwrRight);
            control.pauseUntilTime(currTime, 1);
        }
        stop(true);
    }
    */

}