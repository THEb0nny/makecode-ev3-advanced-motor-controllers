/**
 * A differential drive chassis robot.
 * Шасси робота с дифференциальным приводом.
 */
//% block="Chassis"
//% block.loc.ru="Шасси"
//% color="#02909D" weight="88" icon="\uf1b9"
namespace chassis {

    // export let motorsPair: motors.SynchedMotorPair; // Моторная пара
    export let leftMotor: motors.Motor; // Левый двигатель в шасси
    export let rightMotor: motors.Motor; // Правый двигатель в шасси

    let motorMaxRPM: number = 0; // Максимальная частота (rpm) вращения двигателя
    let wheelDiametr: number = 0; // Радиус колеса в мм
    let baseLength: number = 0; // Расстояние между колесами в мм

    let syncKp: number = 0.03; // Пропорциональный коэффициент синхронизации
    let syncKi: number = 0; // Интегральный коэффициент синхронизации
    let syncKd: number = 0.5; // Дифференциальный коэффициент синхронизации
    let syncKf: number = 0; // Фильтр дифференциального регулятора синхронизации

    let brakeSettleTime = 10; // Время для стабилизации после тормоза шасси (мсек)

    export const pidChassisSync = new automation.PIDController(); // PID для синхронизации двигателей шасси

    // Функция установки свойства удержания сразу для двух двигателей шасси
    export function setBrake(hold: Braking) {
        // motorsPair.setBrake(hold == Braking.Hold);
        leftMotor.setBrake(hold == Braking.Hold);
        rightMotor.setBrake(hold == Braking.Hold);
    }

    // Только для двух двигателей одновременно в моторной паре
    /*
    function splitDoubleOutput(out: Output): Output[] {
        if (out == Output.BC) return [Output.B, Output.C];
        else if (out == Output.AB) return [Output.A, Output.B];
        else if (out == Output.CD) return [Output.C, Output.D];
        else if (out == Output.AD) return [Output.A, Output.D];
        return [];
    }
    */
    
    // Получить выход (Output) двигателей из входной строки
    /*
    function strNameToOutput(outStr: string): Output {
        if (outStr == "B+C") return Output.BC;
        else if (outStr == "A+B") return Output.AB;
        else if (outStr == "C+D") return Output.CD;
        else if (outStr == "A+D") return Output.AD;
        return Output.ALL;
    }
    */

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
    /*
    function setChassis(newMotorsPair: motors.SynchedMotorPair, setLeftMotReverse: boolean, setRightMotReverse: boolean) {
        return;
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
        leftMotor.setRegulated(false); // Отключить регулирование скорости прошивки левого мотора
        rightMotor.setRegulated(false); // Отключить регулирование скорости прошивки правого мотора
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
    }
    */

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
        setSpeedRegulated(false); // Отключить регулирование скорости прошивки моторов
        const motorLeftType = leftMotor.toString().split(" ")[0][0];
        const motorRightType = leftMotor.toString().split(" ")[0][0];
        if (motorLeftType === "M" && motorRightType === "M") motorMaxRPM = 250;
        else motorMaxRPM = 170;
    }

    /**
     * Установить регулирование скоростей уровня прошивки для моторов шасси. Изначально выключено.
     * Если выключено, тогда мы управляем мощностью моторов.
     * @param regulated регулирование скорости, eg: false
     */
    //% blockId="ChassisSetSpeedRegulated"
    //% block="set speed regulated motors to chassis $regulated"
    //% block.loc.ru="установить регулирование скоростей моторов шасси $regulated"
    //% regulated.shadow="toggleOnOff"
    //% inlineInputMode="inline"
    //% weight="97"
    //% group="Установить"
    //% blockHidden="true"
    export function setSpeedRegulated(regulated: boolean) {
        leftMotor.setRegulated(regulated);
        rightMotor.setRegulated(regulated);
    }

    /**
     * Установить время стабилизации тормоза шасси в мсек.
     * @param settleTime время стабилизации шасси в мсек, eg: 100
     */
    //% blockId="ChassisSetBrakeSettleTime"
    //% block="set settle time after break $settleTime ms"
    //% block.loc.ru="установить время стабилизации после остановки $settleTime мсек"
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
     * Задает диаметр колеса в мм.
     * @param diametr диаметр колеса, eg: 56
     */
    //% blockId="ChassisSetWheelВiametr"
    //% block="set wheel diametr = $diametr mm"
    //% block.loc.ru="установить диаметр колёс шасси = $diametr мм"
    //% weight="95" blockGap="8"
    //% group="Колёса"
    export function setWheelDiametr(diametr: number) {
        wheelDiametr = diametr;
    }

    /**
     * Возвращает диаметр колеса.
     */
    //% blockId="ChassisGetWheelDiametr"
    //% block="get wheel diametr in mm"
    //% block.loc.ru="диаметр колёс шасси в мм"
    //% weight="94"
    //% group="Колёса"
    export function getWheelDiametr(): number {
        return wheelDiametr;
    }

    /**
     * Устанавливает длину базы (колеи) в мм.
     * @param length расстояние между центрами колёс в мм, eg: 130
     */
    //% blockId="ChassisSetBaseLength"
    //% block="set base length = $length $unit"
    //% block.loc.ru="установить размер колеи шасси = $length"
    //% weight="93" blockGap="8"
    //% group="Колея"
    export function setBaseLength(length: number) {
        baseLength = length;
    }

    /**
     * Получить длину базы (колеи) в мм.
     */
    //% blockId="ChassisGetBaseLength"
    //% block="get base length in mm"
    //% block.loc.ru="размер колеи шасси в мм"
    //% weight="92"
    //% group="Колея"
    export function getBaseLength() {
        return baseLength;
    }

    /**
     * Отключите двигатели шасси.
     * @param setBrake удерживайте двигатели при торможении, если не установить, то состояние торможение не меняется с прошлого раза, eg: Braking.Hold
     * @param settleTime время для стабилизации шасси после остановки, eg: 100
     */
    //% blockId="ChassisStop"
    //% block="chassis stop||hold $setBrake|settle time $settleTime"
    //% block.loc.ru="остановить шасси||удержание $setBrake|время стабилизации $settleTime"
    //% inlineInputMode="inline"
    //% expandableArgumentMode="toggle"
    //% weight="99"
    //% group="Move"
    export function stop(setBrake?: Braking, settleTime?: number) {
        if (!leftMotor && !rightMotor) return;
        if (settleTime == undefined) settleTime = brakeSettleTime;
        if (setBrake !== undefined) {
            chassis.setBrake(setBrake);
        }
        leftMotor.setBrakeSettleTime(0); rightMotor.setBrakeSettleTime(0); // Установить двигателям по отдельности задержку для стабилизации при остановке на 0, т.к. нам не нужно, чтобы один мотор отстаналвивался и ждал, а потом это же делал второй
        leftMotor.stop(); rightMotor.stop(); // Команда остановки моторам
        leftMotor.setBrakeSettleTime(10); rightMotor.setBrakeSettleTime(10); // Установить обратно моторам по отдельности ожидание для стабилизации
        pause(Math.max(0, settleTime)); // Пауза для стабилизации шассии
    }

    // Получить скорости моторов при входном значении рулевого параметра и скорости
    export function getSpeedsAtSteering(turnRatio: number, speed: number): { speedLeft: number, speedRight: number } {
        speed = Math.clamp(-100, 100, speed >> 0); // Ограничиваем скорость от -100 до 100 и отсекаем дробную часть
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
        leftMotor.run(speedLeft);
        rightMotor.run(speedRight);
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
    /*
    export function drive(speed: number, rotationSpeed: number, distance: number = 0, unit: MeasurementUnit = MeasurementUnit.Millimeters) {
        if (!leftMotor && !rightMotor) return;
        if (speed == 0 || wheelDiametr == 0 || baseLength == 0) {
            stop(Braking.Hold);
            return;
        }

        // Speed is expressed in %
        const D = wheelDiametr * 10; // cm
        const R = D / 2; // cm
        const L = baseLength * 10; // cm

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

        // motorsPair.tank(sr, sl, seconds, MoveUnit.Seconds);
    }
    */
    
    /**
     * Синхронизация двигателей в шасси с настройкой скоростей для каждого двигателя. Нет поддержки ускорения или замедления.
     * @param vLeft входное значение частоты вращения левого двигателя, eg: 50
     * @param vRight входное значение частоты вращения правого двигателя, eg: 50
     * @param value размерность вращения, eg: 500
     * @param unit значение единицы измерения, eg: MoveUnit.Degrees
     * @param braking тип торможения, eg: MotionBraking.Hold
     */
    //% blockId="ChassisSyncMovement"
    //% block="sync chassis movement at $vLeft\\% $vRight\\% for value = $value $unit braking $braking"
    //% block.loc.ru="синхронизированное управление шасси с $vLeft\\% $vRight\\% на $value $unit торможение $braking"
    //% inlineInputMode="inline"
    //% vLeft.shadow="motorSpeedPicker"
    //% vRight.shadow="motorSpeedPicker"
    //% weight="98" blockGap="8"
    //% group="Синхронизированное движение"
    export function syncMovement(vLeft: number, vRight: number, value: number, unit: MoveUnit = MoveUnit.Degrees, braking: MotionBraking = MotionBraking.Hold) {
        if (!leftMotor && !rightMotor) return;
        if (vLeft == 0 && vRight == 0 ||
            ((unit == MoveUnit.Rotations || unit == MoveUnit.Degrees) && value == 0) ||
            ((unit == MoveUnit.Seconds || unit == MoveUnit.MilliSeconds) && value <= 0)) {
            stop(Braking.Hold);
            return;
        }
        
        vLeft = Math.clamp(-100, 100, vLeft >> 0); // Ограничиваем скорость левого мотора от -100 до 100 и отсекаем дробную часть
        vRight = Math.clamp(-100, 100, vRight >> 0); // Ограничиваем скорость правого мотора от -100 до 100 и отсекаем дробную часть
        const emlPrev = leftMotor.angle(), emrPrev = rightMotor.angle(); // Перед запуском считываем значение с энкодеров левого и правого двигателя
        if (unit == MoveUnit.Rotations) value /= 360; // Преобразуем градусы в обороты, если выбран соответствующий режим
        const emlValue = (Math.abs(vLeft) != 0 ? value : 0); // Значение, которое должен выполнить левый двигатель, если скорость мотора не 0
        const emrValue = (Math.abs(vRight) != 0 ? value : 0); // Значение, которое должен выполнить правый двигатель, если скорость мотора не 0

        // advmotctrls.syncMotorsConfig(vLeft, vRight); // Установите скорости двигателя для последующего регулирования
        pidChassisSync.setGains(syncKp, syncKi, syncKd); // Установка коэффициентов регулятора синхронизации
        pidChassisSync.setControlSaturation(-100, 100); // Ограничение регулятора
        pidChassisSync.setPoint(0); // Установить нулевую уставку регулятору
        pidChassisSync.reset(); // Сброс ПИД-регулятора

        let prevTime = 0; // Переменная для хранения предыдущего времени для цикла регулятора
        if (unit == MoveUnit.Seconds) value *= 0.001; // Если значение было указано в сек, то перевести его в мсек
        const startTime = control.millis() * (unit == MoveUnit.Seconds ? 0.001 : 1); // Фиксируем время до начала цикла регулирования, если время было указано в секундах, тогда перевести в мсек
        const endTime = (unit == MoveUnit.MilliSeconds || unit == MoveUnit.Seconds ? startTime + value : 0); // Вычисляем время окончания цикла регулирования, если выбран соответствующий режим
        while (true) { // Цикл синхронизации движения
            let currTime = control.millis();
            let dt = currTime - prevTime;
            prevTime = currTime;
            let eml = leftMotor.angle() - emlPrev, emr = rightMotor.angle() - emrPrev; // Получить текущее значение энкодера левого и правого двигателя
            if ((unit == MoveUnit.Degrees || unit == MoveUnit.Rotations) &&
                Math.abs(eml) >= Math.abs(emlValue) && Math.abs(emr) >= Math.abs(emrValue)) break; // Условие завершения, если режим поворота на градусы или обороты
            else if ((unit == MoveUnit.MilliSeconds || unit == MoveUnit.Seconds) && 
                control.millis() >= endTime) break; // Условия завершения, если режим по времени
            // else if (unit == MoveUnit.Seconds && control.millis() * 0.001 >= endTime) break; // Условие завершения, если выбран режим в мсек
            let error = advmotctrls.getErrorSyncMotorsAtPwr(eml, emr, vLeft, vRight); // Найдите ошибку в управлении двигателей
            // pidChassisSync.setPoint(error); // Передать ошибку управления регулятору
            let U = pidChassisSync.compute(dt, -error); // Получить управляющее воздействие от регулятора
            let powers = advmotctrls.getPwrSyncMotors(U); // Узнайте мощность двигателей для регулирования, передав управляющее воздействие
            setSpeedsCommand(powers.pwrLeft, powers.pwrRight); // Установить скорости/мощности моторам
            control.pauseUntilTime(currTime, 1); // Подождите, пока цикл управления не достигнет установленного количества времени
        }
        if (braking == MotionBraking.Hold) stop(Braking.Hold); // Торможение и удержание
        else if (braking == MotionBraking.Float) stop(Braking.Float); // Торможение с освобождением (без удержания)
        else if (braking == MotionBraking.Coasting) setSpeedsCommand(vLeft, vRight); // Двигаться дальше
    }

    /**
     * Синхронизация двигателей в шасси с настройкой скоростей (мощностей) для каждого двигателя. Нет поддержки ускорения или замедления.
     * @param turnRatio рулевой параметр, если больше 0, то поворачиваем вправо, а если меньше, то влево, eg: 0
     * @param speed входное значение скорости (мощности), eg: 50
     * @param value размерность вращения, eg: 500
     * @param unit значение единицы измерения, eg: MoveUnit.Degrees
     * @param braking тип торможения, eg: MotionBraking.Hold
     */
    //% blockId="ChassisSyncSteeringMovement"
    //% block="sync chassis movement in direction $turnRatio at $speed\\% for value = $value $unit braking $braking"
    //% block.loc.ru="синхронизированное управление шасси по направлению $turnRatio с $speed\\% при $value $unit торможение $braking"
    //% inlineInputMode="inline"
    //% turnRatio.shadow="motorTurnRatioPicker"
    //% speed.shadow="motorSpeedPicker"
    //% weight="98" blockGap="8"
    //% group="Синхронизированное движение"
    export function syncSteeringMovement(turnRatio: number, speed: number, value: number, unit: MoveUnit = MoveUnit.Degrees, braking: MotionBraking = MotionBraking.Hold) {
        const { speedLeft, speedRight } = getSpeedsAtSteering(turnRatio, speed);
        syncMovement(speedLeft, speedRight, value, unit, braking);
    }

    // Функция выполнения синхронизированного движения с фазами
    export function executeRampMovement(minStartPwr: number, maxPwr: number, minEndPwr: number, accelDist: number, decelDist: number, totalDist: number) {
        // Защиту входных данных следует провести в функции, которая запускает executeRampMovement

        advmotctrls.accTwoEncConfig(minStartPwr, maxPwr, minEndPwr, accelDist, decelDist, totalDist);
        pidChassisSync.setGains(syncKp, syncKi, syncKd);
        pidChassisSync.setControlSaturation(-100, 100);
        pidChassisSync.setPoint(0); // Установить нулевую уставку регулятору
        pidChassisSync.reset();

        const emlPrev = leftMotor.angle(), emrPrev = rightMotor.angle(); // Перед запуском мы считываем значение с энкодера левого и правого двигателя
        // const { emLeft: emlPrev, emRight: emrPrev } = getEncoderValues();

        let prevTime = 0;
        while (true) {
            let currTime = control.millis();
            let dt = currTime - prevTime;
            prevTime = currTime;
            let eml = leftMotor.angle() - emlPrev, emr = rightMotor.angle() - emrPrev;
            let out = advmotctrls.accTwoEnc(eml, emr);
            if (out.isDone) break;
            let error = advmotctrls.getErrorSyncMotorsAtPwr(eml, emr, out.pwr, out.pwr);
            // pidChassisSync.setPoint(error);
            let U = pidChassisSync.compute(dt, -error);
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
        if (!leftMotor && !rightMotor) return;
        if (maxSpeed == 0 || totalValue == 0) {
            stop(Braking.Hold);
            return;
        } else if (startSpeed > maxSpeed || maxSpeed < finishSpeed) {
            stop(Braking.Hold);
            return;
        }

        executeRampMovement(startSpeed, maxSpeed, finishSpeed, accelValue, decelValue, totalValue); // Выполнение синхронизированного движения с фазами
        stop(Braking.Hold); // Остановить с удержанием
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