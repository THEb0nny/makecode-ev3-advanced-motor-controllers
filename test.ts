// Rectilinear movement with synchronization
function StraightlineMovementExample(speed: number) {
    advmotctrls.SyncMotorsConfig(speed, speed);

    chassis.pidChassisSync.setGains(0.03, 0, 0.5); // Установка значений регулятору
    chassis.pidChassisSync.setControlSaturation(-100, 100); // Ограничения ПИДа
    chassis.pidChassisSync.reset(); // Сброс ПИДа

    let prevTime = 0;
    while (true) {
        let currTime = control.millis();
        let dt = currTime - prevTime;
        prevTime = currTime;

        let encB = chassis.leftMotor.angle();
        let encC = chassis.rightMotor.angle();
        if ((encB + encC) / 2 >= 600) break;

        let error = advmotctrls.GetErrorSyncMotors(encB, encC);
        chassis.pidChassisSync.setPoint(error);
        let U = chassis.pidChassisSync.compute(dt, 0);
        let powers = advmotctrls.GetPwrSyncMotors(U);
        chassis.leftMotor.run(powers.pwrLeft);
        chassis.rightMotor.run(powers.pwrRight);
        control.PauseUntilTime(currTime, 5);
    }
    chassis.ChassisStop(true);
}

// Arc synchronized movement
function ArcMovementExample(lMotPwr: number, rMotPwr: number) {
    advmotctrls.SyncMotorsConfig(lMotPwr, rMotPwr);

    chassis.pidChassisSync.setGains(0.03, 0, 0.5); // Установка значений регулятору
    chassis.pidChassisSync.setControlSaturation(-100, 100); // Ограничения ПИДа
    chassis.pidChassisSync.reset(); // Сброс ПИДа

    let prevTime = 0;
    while (true) {
        let currTime = control.millis();
        let dt = currTime - prevTime;
        prevTime = currTime;

        let encB = chassis.leftMotor.angle();
        let encC = chassis.rightMotor.angle();
        if ((encB + encC) / 2 >= 775) break;

        let error = advmotctrls.GetErrorSyncMotors(encB, encC);
        chassis.pidChassisSync.setPoint(error);
        let U = chassis.pidChassisSync.compute(dt, 0);
        let powers = advmotctrls.GetPwrSyncMotors(U);
        chassis.leftMotor.run(powers.pwrLeft);
        chassis.rightMotor.run(powers.pwrRight);
        control.PauseUntilTime(currTime, 5);
    }
    chassis.ChassisStop(true);
}

// Synchronization with smooth acceleration and deceleration during straight-line motion
function SyncAccelStraightlineMovementExample() {
    advmotctrls.AccTwoEncConfig(15, 90, 100, 300, 1000);

    chassis.pidChassisSync.setGains(0.03, 0, 0.5); // Установка значений регулятору
    chassis.pidChassisSync.setControlSaturation(-100, 100); // Ограничения ПИДа
    chassis.pidChassisSync.reset(); // Сброс ПИДа
    
    let prevTime = 0;
    while (true) {
        let currTime = control.millis();
        let dt = currTime - prevTime;
        prevTime = currTime;

        let eml = chassis.leftMotor.angle();
        let emr = chassis.rightMotor.angle();
        let out = advmotctrls.AccTwoEnc(eml, emr);
        if (out.isDone) break;

        let error = advmotctrls.GetErrorSyncMotorsInPwr(eml, emr, out.pwrOut, out.pwrOut);
        chassis.pidChassisSync.setPoint(error);
        let U = chassis.pidChassisSync.compute(dt, 0);
        let powers = advmotctrls.GetPwrSyncMotorsInPwr(U, out.pwrOut, out.pwrOut);
        chassis.leftMotor.run(powers.pwrLeft);
        chassis.rightMotor.run(powers.pwrRight);

        control.PauseUntilTime(currTime, 5);
    }
    chassis.ChassisStop(true);
}

const B_REF_RAW_CS2 = 636;
const W_REF_RAW_CS2 = 490;
const B_REF_RAW_CS3 = 665;
const W_REF_RAW_CS3 = 501;

function LineFollowExample(speed: number) {
    advmotctrls.SyncMotorsConfig(speed, speed);
    automation.pid1.setGains(0.8, 0, 0.5); // Установка значений регулятору
    automation.pid1.setControlSaturation(-100, 100); // Ограничения ПИДа
    automation.pid1.reset(); // Сброс ПИДа

    let prevTime = 0;
    while (true) {
        let currTime = control.millis();
        let dt = currTime - prevTime;
        prevTime = currTime;

        let rrcs2 = sensors.color2.light(LightIntensityMode.ReflectedRaw);
        let rrcs3 = sensors.color3.light(LightIntensityMode.ReflectedRaw);
        let rcs2 = GetNormRefValCS(rrcs2, B_REF_RAW_CS2, W_REF_RAW_CS2);
        let rcs3 = GetNormRefValCS(rrcs3, B_REF_RAW_CS3, W_REF_RAW_CS3);

        let eml = chassis.leftMotor.angle();
        let emr = chassis.rightMotor.angle();

        //let sync_error = advmotctrls.GetErrorSyncMotors(eml, emr);
        let error = rcs2 - rcs3;
        automation.pid1.setPoint(error);
        let U = automation.pid1.compute(dt, 0);
        let powers = advmotctrls.GetPwrSyncMotors(U);
        // let pwrLeft = out.pwrOut + U;
        // let pwrRight = out.pwrOut - U;
        chassis.leftMotor.run(powers.pwrLeft);
        chassis.rightMotor.run(powers.pwrRight);

        control.PauseUntilTime(currTime, 10);
    }
    chassis.ChassisStop(true);
}

// Smooth acceleration and deceleration when moving along the line
function AccelLineFollowExample() {
    advmotctrls.AccTwoEncConfig(15, 70, 200, 300, 4000);
    automation.pid1.setGains(0.8, 0, 0.5); // Установка значений регулятору
    automation.pid1.setControlSaturation(-100, 100); // Ограничения ПИДа
    automation.pid1.reset(); // Сброс ПИДа

    let prevTime = 0;
    while (true) {
        let currTime = control.millis();
        let dt = currTime - prevTime;
        prevTime = currTime;

        let eml = chassis.leftMotor.angle();
        let emr = chassis.rightMotor.angle();
        let out = advmotctrls.AccTwoEnc(eml, emr);
        if (out.isDone) break;

        let rrcs2 = sensors.color2.light(LightIntensityMode.ReflectedRaw);
        let rrcs3 = sensors.color3.light(LightIntensityMode.ReflectedRaw);
        let rcs2 = GetNormRefValCS(rrcs2, B_REF_RAW_CS2, W_REF_RAW_CS2);
        let rcs3 = GetNormRefValCS(rrcs3, B_REF_RAW_CS3, W_REF_RAW_CS3);

        let error = rcs2 - rcs3;
        automation.pid1.setPoint(error);
        let U = automation.pid1.compute(dt, 0);
        let pwrLeft = out.pwrOut + U;
        let pwrRight = out.pwrOut - U;
        chassis.leftMotor.run(pwrLeft);
        chassis.rightMotor.run(pwrRight);
        
        control.PauseUntilTime(currTime, 10);
    }
    chassis.ChassisStop(true);
}

// Синхроннизированный поворот на двух средних моторах на нужный угол
function SpinTurnExample(deg: number, speed: number) {
    if (deg == 0 || speed <= 0) return;

    let emlPrev = chassis.leftMotor.angle(), emrPrev = chassis.rightMotor.angle(); // Считываем с моторов значения с энкодеров перед стартом алгаритма
    let calcMotRot = Math.round((deg * chassis.getBaseLength()) / chassis.getWheelRadius()); // Расчёт угла поворота моторов для поворота
    //let totalLeftMotRot = emlPrev + calcMotRot, totalRightMotRot = emrPrev + calcMotRot * -1; // Расчитываем итоговое значение углов на каждый мотор

    if (deg > 0) advmotctrls.SyncMotorsConfig(speed, -speed);
    else if (deg < 0) advmotctrls.SyncMotorsConfig(-speed, speed);

    chassis.pidChassisSync.setGains(0.03, 0, 0.5); // Установка значений регулятору
    chassis.pidChassisSync.setControlSaturation(-100, 100); // Ограничения ПИДа
    chassis.pidChassisSync.reset(); // Сброс ПИДа

    let prevTime = 0;
    while (true) {
        let currTime = control.millis();
        let dt = currTime - prevTime;
        prevTime = currTime;

        let eml = chassis.leftMotor.angle();
        let emr = chassis.rightMotor.angle();

        if ((Math.abs(eml - emlPrev) + Math.abs(emr - emrPrev)) / 2 >= Math.abs(calcMotRot)) break;

        let error = advmotctrls.GetErrorSyncMotors(eml, emr);
        chassis.pidChassisSync.setPoint(error);
        let U = chassis.pidChassisSync.compute(dt, 0);
        let powers = advmotctrls.GetPwrSyncMotors(U);
        chassis.leftMotor.run(powers.pwrLeft);
        chassis.rightMotor.run(powers.pwrRight);

        control.PauseUntilTime(currTime, 5);
    }
    chassis.ChassisStop(true);
}

// Перечисление о типах относительных поворотов
enum WheelPivot {
    //% block="левого колеса"
    LeftWheel,
    //% block="правого колеса"
    RightWheel
}

// Синхроннизированный поворот на двух средних моторах на нужный угол относительно одного из колёс
function PivotTurnExample(deg: number, speed: number, wheelPivot: WheelPivot) {
    if (deg == 0 || speed == 0 || deg > 0 && speed < 0 || deg < 0 && speed > 0) return;

    let emPrev = 0; // Считываем с моторов значения с энкодеров перед стартом алгаритма
    if (wheelPivot == WheelPivot.LeftWheel) emPrev = chassis.rightMotor.angle();
    else if (wheelPivot == WheelPivot.RightWheel) emPrev = chassis.leftMotor.angle();
    let calcMotRot = Math.round(((deg * chassis.getBaseLength()) / chassis.getWheelRadius()) * 2); // Расчёт угла поворота моторов для поворота
    let totalMotRot = emPrev + calcMotRot; // Считаем итоговое значение поворота
    // console.logValue("calcMotRot", calcMotRot);
    // console.logValue("totalMotRot", totalMotRot);

    chassis.ChassisStop(true);
    if (wheelPivot == WheelPivot.LeftWheel) advmotctrls.SyncMotorsConfig(0, speed);
    else if (wheelPivot == WheelPivot.RightWheel) advmotctrls.SyncMotorsConfig(speed, 0);

    chassis.pidChassisSync.setGains(0.03, 0, 0.5); // Установка значений регулятору
    chassis.pidChassisSync.setControlSaturation(-100, 100); // Ограничения ПИДа
    chassis.pidChassisSync.reset(); // Сброс ПИДа

    let prevTime = 0;
    while (true) {
        let currTime = control.millis();
        let dt = currTime - prevTime;
        prevTime = currTime;

        let eml = chassis.leftMotor.angle();
        let emr = chassis.rightMotor.angle();

        // console.logValue("eml", eml);
        // console.logValue("emr", emr);
        console.sendToScreen();

        if (wheelPivot == WheelPivot.LeftWheel) {
            if (Math.abs(emr) >= Math.abs(totalMotRot)) break;
        } else if (wheelPivot == WheelPivot.RightWheel) {
            if (Math.abs(eml) >= Math.abs(totalMotRot)) break;
        }

        let error = 0;
        if (wheelPivot == WheelPivot.LeftWheel) error = advmotctrls.GetErrorSyncMotors(eml, emr);
        else if (wheelPivot == WheelPivot.RightWheel) error = advmotctrls.GetErrorSyncMotors(eml, emr);
        chassis.pidChassisSync.setPoint(error);
        let U = chassis.pidChassisSync.compute(dt, 0);
        let powers = advmotctrls.GetPwrSyncMotors(U);
        if (wheelPivot == WheelPivot.LeftWheel) chassis.rightMotor.run(powers.pwrRight);
        else if (wheelPivot == WheelPivot.RightWheel) chassis.leftMotor.run(powers.pwrLeft);
        
        control.PauseUntilTime(currTime, 5);
    }
    chassis.ChassisStop(false);
}

// Функция для нормализации сырых значений с датчика
function GetNormRefValCS(refRawValCS: number, bRefRawValCS: number, wRefRawValCS: number): number {
    let refValCS = Math.map(refRawValCS, bRefRawValCS, wRefRawValCS, 0, 100);
    refValCS = Math.constrain(refValCS, 0, 100);
    return refValCS;
}

function Test() {
    motors.mediumB.setInverted(true); motors.mediumC.setInverted(false);
    motors.mediumB.setRegulated(false); motors.mediumC.setRegulated(false);
    motors.mediumB.setBrake(true); motors.mediumC.setBrake(true);
    chassis.setWheelRadius(62.4);
    chassis.setBaseLength(185);
    // motors.mediumB.run(10);
    // motors.mediumC.run(10);
    // motors.mediumBC.run(10);
    // motors.mediumBC.tank(10, 10);
    chassis.setChassisMotors(motors.mediumBC);
    //brick.printString("RUN example", 7, 10);
    brick.buttonEnter.pauseUntil(ButtonEvent.Pressed);
    brick.clearScreen();
    brick.showPorts();
    // PivotTurnExample(90, 30, WheelPivot.LeftWheel);
    //SpinTurnExample(90, 20);
    // ArcMovementExample(25, 50);
    chassis.SyncChassisMovement(20, 20, 360, MoveUnit.Degrees);
}

Test();