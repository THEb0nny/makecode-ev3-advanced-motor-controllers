// Rectilinear movement with synchronization
function StraightlineMovementExample() {
    advmotctrls.SyncMotorsConfig(20, 20);

    automation.pid1.setGains(0.03, 0, 0.5); // Установка значений регулятору
    automation.pid1.setControlSaturation(-100, 100); // Ограничения ПИДа
    automation.pid1.reset(); // Сброс ПИДа

    let prevTime = 0;
    while (true) {
        let currTime = control.millis();
        let dt = currTime - prevTime;
        prevTime = currTime;

        let encB = chassis.leftMotor.angle();
        let encC = chassis.rightMotor.angle();
        if ((encB + encC) / 2 >= 600) break;

        let error = advmotctrls.GetErrorSyncMotors(encB, encC);
        automation.pid1.setPoint(error);
        let U = automation.pid1.compute(dt, 0);
        let powers = advmotctrls.GetPwrSyncMotors(U);
        chassis.leftMotor.run(powers.pwrLeft);
        chassis.rightMotor.run(powers.pwrRight);
        control.PauseUntilTime(currTime, 5);
    }
    chassis.ChassisStop();
}

// Arc synchronized movement
function ArcMovementExample() {
    advmotctrls.SyncMotorsConfig(25, 50);

    automation.pid1.setGains(0.03, 0, 0.5); // Установка значений регулятору
    automation.pid1.setControlSaturation(-100, 100); // Ограничения ПИДа
    automation.pid1.reset(); // Сброс ПИДа

    let prevTime = 0;
    while (true) {
        let currTime = control.millis();
        let dt = currTime - prevTime;
        prevTime = currTime;

        let encB = chassis.leftMotor.angle();
        let encC = chassis.rightMotor.angle();
        if ((encB + encC) / 2 >= 775) break;

        let error = advmotctrls.GetErrorSyncMotors(encB, encC);
        automation.pid1.setPoint(error);
        let U = automation.pid1.compute(dt, 0);
        let powers = advmotctrls.GetPwrSyncMotors(U);
        chassis.leftMotor.run(powers.pwrLeft);
        chassis.rightMotor.run(powers.pwrRight);
        control.PauseUntilTime(currTime, 5);
    }
    chassis.ChassisStop();
}

// Synchronization with smooth acceleration and deceleration during straight-line motion
function SyncAccelerationStraightlineMovementExample() {
    advmotctrls.AccTwoEncConfig(15, 90, 100, 300, 1000);

    automation.pid1.setGains(0.03, 0, 0.5); // Установка значений регулятору
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

        let error = advmotctrls.GetErrorSyncMotorsInPwr(eml, emr, out.pwrOut, out.pwrOut);
        automation.pid1.setPoint(error);
        let U = automation.pid1.compute(dt, 0);
        let powers = advmotctrls.GetPwrSyncMotorsInPwr(U, out.pwrOut, out.pwrOut);
        chassis.leftMotor.run(powers.pwrLeft);
        chassis.rightMotor.run(powers.pwrRight);

        control.PauseUntilTime(currTime, 5);
    }
    chassis.ChassisStop();
}

// Smooth acceleration and deceleration when moving along the line
function AccelLineFollowExample() {
    const B_REF_RAW_CS2 = 636;
    const W_REF_RAW_CS2 = 490;
    const B_REF_RAW_CS3 = 665;
    const W_REF_RAW_CS3 = 501;

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
        
        control.PauseUntilTime(currTime, 5);
    }
    chassis.ChassisStop();
}

// Синхроннизированный поворот на двух средних моторах на нужный угол
function TurnExample(deg: number, speed: number) {
    if (deg == 0 || speed <= 0) return;

    let emlPrev = chassis.leftMotor.angle(), emrPrev = chassis.rightMotor.angle(); // Считываем с моторов значения с энкодеров перед стартом алгаритма
    let calcMotRot = Math.round(deg * chassis.getBaseLength() / chassis.getWheelRadius()); // Расчёт угла поворота моторов для поворота
    let lMotRotCalc = emlPrev + calcMotRot, rMotRotCalc = emrPrev + calcMotRot * -1; // Расчитываем итоговое значение углов на каждый мотор

    if (deg > 0) advmotctrls.SyncMotorsConfig(speed, -speed);
    else if (deg < 0) advmotctrls.SyncMotorsConfig(-speed, speed);

    automation.pid1.setGains(0.03, 0, 0.5); // Установка значений регулятору
    automation.pid1.setControlSaturation(-100, 100); // Ограничения ПИДа
    automation.pid1.reset(); // Сброс ПИДа

    let prevTime = 0;
    while (true) {
        let currTime = control.millis();
        let dt = currTime - prevTime;
        prevTime = currTime;

        let eml = chassis.leftMotor.angle();
        let emr = chassis.rightMotor.angle();

        if ((Math.abs(eml) + Math.abs(emr)) / 2 >= Math.abs(calcMotRot)) break;

        let error = advmotctrls.GetErrorSyncMotors(eml, emr);
        automation.pid1.setPoint(error);
        let U = automation.pid1.compute(dt, 0);
        let powers = advmotctrls.GetPwrSyncMotors(U);
        chassis.leftMotor.run(powers.pwrLeft);
        chassis.rightMotor.run(powers.pwrRight);

        control.PauseUntilTime(currTime, 5);
    }
    chassis.ChassisStop();
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
    chassis.setBaseLength(180);
    // motors.mediumB.run(10); motors.mediumC.run(10);
    // motors.mediumBC.run(10);
    // motors.mediumBC.tank(10, 10);
    chassis.setChassisMotors(motors.mediumBC);
    brick.printString("RUN example", 7, 10);
    brick.buttonEnter.pauseUntil(ButtonEvent.Pressed);
    brick.clearScreen();
    TurnExample(-90, 40);
    // chassis.SyncChassisMovement(20, 20, 360, MoveUnit.Degrees);
}

Test();