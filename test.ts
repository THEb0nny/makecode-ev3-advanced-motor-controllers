// Arc synchronized movement
function ArcMovementExample(lMotPwr: number, rMotPwr: number) {
    //if (!motorsPair) return;
    advmotctrls.syncMotorsConfig(lMotPwr, rMotPwr);
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
        let error = advmotctrls.getErrorSyncMotors(encB, encC);
        chassis.pidChassisSync.setPoint(error);
        let U = chassis.pidChassisSync.compute(dt, 0);
        let powers = advmotctrls.getPwrSyncMotors(U);
        chassis.leftMotor.run(powers.pwrLeft);
        chassis.rightMotor.run(powers.pwrRight);
        control.pauseUntilTime(currTime, 5);
    }
    chassis.stop(true);
}

function LineFollowExample(speed: number) {
    const B_REF_RAW_CS2 = 636;
    const W_REF_RAW_CS2 = 490;
    const B_REF_RAW_CS3 = 665;
    const W_REF_RAW_CS3 = 501;

    advmotctrls.syncMotorsConfig(speed, speed);
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
        let powers = advmotctrls.getPwrSyncMotors(U);
        // let pwrLeft = out.pwrOut + U;
        // let pwrRight = out.pwrOut - U;
        chassis.leftMotor.run(powers.pwrLeft);
        chassis.rightMotor.run(powers.pwrRight);

        control.pauseUntilTime(currTime, 10);
    }
    chassis.stop(true);
}

// Smooth acceleration and deceleration when moving along the line
function AccelLineFollowExample() {
    const B_REF_RAW_CS2 = 636;
    const W_REF_RAW_CS2 = 490;
    const B_REF_RAW_CS3 = 665;
    const W_REF_RAW_CS3 = 501;

    advmotctrls.accTwoEncConfig(15, 70, 200, 300, 4000);
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
        let out = advmotctrls.accTwoEnc(eml, emr);
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
        
        control.pauseUntilTime(currTime, 10);
    }
    chassis.stop(true);
}

// Функция для нормализации сырых значений с датчика
function GetNormRefValCS(refRawValCS: number, bRefRawValCS: number, wRefRawValCS: number): number {
    let refValCS = Math.map(refRawValCS, bRefRawValCS, wRefRawValCS, 0, 100);
    refValCS = Math.constrain(refValCS, 0, 100);
    return refValCS;
}

function Test() {
    // chassis.setChassisMotors(motors.mediumBC);
    // chassis.setChassisMotors(motors.largeBC);
    chassis.setSeparatelyChassisMotors(motors.mediumB, motors.mediumC, true, false);
    chassis.setRegulatorGains(0.02, 0, 0.5);
    chassis.setWheelRadius(62.4);
    chassis.setBaseLength(185);
    brick.printString("RUN example", 7, 10);
    brick.buttonEnter.pauseUntil(ButtonEvent.Pressed);
    brick.clearScreen();
    // chassis.syncMovement(-20, -20, -500, MoveUnit.Degrees);
    // SyncAccelStraightlineMovementExample(5, 30, 50, 50, 400);
    // chassis.pivotTurn(90, 30, WheelPivot.LeftWheel);
    // SpinTurnExample(90, 20);
    // chassis.spinTurn(90, 20);
    // ArcMovementExample(25, 50);
    // chassis.SyncChassisMovement(20, 20, 360, MoveUnit.Degrees);
}

Test();