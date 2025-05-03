// ОШИБКИ при панике
// 1) Функции spinTurn передана отрицательная скорость speed
// 2) Функции pivotTurn передан отрицательный угол deg

/*
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
        let rcs2 = Math.map(rrcs2, B_REF_RAW_CS2, W_REF_RAW_CS2, 0, 100);
        rcs2 = Math.constrain(rcs2, 0, 100);
        let rcs3 = Math.map(rrcs3, B_REF_RAW_CS3, B_REF_RAW_CS3, 0, 100);
        rcs3 = Math.constrain(rcs3, 0, 100);
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
function RampLineFollowExample() {
    const B_REF_RAW_CS2 = 636;
    const W_REF_RAW_CS2 = 490;
    const B_REF_RAW_CS3 = 665;
    const W_REF_RAW_CS3 = 501;
    advmotctrls.linearAccTwoEncConfig(15, 50, 15, 200, 300, 4000);
    automation.pid1.setGains(0.8, 0, 0.5); // Установка значений регулятору
    automation.pid1.setControlSaturation(-200, 200); // Ограничения ПИДа
    automation.pid1.reset(); // Сброс ПИДа
    let prevTime = 0;
    while (true) {
        let currTime = control.millis();
        let dt = currTime - prevTime;
        prevTime = currTime;
        let eml = chassis.leftMotor.angle();
        let emr = chassis.rightMotor.angle();
        let out = advmotctrls.linearAccTwoEnc(eml, emr);
        if (out.isDone) break;
        let rrcs2 = sensors.color2.light(LightIntensityMode.ReflectedRaw);
        let rrcs3 = sensors.color3.light(LightIntensityMode.ReflectedRaw);
        let rcs2 = Math.map(rrcs2, B_REF_RAW_CS2, W_REF_RAW_CS2, 0, 100);
        rcs2 = Math.constrain(rcs2, 0, 100);
        let rcs3 = Math.map(rrcs3, B_REF_RAW_CS3, B_REF_RAW_CS3, 0, 100);
        rcs3 = Math.constrain(rcs3, 0, 100);
        let error = rcs2 - rcs3;
        automation.pid1.setPoint(error);
        let U = automation.pid1.compute(dt, 0);
        let pwrLeft = out.pwrLeft + U;
        let pwrRight = out.pwrRight - U;
        chassis.leftMotor.run(pwrLeft);
        chassis.rightMotor.run(pwrRight);
        control.pauseUntilTime(currTime, 10);
    }
    chassis.stop(true);
}
*/

function RampArcMovementExample() {
    const emlPrev = chassis.leftMotor.angle(), emrPrev = chassis.rightMotor.angle(); // We read the value from the encoder from the left motor and right motor before starting

    // advmotctrls.syncMotorsConfig(lMotPwr, rMotPwr); // Обновлять в цикле
    advmotctrls.accTwoEncExtConfig(20, 20, 50, 75, 10, 10, 400, 400, 1000);
    chassis.pidChassisSync.setGains(chassis.getSyncRegulatorKp(), chassis.getSyncRegulatorKi(), chassis.getSyncRegulatorKd()); // Установка значений регулятору
    chassis.pidChassisSync.setControlSaturation(-100, 100); // Ограничения ПИДа
    chassis.pidChassisSync.reset(); // Сброс ПИДа
    control.timer8.reset();
    let prevTime = 0;
    while (true) {
        let currTime = control.millis();
        let dt = currTime - prevTime;
        prevTime = currTime;
        let eml = chassis.leftMotor.angle() - emlPrev, emr = chassis.rightMotor.angle() - emrPrev; // Get left motor and right motor encoder current value
        let out = advmotctrls.accTwoEncExt(eml, emr);
        if (out.isDone) break;
        // advmotctrls.syncMotorsConfig(out.pwrLeft, out.pwrRight); // Обновлять в цикле
        // let error = advmotctrls.getErrorSyncMotors(eml, emr);
        // let error = advmotctrls.getErrorSyncMotorsAtPwr(eml, emr, out.pwrLeft, out.pwrRight);
        // chassis.pidChassisSync.setPoint(error);
        // let U = chassis.pidChassisSync.compute(dt, 0);
        // let powers = advmotctrls.getPwrSyncMotorsAtPwr(U, out.pwrLeft, out.pwrRight);
        // let powers = advmotctrls.getPwrSyncMotors(U); // Find out the power of motors for regulation
        chassis.setSpeedsCommand(out.pwrLeft, out.pwrRight); // Set power/speed motors
        if (control.timer8.millis() >= 15) {
            console.log(`time: ${control.millis()}, pwrLeft: ${out.pwrLeft}, pwrRight: ${out.pwrRight}, eml: ${eml}, emr: ${emr}`);
            control.timer8.reset();
        }
        control.pauseUntilTime(currTime, 1);
    }
    chassis.stop(true);
}

function Test() {
    // chassis.setChassisMotors(motors.mediumBC);
    chassis.setChassisMotors(motors.mediumB, motors.mediumC, true, false);
    chassis.setSyncRegulatorGains(0.02, 0, 0.5);
    chassis.setWheelDiametr(62.4);
    chassis.setBaseLength(185);
    brick.printString("RUN example", 7, 10);
    brick.buttonEnter.pauseUntil(ButtonEvent.Pressed);
    brick.clearScreen();
    RampArcMovementExample();
    // chassis.syncMovement(-20, -20, -500, MoveUnit.Degrees);
    // chassis.pivotTurn(90, 30, WheelPivot.LeftWheel);
    // chassis.spinTurn(90, 20);
    // ArcMovementExample(25, 50, 775);
}

Test();