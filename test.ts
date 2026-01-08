// ОШИБКИ при панике
// 1) Функции spinTurn передана отрицательная скорость speed
// 2) Функции pivotTurn передан отрицательный угол deg

/*
function LineFollowExample(speed: number) {
    const B_REF_RAW_CS2 = 636, W_REF_RAW_CS2 = 490;
    const B_REF_RAW_CS3 = 665, W_REF_RAW_CS3 = 501;
    advmotctrls.syncMotorsConfig(speed, speed);
    chassis.pidChassisSync.setGains(0.8, 0, 0.5); // Установка значений регулятору
    chassis.pidChassisSync.setControlSaturation(-100, 100); // Ограничения ПИДа
    chassis.pidChassisSync.setPoint(0);
    chassis.pidChassisSync.reset(); // Сброс ПИДа
    let prevTime = control.millis();
    while (true) {
        const currTime = control.millis();
        const dt = currTime - prevTime;
        prevTime = currTime;
        const rrcs2 = sensors.color2.light(LightIntensityMode.ReflectedRaw);
        const rrcs3 = sensors.color3.light(LightIntensityMode.ReflectedRaw);
        let rcs2 = Math.map(rrcs2, B_REF_RAW_CS2, W_REF_RAW_CS2, 0, 100);
        rcs2 = Math.constrain(rcs2, 0, 100);
        let rcs3 = Math.map(rrcs3, B_REF_RAW_CS3, B_REF_RAW_CS3, 0, 100);
        rcs3 = Math.constrain(rcs3, 0, 100);
        const eml = chassis.leftMotor.angle(), emr = chassis.rightMotor.angle();
        //const sync_error = advmotctrls.GetErrorSyncMotors(eml, emr);
        const error = rcs2 - rcs3;
        const u = chassis.pidChassisSync.compute(dt == 0 ? 1 : dt, -error);
        const powers = advmotctrls.getPwrSyncMotors(u);
        chassis.setSpeedsCommand(powers.pwrLeft, powers.pwrRight);
        control.pauseUntilTime(currTime, 10);
    }
    chassis.stop(true);
}
*/


function RampArcMovementExample(vStarting: number, vLeftMax: number, vRightMax: number, vFinishing: number, accelDist: number, decelDist: number, totalDist: number, debug: boolean = false) {
    if (totalDist == 0) {
        return;
    }
    if (vLeftMax == 0 && vRightMax == 0) { // Если обе набираемых скоростей указаны как 0
        console.log("Error: Both vLeftMax and vRightMax cannot be 0!");
        control.assert(false, 2);
        return;
    }
    
    const emlPrev = chassis.leftMotor.angle(), emrPrev = chassis.rightMotor.angle();
    const direction = totalDist >= 0 ? 1 : -1;
    vStarting = Math.clamp(0, 100, Math.abs(vStarting) >> 0);
    vLeftMax = Math.clamp(0, 100, Math.abs(vLeftMax) >> 0);
    vRightMax = Math.clamp(0, 100, Math.abs(vRightMax) >> 0);
    vFinishing = Math.clamp(0, 100, Math.abs(vFinishing) >> 0);
    
    if (vStarting > vLeftMax || vStarting > vRightMax) { // Проверка логики скоростей(стартовая должна быть меньше максимальной)
        console.log(`Warning: vStarting (${vStarting}) greater than max speeds. This may cause unexpected behavior.`);
    }

    vLeftMax *= direction, vRightMax *= direction; // Указываем направление движения в макс скоростях

    if (accelDist < 0) { // Проверка дистанций на отрицательные значения (должны быть положительными перед abs())
        console.log(`Warning: accelDist is negative (${accelDist}). Using absolute value.`);
    }
    if (decelDist < 0) {
        console.log(`Warning: decelDist is negative (${decelDist}). Using absolute value.`);
    }

    const absTotalDist = Math.abs(totalDist);
    const absAccelDist = Math.abs(accelDist);
    const absDecelDist = Math.abs(decelDist);
    const accelCalcMotRot = (absAccelDist / (Math.PI * chassis.getWheelDiametr())) * motors.cpr;
    const decelCalcMotRot = (absDecelDist / (Math.PI * chassis.getWheelDiametr())) * motors.cpr;
    const totalCalcMotRot = (absTotalDist / (Math.PI * chassis.getWheelDiametr())) * motors.cpr;

    if (absAccelDist + absDecelDist > absTotalDist) { // Проверка суммы дистанций ускорения/замедления
        console.log(`Warning: accelDist (${absAccelDist}) + decelDist (${absDecelDist}) > totalDist (${absTotalDist}). Profile will be scaled down.`);
    }

    advmotctrls.accTwoEncComplexMotionConfig(vStarting, vLeftMax, vRightMax, vFinishing, totalCalcMotRot, accelCalcMotRot, decelCalcMotRot);
    
    chassis.pidChassisSync.setGains(chassis.getSyncRegulatorKp(), chassis.getSyncRegulatorKi(), chassis.getSyncRegulatorKd());
    chassis.pidChassisSync.setControlSaturation(-100, 100);
    chassis.pidChassisSync.setPoint(0); // Установить нулевую уставку регулятору
    chassis.pidChassisSync.reset();
    
    control.timer8.reset();
    let prevTime = control.millis();
    while (true) {
        const currTime = control.millis();
        const dt = currTime - prevTime;
        prevTime = currTime;
        const eml = chassis.leftMotor.angle() - emlPrev, emr = chassis.rightMotor.angle() - emrPrev;
        const out = advmotctrls.accTwoEncComplexMotionCompute(eml, emr);
        if (out.isDoneLeft || out.isDoneRight) break;
        const error = advmotctrls.getErrorSyncMotorsAtPwr(eml, emr, out.pwrLeft, out.pwrRight);
        const u = chassis.pidChassisSync.compute(dt == 0 ? 1 : dt, -error);
        const powers = advmotctrls.getPwrSyncMotorsAtPwr(u, out.pwrLeft, out.pwrRight);
        chassis.setSpeedsCommand(powers.pwrLeft, powers.pwrRight);
        if (debug && control.timer8.millis() >= 10) {
            console.log(`pwrLeft: ${out.pwrLeft}, pwrRight: ${out.pwrRight}, eml: ${eml}, emr: ${emr}`);
            control.timer8.reset();
        }
        control.pauseUntilTime(currTime, 1);
    }
    chassis.stop(Braking.Hold);
}


function Test() {
    chassis.setChassisMotors(motors.mediumB, motors.mediumC, true, false);
    chassis.setWheelDiametr(62.4);
    chassis.setBaseLength(175);
    chassis.setSyncRegulatorGains(0.01, 0, 0.5);
    brick.printString("RUN example", 7, 10);
    brick.buttonEnter.pauseUntil(ButtonEvent.Pressed);
    brick.clearScreen();
    // RampArcMovementExample(30, 50, 50, 20, 100, 150, 300);
    // pause(1000);
    // RampArcMovementExample(30, 50, 50, 20, 0, 150, 300);
    // pause(1000);
    // RampArcMovementExample(30, 50, 50, 20, 100, 0, 300);
    // pause(1000);
    // RampArcMovementExample(30, 50, 50, 20, 100, 150, -300);
    // pause(1000);
    // RampArcMovementExample(30, 50, 80, 20, 100, 150, 300);
    // pause(1000);
    // RampArcMovementExample(30, 80, 50, 20, 100, 150, 300);
    // pause(1000);
    // RampArcMovementExample(30, 50, 80, 20, 100, 150, -300);
    // pause(1000);
    // RampArcMovementExample(30, 80, 50, 20, 100, 150, -300);

    // rampSpinTurnExample(-90, 35, 90);
    // pause(1000);
    // rampSpinTurnExample(180, 35, 80, 45, 45);
    // pause(1000);
    // rampSpinTurnExample(90, 35, 80, 30, 30);
    // pause(1000);
    // rampSpinTurnExample(90, 35, 80);
    // pause(1000);
}

Test();