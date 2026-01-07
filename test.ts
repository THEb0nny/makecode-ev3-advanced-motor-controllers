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
    const emlPrev = chassis.leftMotor.angle(), emrPrev = chassis.rightMotor.angle();
    const accelCalcMotRot = (accelDist / (Math.PI * chassis.getWheelDiametr())) * motors.cpr;
    const decelCalcMotRot = (decelDist / (Math.PI * chassis.getWheelDiametr())) * motors.cpr;
    const totalCalcMotRot = (totalDist / (Math.PI * chassis.getWheelDiametr())) * motors.cpr;

    advmotctrls.accTwoEncComplexMotionConfig(vStarting, vLeftMax, vRightMax, vFinishing, accelCalcMotRot, decelCalcMotRot, totalCalcMotRot);
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
    // RampArcMovementExample(30, -50, -50, 20, 100, 150, 300);
    // pause(1000);
    // RampArcMovementExample(30, -50, -80, 20, 100, 150, 300);
    // pause(1000);
    // RampArcMovementExample(30, -80, -50, 20, 100, 150, 300);
    // pause(1000);
    // RampArcMovementExample(30, -80, 80, 30, 200, 300, 600);
    // pause(1000);
    // RampArcMovementExample(30, 70, -70, 30, 200, 300, 500);

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