// ОШИБКИ при панике
// 1) Функции spinTurn передана отрицательная скорость speed
// 2) Функции pivotTurn передан отрицательный угол deg

/*
function LineFollowExample(speed: number) {
    const B_REF_RAW_CS2 = 636, W_REF_RAW_CS2 = 490;
    const B_REF_RAW_CS3 = 665, W_REF_RAW_CS3 = 501;
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
*/

function RampArcMovementExample(vStarting: number, vLeftMax: number, vRightMax: number, vFinishing: number, accelDist: number, decelDist: number, totalDist: number, debug: boolean = false) {
    const emlPrev = chassis.leftMotor.angle(), emrPrev = chassis.rightMotor.angle();
    const accelCalcMotRot = (accelDist / (Math.PI * chassis.getWheelDiametr())) * motors.cpr;
    const decelCalcMotRot = (decelDist / (Math.PI * chassis.getWheelDiametr())) * motors.cpr;
    const calcMotRot = (totalDist / (Math.PI * chassis.getWheelDiametr())) * motors.cpr;

    advmotctrls.accTwoEncComplexMotionConfig(vStarting, vLeftMax, vRightMax, vFinishing, accelCalcMotRot, decelCalcMotRot, calcMotRot);
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
        const u = chassis.pidChassisSync.compute(dt, -error);
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

function rampSpinTurnExample(deg: number, maxSpeed: number, accelDeg?: number, decelDeg?: number, timeOut?: number) {
    if (deg == 0 || maxSpeed == 0) {
        chassis.stop(Braking.Hold);
        return;
    } else if (maxSpeed < 0) {
        console.log("Error: the rotation maxSpeed relative to the center is negative!");
        control.assert(false, 7);
    }

    maxSpeed = Math.clamp(0, 100, maxSpeed >> 0); // Ограничиваем скорость от 0 до 100 и отсекаем дробную часть
    const emlPrev = chassis.leftMotor.angle(), emrPrev = chassis.rightMotor.angle(); // Считываем значение с энкодера с левого двигателя, правого двигателя перед запуском
    const absDeg = Math.abs(deg);
    accelDeg = accelDeg !== undefined ? accelDeg : absDeg * 0.20; // 20% на ускорение
    decelDeg = decelDeg !== undefined ? decelDeg : absDeg * 0.20; // 20% на замедление
    if (accelDeg + decelDeg > absDeg) { // Проверка: если ускорение + замедление > всего пути, обрезаем
        const ratio = absDeg / (accelDeg + decelDeg);
        accelDeg *= ratio;
        decelDeg *= ratio;
    }
    const accelCalcMotRot = Math.round(accelDeg * chassis.getBaseLength() / chassis.getWheelDiametr()); // Расчёт угла поворота моторов для поворота
    const decelCalcMotRot = Math.round(decelDeg * chassis.getBaseLength() / chassis.getWheelDiametr());
    const calcMotRot = Math.round(absDeg * chassis.getBaseLength() / chassis.getWheelDiametr()); // Расчёт угла поворота моторов для поворота
    const vStarting = 30;
    const vFinishing = 30;
    const vLeftMax = deg > 0 ? maxSpeed : -maxSpeed;
    const vRightMax = deg > 0 ? -maxSpeed : maxSpeed;

    advmotctrls.accTwoEncComplexMotionConfig(vStarting, vLeftMax, vRightMax, vFinishing, accelCalcMotRot, decelCalcMotRot, calcMotRot);
    chassis.pidChassisSync.setGains(chassis.getSyncRegulatorKp(), chassis.getSyncRegulatorKi(), chassis.getSyncRegulatorKd()); // Установка коэффицентов ПИД регулятора
    chassis.pidChassisSync.setControlSaturation(-100, 100); // Установка интервалов регулирования
    chassis.pidChassisSync.setPoint(0); // Установить нулевую уставку регулятору
    chassis.pidChassisSync.reset();

    let prevTime = control.millis(); // Переменная для хранения предыдущего времени для цикла регулирования
    const startTime = control.millis(); // Стартовое время алгоритма
    while (true) {
        const currTime = control.millis();
        const dt = currTime - prevTime;
        prevTime = currTime;
        if (timeOut && currTime - startTime >= timeOut) break; // Выход из алгоритма, если время вышло
        const eml = chassis.leftMotor.angle() - emlPrev, emr = chassis.rightMotor.angle() - emrPrev;
        const out = advmotctrls.accTwoEncComplexMotionCompute(eml, emr);
        if (out.isDoneLeft || out.isDoneRight
            || ((Math.abs(eml) + Math.abs(emr)) / 2 >= Math.abs(calcMotRot))) break;
        const error = advmotctrls.getErrorSyncMotorsAtPwr(eml, emr, out.pwrLeft, out.pwrRight);
        const u = chassis.pidChassisSync.compute(dt, -error);
        const powers = advmotctrls.getPwrSyncMotorsAtPwr(u, out.pwrLeft, out.pwrRight);
        chassis.setSpeedsCommand(powers.pwrLeft, powers.pwrRight);
        control.pauseUntilTime(currTime, 1);
    }
    chassis.stop(Braking.Hold); // Удерживание при торможении
}

function Test() {
    chassis.setChassisMotors(motors.mediumB, motors.mediumC, true, false);
    chassis.setWheelDiametr(62.4);
    chassis.setBaseLength(175);
    chassis.setSyncRegulatorGains(0.01, 0, 0);
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

    // rampSpinTurnExample(-90, 90, 30);
    // pause(1000);
    // rampSpinTurnExample(180, 80, 45, 45);
    // pause(1000);
    // rampSpinTurnExample(90, 80, 30, 30);
    // pause(1000);
    // rampSpinTurnExample(90, 80); // Вариант 2: передаешь только градусы и скорость (дефолт 15%/15%)
    // pause(1000);

    // chassis.syncMovement(-20, -20, -500, MoveUnit.Degrees);
    // chassis.pivotTurn(90, 30, WheelPivot.LeftWheel);
    // chassis.spinTurn(90, 20);
}

Test();