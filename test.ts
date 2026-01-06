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


function RampPivotTurnExample(wheelPivot: WheelPivot, deg: number, vMin: number, vMax: number, accelDeg?: number, decelDeg?: number, timeOut?: number, debug: boolean = false) {
    chassis.stop(Braking.Hold); // Установить тормоз и удержание моторов перед поворотом
    vMin = Math.clamp(-100, 100, vMin >> 0); // Ограничиваем мин скорость от -100 до 100 и отсекаем дробную часть
    vMax = Math.clamp(-100, 100, vMax >> 0); // Ограничиваем макс скорость от -100 до 100 и отсекаем дробную часть
    // Проверка перепутанных скоростей по модулю
    if (Math.abs(vMin) > Math.abs(vMax)) {
        const temp = vMin;
        vMin = vMax;
        vMax = temp;
    }
    // Проверка равенства скоростей по модулю
    if (Math.abs(vMin) === Math.abs(vMax)) {
        if (vMin > 0) vMin = Math.max(0, vMin - 10);
        else vMin = Math.min(0, vMin + 10);
    }
    const emlPrev = chassis.leftMotor.angle(), emrPrev = chassis.rightMotor.angle(); // Считываем значение с энкодера с левого двигателя, правого двигателя перед запуском
    const absDeg = Math.abs(deg); // Угол поворота
    accelDeg = accelDeg !== undefined ? accelDeg : absDeg * 0.25; // 25% на ускорение
    decelDeg = decelDeg !== undefined ? decelDeg : absDeg * 0.25; // 25% на замедление
    if (accelDeg + decelDeg > absDeg) { // Проверка если ускорение + замедление > всего пути, обрезаем
        const ratio = absDeg / (accelDeg + decelDeg);
        accelDeg *= ratio;
        decelDeg *= ratio;
    }
    const accelCalcMotRot = Math.round(((accelDeg * chassis.getBaseLength()) / chassis.getWheelDiametr()) * 2); // Расчёт угла поворота моторов для поворота для ускорения
    const decelCalcMotRot = Math.round(((decelDeg * chassis.getBaseLength()) / chassis.getWheelDiametr()) * 2); // Расчёт угла поворота моторов для поворота для замедления
    const totalCalcMotRot = Math.round(((Math.abs(deg) * chassis.getBaseLength()) / chassis.getWheelDiametr()) * 2); // Расчёт угла поворота моторов для поворота общего угла
    const v = deg > 0 ? vMax : -vMax;
    const vLeftMax = wheelPivot === WheelPivot.LeftWheel ? 0 : v;
    const vRightMax = wheelPivot === WheelPivot.LeftWheel ? v : 0;

    advmotctrls.accTwoEncComplexMotionConfig(vMin, vLeftMax, vRightMax, vMin, accelCalcMotRot, decelCalcMotRot, totalCalcMotRot);
    chassis.pidChassisSync.setGains(chassis.getSyncRegulatorKp(), chassis.getSyncRegulatorKi(), chassis.getSyncRegulatorKd()); // Установка коэффицентов ПИД регулятора
    chassis.pidChassisSync.setControlSaturation(-100, 100); // Установка интервалов регулирования
    chassis.pidChassisSync.setPoint(0); // Установить нулевую уставку регулятору
    chassis.pidChassisSync.reset(); // Сбросить регулятор

    let prevTime = control.millis(); // Переменная для хранения предыдущего времени для цикла регулирования
    const startTime = control.millis(); // Стартовое время алгоритма
    while (true) {
        const currTime = control.millis();
        const dt = currTime - prevTime;
        prevTime = currTime;
        if (timeOut && currTime - startTime >= timeOut) break; // Выход из алгоритма, если время вышло
        const eml = chassis.leftMotor.angle() - emlPrev, emr = chassis.rightMotor.angle() - emrPrev;
        const out = advmotctrls.accTwoEncComplexMotionCompute(eml, emr);
        // Условие выхода: проверяем только движущееся колесо
        if (wheelPivot == WheelPivot.LeftWheel && Math.abs(emr) >= totalCalcMotRot ||
            wheelPivot == WheelPivot.RightWheel && Math.abs(eml) >= totalCalcMotRot) {
                break;
        }
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
    chassis.stop(Braking.Hold); // Удерживание при торможении
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

    RampPivotTurnExample(WheelPivot.LeftWheel, 90, 40, 70);
    pause(1000);
    RampPivotTurnExample(WheelPivot.LeftWheel, 90, -40, -70);
}

Test();