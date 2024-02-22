// Синхроннизированное движение на двух средних моторах на расстояние в 600 тиков энкодера прямо
function Example1() {
    advmotctrls2.SyncMotorsConfig(50, 50);

    automation.pid1.setGains(0.03, 0, 0.5); // Установка значений регулятору
    automation.pid1.setControlSaturation(-100, 100); // Ограничения ПИДа
    automation.pid1.reset(); // Сброс ПИДа

    let prevTime = 0;
    while (true) {
        control.timer1.reset();
        let currTime = control.millis()
        let loopTime = currTime - prevTime;
        prevTime = currTime;

        let encB = motors.mediumB.angle();
        let encC = motors.mediumC.angle();
        if ((encB + encC) / 2 >= 600) break;

        let error = advmotctrls2.GetErrorSyncMotors(encB, encC);
        automation.pid1.setPoint(error);
        let U = automation.pid1.compute(loopTime, 0);
        let powers = advmotctrls2.GetPwrSyncMotors(U);
        motors.mediumB.run(powers.pwrLeft);
        motors.mediumC.run(powers.pwrRight);
        control.timer1.pauseUntil(1);
    }
    motors.mediumB.stop();
    motors.mediumC.stop();
    //motors.mediumBC.stop(); // Остановить моторы
}

// Синхроннизированное движение на двух средних моторах на расстояние в 775 тиков энкодера в сторону
function Example2() {
    advmotctrls2.SyncMotorsConfig(25, 50);

    automation.pid1.setGains(0.03, 0, 0.5); // Установка значений регулятору
    automation.pid1.setControlSaturation(-100, 100); // Ограничения ПИДа
    automation.pid1.reset(); // Сброс ПИДа

    let prevTime = 0;
    while (true) {
        control.timer1.reset();
        let currTime = control.millis()
        let loopTime = currTime - prevTime;
        prevTime = currTime;

        let encB = motors.mediumB.angle();
        let encC = motors.mediumC.angle();
        if ((encB + encC) / 2 >= 775) break;

        let error = advmotctrls2.GetErrorSyncMotors(encB, encC);
        automation.pid1.setPoint(error);
        let U = automation.pid1.compute(loopTime, 0);
        let powers = advmotctrls2.GetPwrSyncMotors(U);
        motors.mediumB.run(powers.pwrLeft);
        motors.mediumC.run(powers.pwrRight);

        control.timer1.pauseUntil(1);
    }
    //motors.mediumBC.stop(); // Остановить моторы
    motors.mediumB.stop();
    motors.mediumC.stop();
}

// Синхронизация и плавное ускорение и замедление
function Example3() {
    advmotctrls2.AccTwoEncConfig(15, 90, 100, 300, 1000);

    automation.pid1.setGains(0.03, 0, 0.5); // Установка значений регулятору
    automation.pid1.setControlSaturation(-100, 100); // Ограничения ПИДа
    automation.pid1.reset(); // Сброс ПИДа
    
    let prevTime = 0;
    while (true) {
        control.timer1.reset();
        let currTime = control.millis()
        let loopTime = currTime - prevTime;
        prevTime = currTime;

        let encB = motors.mediumB.angle();
        let encC = motors.mediumC.angle();
        let out = advmotctrls2.AccTwoEnc(encB, encC);
        if (out.isDone) break;

        let error = advmotctrls2.GetErrorSyncMotorsInPwr(encB, encC, out.pwrOut, out.pwrOut);
        automation.pid1.setPoint(error);
        let U = automation.pid1.compute(loopTime, 0);
        let powers = advmotctrls2.GetPwrSyncMotorsInPwr(U, out.pwrOut, out.pwrOut);
        motors.mediumB.run(powers.pwrLeft);
        motors.mediumC.run(powers.pwrRight);

        control.timer1.pauseUntil(1);
    }
    //motors.mediumBC.stop(); // Остановить моторы
    motors.mediumB.stop();
    motors.mediumC.stop();
}

// Плавное ускорение и замедление при движении по линии
function Example4() {
    const B_REF_RAW_CS2 = 636;
    const W_REF_RAW_CS2 = 490;
    const B_REF_RAW_CS3 = 665;
    const W_REF_RAW_CS3 = 501;

    advmotctrls2.AccTwoEncConfig(15, 70, 200, 300, 4000);
    automation.pid1.setGains(0.8, 0, 0.5); // Установка значений регулятору
    automation.pid1.setControlSaturation(-100, 100); // Ограничения ПИДа
    automation.pid1.reset(); // Сброс ПИДа

    let prevTime = 0;
    while (true) {
        control.timer1.reset();
        let currTime = control.millis()
        let loopTime = currTime - prevTime;
        prevTime = currTime;

        let encB = motors.mediumB.angle();
        let encC = motors.mediumC.angle();
        let out = advmotctrls2.AccTwoEnc(encB, encC);
        if (out.isDone) break;

        let rrcs2 = sensors.color2.light(LightIntensityMode.ReflectedRaw);
        let rrcs3 = sensors.color3.light(LightIntensityMode.ReflectedRaw);
        let rcs2 = GetNormRefValCS(rrcs2, B_REF_RAW_CS2, W_REF_RAW_CS2);
        let rcs3 = GetNormRefValCS(rrcs3, B_REF_RAW_CS3, W_REF_RAW_CS3);

        let error = rcs2 - rcs3;
        automation.pid1.setPoint(error);
        let U = automation.pid1.compute(loopTime, 0);
        let pwrLeft = out.pwrOut + U;
        let pwrRight = out.pwrOut - U;
        motors.mediumB.run(pwrLeft);
        motors.mediumC.run(pwrRight);
        
        control.timer1.pauseUntil(10);
    }
    motors.mediumB.stop(); motors.mediumC.stop();
}

// Функция для нормализации сырых значений с датчика
function GetNormRefValCS(refRawValCS: number, bRefRawValCS: number, wRefRawValCS: number): number {
    let refValCS = Math.map(refRawValCS, bRefRawValCS, wRefRawValCS, 0, 100);
    refValCS = Math.constrain(refValCS, 0, 100);
    return refValCS;
}

function Test() {
    motors.mediumB.setInverted(true); motors.mediumC.setInverted(false);
    motors.mediumB.setRegulated(false); motors.mediumC.setRegulated(false)
    motors.mediumB.setBrake(true); motors.mediumC.setBrake(true);
    motors.mediumB.stop(); motors.mediumC.stop();
    motors.mediumB.clearCounts(); motors.mediumC.clearCounts();
    brick.printString("RUN", 7, 13);
    brick.buttonEnter.pauseUntil(ButtonEvent.Pressed);
    brick.clearScreen();
    Example4();
    brick.buttonEnter.pauseUntil(ButtonEvent.Pressed);
}

Test();