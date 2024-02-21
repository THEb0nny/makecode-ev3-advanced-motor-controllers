function MyExample51() {
    advmotctrls2.SyncMotorsConfig(30, 30);

    automation.pid1.setGains(0.03, 0, 0.5); // Установка значений регулятору
    automation.pid1.setControlSaturation(-100, 100); // Ограничения ПИДа
    automation.pid1.reset(); // Сброс ПИДа

    let prevTime = 0;
    while (true) {
        control.timer1.reset();
        let currTime = control.millis()
        let loopTime = currTime - prevTime;
        prevTime = currTime;

        const encB = Math.abs(motors.mediumB.angle());
        const encC = Math.abs(motors.mediumC.angle());
        const error = advmotctrls2.GetErrorSyncMotors(encB, encC);
        automation.pid1.setPoint(error);
        const U = automation.pid1.compute(loopTime, 0);
        const powers = advmotctrls2.GetPowerSyncMotors(U);
        motors.mediumB.run(powers.powerLeft);
        motors.mediumC.run(powers.powerRight);

        if ((encB + encC) / 2 >= 600) break;
        control.timer1.pauseUntil(1);
    }
    motors.mediumBC.stop();
}

// Функция для нормализации сырых значений с датчика
function GetNormRefValCS(refRawValCS: number, bRefRawValCS: number, wRefRawValCS: number): number {
    let refValCS = Math.map(refRawValCS, bRefRawValCS, wRefRawValCS, 0, 100);
    refValCS = Math.constrain(refValCS, 0, 100);
    return refValCS;
}

const B_REF_RAW_CS2 = 652;
const W_REF_RAW_CS2 = 423;

const B_REF_RAW_CS3 = 640;
const W_REF_RAW_CS3 = 462;

function Main() {
    motors.mediumB.setInverted(true); motors.mediumC.setInverted(false);
    motors.mediumB.setBrake(true); motors.mediumC.setBrake(true);
    motors.mediumB.stop(); motors.mediumC.stop();
    motors.mediumB.clearCounts(); motors.mediumC.clearCounts();
    brick.printString("RUN", 7, 13);
    brick.buttonEnter.pauseUntil(ButtonEvent.Pressed);
    brick.clearScreen();
    MyExample51();
    brick.buttonEnter.pauseUntil(ButtonEvent.Pressed);
}

Main();