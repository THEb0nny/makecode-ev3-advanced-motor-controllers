namespace chassis {

    const pidChassisSync = new automation.PIDController();

    /**
        Synchronized chassis movement.
        @param vLeft входное значение скорости левого мотора, eg. 50
        @param vRight входное значение скорости правого мотора, eg. 50
    **/
    //% blockId=SyncMotorsConfig
    //% block="Synchronized chassis movement at vLeft = $vLeft|vRight = $vRight|lenght = $lenght"
    export function SyncChassisMovement(vLeft: number, vRight: number, value: number = 0, unit: MoveUnit = MoveUnit.Degrees) {
        vLeft = Math.clamp(-100, 100, vLeft >> 0);
        vRight = Math.clamp(-100, 100, vRight >> 0);

        advmotctrls.SyncMotorsConfig(vLeft, vRight); // Set motor speeds for subsequent regulation

        pidChassisSync.setGains(0.03, 0, 0.5); // Setting the regulator coefficients
        pidChassisSync.setControlSaturation(-100, 100); // Regulator limitation
        pidChassisSync.reset(); // Reset pid controller

        let prevTime = 0; // Last time time variable for loop
        let startTime = control.millis();
        while (true) { // Synchronized motion control cycle
            let currTime = control.millis();
            let dt = currTime - prevTime;
            prevTime = currTime;

            let encB = motors.mediumB.angle(); // Get left motor encoder current value
            let encC = motors.mediumC.angle(); // Get right motor encoder current value
            if (unit == MoveUnit.Degrees) {
                if ((encB + encC) / 2 >= value) break;
            } else if (unit == MoveUnit.Rotations) {

            } else if (unit == MoveUnit.MilliSeconds) {

            } else if (unit == MoveUnit.Seconds) {

            }

            let error = advmotctrls.GetErrorSyncMotors(encB, encC); // Find out the error in motor speed control
            pidChassisSync.setPoint(error); // Transfer control error to controller
            let U = pidChassisSync.compute(dt, 0); // Find out and record the control action of the regulator
            let powers = advmotctrls.GetPwrSyncMotors(U); // Find out the power of motors for regulation
            motors.mediumB.run(powers.pwrLeft); // Set power/speed left motor
            motors.mediumC.run(powers.pwrRight); // Set power/speed right motor
            advmotctrls.PauseUntilTime(currTime, 5); // Wait until the control cycle reaches the set amount of time passed
        }
        motors.mediumB.stop(); // Stop left motor
        motors.mediumC.stop(); // Stop right motor
        //motors.mediumBC.stop(); // Остановить моторы
    }
    
}