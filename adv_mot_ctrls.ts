/**
 * Motor controllers based OFDL Advanced Motor Controller Block module (algorithm part).
 * Based 1.1 ver, 2023/09/27.
 * https://github.com/ofdl-robotics-tw/EV3-CLEV3R-Modules/blob/main/Mods/AdvMtCtrls.bpm
 */
//% block="AdvMotCtrls" weight="89" color="#02ab38" icon="\uf3fd"
namespace advmotctrls {

    let syncVLeft: number;
    let syncVRight: number;
    let syncVLeftSign: number;
    let syncVRightSign: number;

    let accMotorMinPwr: number;
    let accMotorMaxPwr: number;
    let accMotorAccelDist: number;
    let accMotorDecelDist: number;
    let accMotorTotalDist: number;
    let accMotorIsNeg: boolean;

    // let accTwoEncMinPwr: number;
    let accMotorsStartPwr: number;
    let accMotorsMaxPwr: number;
    let accMotorsEndPwr: number;
    let accMotorsAccelDist: number;
    let accMotorsDecelDist: number;
    let accMotorsTotalDist: number;
    let accMotorsIsNeg: boolean;

    interface MotorsPower {
        pwrLeft: number;
        pwrRight: number;
    }

    interface AccelMotor {
        pwr: number,
        isDone: boolean
    }

    interface LinearAccelMotors {
        pwr: number,
        isDone: boolean
    }

    interface AccelMotors {
        pwrLeft: number,
        pwrRight: number,
        isDone: boolean
    }

    /**
     * Configuration of synchronization of chassis motors at the required speed (power).
     * Конфигурация синхронизации моторов шассии на нужной скорости (мощности).
     * @param vLeft входное значение скорости левого мотора, eg: 50
     * @param vRight входное значение скорости правого мотора, eg: 50
     */
    //% blockId="SyncMotorsConfig"
    //% block="config sync сhassis control at vLeft = $vLeft vRight = $vRight"
    //% block.loc.ru="конфигурирация синхронизации управления шасси при vLeft = $vLeft vRight = $vRight"
    //% inlineInputMode="inline"
    //% vLeft.shadow="motorSpeedPicker"
    //% vRight.shadow="motorSpeedPicker"
    //% weight="99"
    //% group="Синхронизация шасси на скорости"
    export function syncMotorsConfig(vLeft: number, vRight: number) {
        syncVLeft = vLeft;
        syncVRight = vRight;
        syncVLeftSign = Math.abs(vLeft + 1) - Math.abs(vLeft);
        syncVRightSign = Math.abs(vRight + 1) - Math.abs(vRight);
    }

    /**
     * Calculate the synchronization error of the chassis motors using the values from the encoders.
     * Speed ​​(power) is constant.
     * Returns the error number for the controller.
     * Посчитать ошибку синхронизации моторов шассии с использованием значений с энкодеров.
     * Скорость (мощность) постоянная.
     * Возвращает число ошибки для регулятора.
     * @param eLeft входное значение энкодера левого мотора
     * @param eRight входное значение энкодера правого мотора
     */
    //% blockId="GetErrorSyncMotors"
    //% block="get error sync chassis at eLeft = $eLeft eRight = $eRight"
    //% block.loc.ru="получить ошибку синхронизации шасси при eLeft = $eLeft eRight = $eRight"
    //% inlineInputMode="inline"
    //% weight="98"
    //% group="Синхронизация шасси на скорости"
    export function getErrorSyncMotors(eLeft: number, eRight: number): number {
        return (syncVRight * eLeft) - (syncVLeft * eRight);
    }
    
    /**
     * To obtain the values of speeds (power) for the chassis motors based on the control action received from the sync regulator.
     * Returns the interface of the speed (power) of the left and right motors.
     * Получить значения скоростей (мощности) для моторов шассии на основе управляющего воздействия, полученного от регулятора синхронизации.
     * Возвращает интерфейс скорости (мощности) левого и правого моторов.
     * @param U входное значение управляющего воздействия от регулятора
     */
    //% blockId="GetPwrSyncMotors"
    //% block="get pwr sync сhassis at U = $U"
    //% block.loc.ru="получить скорости синхронизации шасси при U = $U"
    //% inlineInputMode="inline"
    //% weight="97"
    //% group="Синхронизация шасси на скорости"
    export function getPwrSyncMotors(U: number): MotorsPower {
        const pLeft = syncVLeft - syncVRightSign * U;
        const pRight = syncVRight + syncVLeftSign * U;
        return {
            pwrLeft: pLeft,
            pwrRight: pRight
        };
    }
    
    /**
     * Calculate the synchronization error of the chassis motors using the values from the encoders.
     * Returns the error number for the controller.
     * Посчитать ошибку синхронизации моторов шассии с использованием значений с энкодеров и с учётом необходимых скоростей (мощностей) для моторов.
     * Возвращает число ошибки для регулятора.
     * @param eLeft входное значение энкодера левого мотора
     * @param eRight входное значение энкодера правого мотора
     * @param vLeft входное значение скорости левого мотора, eg: 50
     * @param vRight входное значение скорости правого мотора, eg: 50
     */
    //% blockId="GetErrorSyncMotorsAtPwr"
    //% block="get error sync сhassis at eLeft = $eLeft eRight = $eRight vLeft = $vLeft vRight = $vRight"
    //% block.loc.ru="получить ошибку синхронизации шасси при eLeft = $eLeft eRight = $eRight vLeft = $vLeft vRight = $vRight"
    //% inlineInputMode="inline"
    //% weight="89"
    //% group="Синхронизация шасси на разных скоростях"
    export function getErrorSyncMotorsAtPwr(eLeft: number, eRight: number, vLeft: number, vRight: number): number {
        return (vRight * eLeft) - (vLeft * eRight);
    }

    /**
     * Get speeds (powers) for motors by U action of the regulator and the required speeds (powers).
     * Получить speeds (powers) для моторов по U воздействию регулятора и с необходимыми скоростями (мощностями).
     * @param U входное значение с регулятора
     * @param vLeft входное значение скорости левого мотора, eg: 50
     * @param vRight входное значение скорости правого мотора, eg: 50
     */
    //% blockId="GetPwrSyncMotorsAtPwr"
    //% block="get pwr sync сhassis at U = $U vLeft = $vLeft vRight = $vRight"
    //% block.loc.ru="получить скорости синхронизации шасси при U = $U vLeft = $vLeft vRight = $vRight"
    //% inlineInputMode="inline"
    //% weight="88"
    //% group="Синхронизация шасси на разных скоростях"
    export function getPwrSyncMotorsAtPwr(U: number, vLeft: number, vRight: number): MotorsPower {
        const pLeft = vLeft - (Math.abs(vRight + 1) - Math.abs(vRight)) * U;
        const pRight = vRight + (Math.abs(vLeft + 1) - Math.abs(vLeft)) * U;
        return {
            pwrLeft: pLeft,
            pwrRight: pRight
        };
    }

    /**
     * Acceleration and deceleration configuration of one motor.
     * Конфигурация ускорения и замедления одного мотора.
     * @param minPwr входное значение скорости (мозности) на старте, eg: 20
     * @param maxPwr входное значение максимальной скорости (мощности), eg: 50
     * @param accelDist значение дистанции ускорения, eg: 150
     * @param decelDist значение дистанции замедления, eg: 150
     * @param totalDist значение всей дистанции, eg: 500
     */
    //% blockId="AccOneEncConfig"
    //% block="config motor acceleration minPwr = $minPwr maxPwr = $maxPwr accelDist = $accelDist decelDist = $decelDist totalDist = $totalDist"
    //% block.loc.ru="конфигурация ускорения мотора minPwr = $minPwr maxPwr = $maxPwr accelDist = $accelDist decelDist = $decelDist totalDist = $totalDist"
    //% inlineInputMode="inline"
    //% minPwr.shadow="motorSpeedPicker"
    //% maxPwr.shadow="motorSpeedPicker"
    //% weight="79"
    //% group="Ускорение/замедлениие мотора"
    export function accOneEncConfig(minPwr: number, maxPwr: number, accelDist: number, decelDist: number, totalDist: number) {
        accMotorMinPwr = Math.abs(minPwr);
        accMotorMaxPwr = Math.abs(maxPwr);
        accMotorAccelDist = accelDist;
        accMotorDecelDist = decelDist;
        accMotorTotalDist = totalDist;

        if (minPwr <= 0 && maxPwr < 0) accMotorIsNeg = true;
        else accMotorIsNeg = false;
    }
    
    /**
     * Calculation of acceleration/deceleration for one motor.
     * Расчёт ускорения/замедления для одного мотора.
     * @param enc входное значение энкодера мотора
     */
    //% blockId="AccOneEnc"
    //% block="compute accel/deceleration motor at enc = $enc"
    //% block.loc.ru="расчитать ускорение/замедление управления мотора при enc = $enc"
    //% inlineInputMode="inline"
    //% weight="78"
    //% group="Ускорение/замедлениие мотора"
    export function accOneEnc(enc: number): AccelMotor {
        let done: boolean;
        let pwrOut: number;
        const currEnc = Math.abs(enc);
        if (currEnc >= accMotorTotalDist) done = true;
        else if (currEnc > accMotorTotalDist / 2) {
            if (accMotorDecelDist == 0) pwrOut = accMotorMaxPwr;
            else pwrOut = (accMotorMaxPwr - accMotorMinPwr) / Math.pow(accMotorDecelDist, 2) * Math.pow(currEnc - accMotorTotalDist, 2) + accMotorMinPwr;
            done = false;
        } else {
            if (accMotorAccelDist == 0) pwrOut = accMotorMaxPwr;
            else pwrOut = (accMotorMaxPwr - accMotorMinPwr) / Math.pow(accMotorAccelDist, 2) * Math.pow(currEnc - 0, 2) + accMotorMinPwr;
            done = false;
        }

        if (pwrOut < accMotorMinPwr) pwrOut = accMotorMinPwr;
        else if (pwrOut > accMotorMaxPwr) pwrOut = accMotorMaxPwr;

        if (accMotorIsNeg == true) pwrOut = -pwrOut;
        else pwrOut = pwrOut;

        return {
            pwr: pwrOut,
            isDone: done
        };
    }

    /**
     * The configuration of acceleration and deceleration of the chassis of two motors.
     * Конфигурация ускорения и замедления шассии двух моторов.
     * @param minStartPwr входное значение скорости на старте, eg: 20
     * @param maxPwr входное значение максимальной скорости, eg: 50
     * @param minEndPwr входное значение скорости при замедлении, eg: 20
     * @param accelDist значение дистанции ускорения, eg: 150
     * @param decelDist значение дистанции замедления, eg: 150
     * @param totalDist значение всей дистанции, eg: 500
     */
    //% blockId="AccTwoEncConfig"
    //% block="config accel/deceleration chassis at minStartPwr = $minStartPwr maxPwr = $maxPwr minEndPwr = $minEndPwr|totalDist = $totalDist accelDist = $accelDist decelDist = $decelDist"
    //% block.loc.ru="конфигурирация ускорения/замедления управления шасси при minStartPwr = $minStartPwr maxPwr = $maxPwr minEndPwr = $minEndPwr|totalDist = $totalDist accelDist = $accelDist decelDist = $decelDist"
    //% inlineInputMode="inline"
    //% minStartPwr.shadow="motorSpeedPicker"
    //% maxPwr.shadow="motorSpeedPicker"
    //% minEndPwr.shadow="motorSpeedPicker"
    //% weight="69"
    //% group="Синхронизация шасси с ускорением/замедлением"
    export function accTwoEncConfig(minStartPwr: number, maxPwr: number, minEndPwr: number, accelDist: number, decelDist: number, totalDist: number) {
        // accTwoEncMinPwr = Math.abs(minPwr);
        accMotorsStartPwr = Math.abs(minStartPwr);
        accMotorsMaxPwr = Math.abs(maxPwr);
        accMotorsEndPwr = Math.abs(minEndPwr);
        accMotorsAccelDist = Math.abs(accelDist);
        accMotorsDecelDist = Math.abs(decelDist);
        accMotorsTotalDist = Math.abs(totalDist);
        if (minStartPwr <= 0 && maxPwr < 0 && minEndPwr <= 0) accMotorsIsNeg = true; // if (minPwr <= 0 && maxPwr < 0) accMotorsIsNeg = 1;
        else accMotorsIsNeg = false;
    }

    /**
     * Calculation of acceleration/deceleration for two motors.
     * Расчёт ускорения/замедления для двух моторов.
     * @param eLeft входное значение энкодера левого мотора
     * @param eRight входное значение энкодера правого мотора
     */
    //% blockId="AccTwoEnc"
    //% block="compute accel/deceleration chassis at eLeft = $eLeft eRight = $eRight"
    //% block.loc.ru="расчитать ускорение/замедление управления шасси при eLeft = $eLeft eRight = $eRight"
    //% inlineInputMode="inline"
    //% weight="68"
    //% group="Синхронизация шасси с ускорением/замедлением"
    export function accTwoEnc(eLeft: number, eRight: number): LinearAccelMotors {
        let done: boolean;
        let pwrOut: number;
        const currEnc = (Math.abs(eLeft) + Math.abs(eRight)) / 2;
        if (currEnc >= accMotorsTotalDist) {
            pwrOut = 0;
            done = true;
        } else if (currEnc > accMotorsTotalDist / 2) {
            if (accMotorsDecelDist == 0) pwrOut = accMotorsMaxPwr;
            else pwrOut = (accMotorsMaxPwr - accMotorsEndPwr) / Math.pow(accMotorsDecelDist, 2) * Math.pow(currEnc - accMotorsTotalDist, 2) + accMotorsEndPwr; // pwr = (accMotorsMaxPwr - accTwoEncMinPwr) / Math.pow(accMotorsDecelDist, 2) * Math.pow(currEnc - accMotorsTotalDist, 2) + accTwoEncMinPwr;
            done = false;
        } else {
            if (accMotorsAccelDist == 0) pwrOut = accMotorsMaxPwr;
            else pwrOut = (accMotorsMaxPwr - accMotorsStartPwr) / Math.pow(accMotorsAccelDist, 2) * Math.pow(currEnc - 0, 2) + accMotorsStartPwr; // pwr = (accMotorsMaxPwr - accTwoEncMinPwr) / Math.pow(accMotorsAccelDist, 2) * Math.pow(currEnc - 0, 2) + accTwoEncMinPwr;
            done = false;
        }

        // if (pwr < accTwoEncMinPwr) pwr = accTwoEncMinPwr;
        // else if (pwr > accMotorsMaxPwr) pwr = accMotorsMaxPwr;
        if (currEnc > accMotorsTotalDist / 2 && pwrOut < accMotorsEndPwr) pwrOut = accMotorsEndPwr;
        else if (pwrOut < accMotorsStartPwr) pwrOut = accMotorsStartPwr;
        else if (pwrOut > accMotorsMaxPwr) pwrOut = accMotorsMaxPwr;

        if (accMotorsIsNeg) pwrOut = -pwrOut;

        return {
            pwr: pwrOut,
            isDone: done
        };
    }

    // Глобальные переменные для конфигурации
    let accEncTotalDists = { left: 0, right: 0 };
    let accEncAccelDists = { left: 0, right: 0 };
    let accEncDecelDists = { left: 0, right: 0 };
    let accEncStartingPwrs = { left: 0, right: 0 };
    let accEncMaxPwrs = { left: 0, right: 0 };
    let accEncFinishingPwrs = { left: 0, right: 0 };
    let accEncIsNeg = { left: false, right: false };

    /**
     * The acceleration and deceleration configuration of the chassis of two motors with different maximum speeds.
     * Конфигурация ускорения и замедления шасси двух моторов с разными максимальными скоростями.
     * @param startingPwr входное значение скорости на старте, eg: 20
     * @param maxPwrLeft входное значение максимальной скорости левого мотора, eg: 50
     * @param maxPwrRight входное значение максимальной скорости правого мотора, eg: 75
     * @param finishingPwr входное значение скорости при замедлении, eg: 20
     * @param accelDist значение дистанции ускорения, eg: 150
     * @param decelDist значение дистанции замедления, eg: 150
     * @param totalDist значение всей дистанции, eg: 500
     */
    //% blockId="AccTwoEncExtConfig"
    //% block="config accel/deceleration chassis at startingPwr = $startingPwr maxPwrLeft = $maxPwrLeft maxPwrRight = $maxPwrRight finishingPwr = $finishingPwr|totalDist = $totalDist accelDist = $accelDist decelDist = $decelDist"
    //% block.loc.ru="конфигурирация ускорения/замедления управления шасси при startingPwr = $startingPwr maxPwrLeft = $maxPwrLeft maxPwrRight = $maxPwrRight finishingPwr = $finishingPwr|totalDist = $totalDist accelDist = $accelDist decelDist = $decelDist"
    //% inlineInputMode="inline"
    //% startingPwr.shadow="motorSpeedPicker"
    //% maxPwrLeft.shadow="motorSpeedPicker"
    //% maxPwrRight.shadow="motorSpeedPicker"
    //% finishingPwr.shadow="motorSpeedPicker"
    //% weight="59"
    //% group="Синхронизация шасси с ускорением/замедлением"
    export function accTwoEncExtConfig(startingPwrLeft: number, startingPwrRight: number, maxPwrLeft: number, maxPwrRight: number, finishingPwrLeft: number, finishingPwrRight: number, accelDistCenter: number, decelDistCenter: number, totalDistCenter: number) {
        // Радиус поворота центра
        const vLeft = Math.abs(maxPwrLeft), vRight = Math.abs(maxPwrRight);
        const radius = vLeft !== vRight ? (chassis.getBaseLength() * (vLeft + vRight)) / (2 * (vRight - vLeft)) : Infinity;

        // Коэффициенты расстояния для колёс
        const kLeft = radius !== Infinity ? (radius - chassis.getBaseLength() / 2) / radius : 1;
        const kRight = radius !== Infinity ? (radius + chassis.getBaseLength() / 2) / radius : 1;

        // Дистанции
        accEncAccelDists.left = accelDistCenter * kLeft;
        accEncAccelDists.right = accelDistCenter * kRight;
        accEncDecelDists.left = decelDistCenter * kLeft;
        accEncDecelDists.right = decelDistCenter * kRight;
        accEncTotalDists.left = totalDistCenter * kLeft;
        accEncTotalDists.right = totalDistCenter * kRight;

        console.log(`kLeft: ${kLeft}, kRight: ${kRight}`);
        console.log(`accEncAccelDists.l: ${accEncAccelDists.left}, accEncAccelDists.r: ${accEncAccelDists.right}`);
        console.log(`accEncDecelDists.l: ${accEncDecelDists.left}, accEncDecelDists.r: ${accEncDecelDists.right}`);
        console.log(`accEncTotalDists.l: ${accEncTotalDists.left}, accEncTotalDists.r: ${accEncTotalDists.right}`);

        // Мощности
        accEncStartingPwrs.left = Math.constrain(startingPwrLeft, -100, 100);
        accEncStartingPwrs.right = Math.constrain(startingPwrRight, -100, 100);
        accEncMaxPwrs.left = Math.constrain(maxPwrLeft, -100, 100);
        accEncMaxPwrs.right = Math.constrain(maxPwrRight, -100, 100);
        accEncFinishingPwrs.left = Math.constrain(finishingPwrLeft, -100, 100);
        accEncFinishingPwrs.right = Math.constrain(finishingPwrRight, -100, 100);

        accEncIsNeg.left = startingPwrLeft < 0 && maxPwrLeft < 0 && finishingPwrLeft < 0;
        accEncIsNeg.right = startingPwrRight < 0 && maxPwrRight < 0 && finishingPwrRight < 0;
    }

    /**
     * Acceleration/deceleration calculation for two motors with different maximum speeds.
     * Расчёт ускорения/замедления для двух моторов с разными максимальными скоростями.
     * @param eLeft входное значение энкодера левого мотора
     * @param eRight входное значение энкодера правого мотора
     */
    //% blockId="AccTwoEncExt"
    //% block="compute accel/deceleration chassis at with different max speeds eLeft = $eLeft eRight = $eRight"
    //% block.loc.ru="расчитать ускорение/замедление управления шасси с разными макс скоростями при eLeft = $eLeft eRight = $eRight"
    //% inlineInputMode="inline"
    //% weight="58"
    //% group="Синхронизация шасси с ускорением/замедлением"
    export function accTwoEncExt(eLeft: number, eRight: number): AccelMotors {
        // Расчёт мощности левого мотора
        const pwrLeft = computePower(Math.abs(eLeft), accEncTotalDists.left, accEncAccelDists.left, accEncDecelDists.left, accEncStartingPwrs.left, accEncMaxPwrs.left, accEncFinishingPwrs.left, accEncIsNeg.left);
        // Расчёт мощности правого мотора
        const pwrRight = computePower(Math.abs(eRight), accEncTotalDists.right, accEncAccelDists.right, accEncDecelDists.right, accEncStartingPwrs.right, accEncMaxPwrs.right, accEncFinishingPwrs.right, accEncIsNeg.right);

        const done = Math.abs(eLeft) >= accEncTotalDists.left && Math.abs(eRight) >= accEncTotalDists.right;

        return {
            pwrLeft: pwrLeft,
            pwrRight: pwrRight,
            isDone: done
        };
    }

    function computePower(currEnc: number, totalDist: number, accelDist: number, decelDist: number, startPwr: number, maxPwr: number, endPwr: number, isNeg: boolean) {
        let pwr: number;
        let done = false;

        if (currEnc >= totalDist) {
            pwr = 0;
            done = true;
        } else if (currEnc > totalDist / 2) {
            if (decelDist == 0) pwr = maxPwr;
            else pwr = (maxPwr - endPwr) / Math.pow(decelDist, 2) * Math.pow(currEnc - totalDist, 2) + endPwr;
            done = false;
        } else {
            if (accelDist == 0) pwr = maxPwr;
            else pwr = (maxPwr - startPwr) / Math.pow(accelDist, 2) * Math.pow(currEnc, 2) + startPwr;
            done = false;
        }

        pwr = Math.min(Math.abs(pwr), Math.abs(maxPwr));
        if (Math.abs(pwr) < Math.abs(endPwr)) pwr = endPwr;

        return isNeg ? -pwr : pwr;
    }

}