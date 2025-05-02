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

    let accOneEncMinPwr: number;
    let accOneEncMaxPwr: number;
    let accOneEncAccelDist: number;
    let accOneEncDecelDist: number;
    let accOneEncTotalDist: number;
    let accOneEncIsNeg: number;

    // let accTwoEncMinPwr: number;
    let accTwoEncMinStartPwr: number;
    let accTwoEncMaxPwr: number;
    let accTwoEncMinEndPwr: number;
    let accTwoEncAccelDist: number;
    let accTwoEncDecelDist: number;
    let accTwoEncTotalDist: number;
    let accTwoEncIsNeg: boolean;

    let accTwoEncStartingPwr: number;
    let accTwoEncMaxPwrLeft: number;
    let accTwoEncMaxPwrRight: number;
    let accTwoEncFinishingPwr: number;
    let accTwoEncIsNegLeft: boolean;
    let accTwoEncIsNegRight: boolean;
    let accTwoEncRadius: number;
    let accTwoEncKLeft: number;
    let accTwoEncKRight: number;

    interface MotorsPower {
        pwrLeft: number;
        pwrRight: number;
    }

    interface AccEncReturn {
        pwrLeft: number;
        pwrRight: number;
        isDone: boolean;
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
        accOneEncMinPwr = Math.abs(minPwr);
        accOneEncMaxPwr = Math.abs(maxPwr);
        accOneEncAccelDist = accelDist;
        accOneEncDecelDist = decelDist;
        accOneEncTotalDist = totalDist;

        if (minPwr <= 0 && maxPwr < 0) accOneEncIsNeg = 1;
        else accOneEncIsNeg = 0;
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
    export function accOneEnc(enc: number): AccEncReturn {
        let done: boolean;
        let pwr: number, pwrOut: number;
        const currEnc = Math.abs(enc);
        if (currEnc >= accOneEncTotalDist) done = true;
        else if (currEnc > accOneEncTotalDist / 2) {
            if (accOneEncDecelDist == 0) pwr = accOneEncMaxPwr;
            else pwr = (accOneEncMaxPwr - accOneEncMinPwr) / Math.pow(accOneEncDecelDist, 2) * Math.pow(currEnc - accOneEncTotalDist, 2) + accOneEncMinPwr;
            done = false;
        } else {
            if (accOneEncAccelDist == 0) pwr = accOneEncMaxPwr;
            else pwr = (accOneEncMaxPwr - accOneEncMinPwr) / Math.pow(accOneEncAccelDist, 2) * Math.pow(currEnc - 0, 2) + accOneEncMinPwr;
            done = false;
        }

        if (pwr < accOneEncMinPwr) pwr = accOneEncMinPwr;
        else if (pwr > accOneEncMaxPwr) pwr = accOneEncMaxPwr;

        if (accOneEncIsNeg == 1) pwrOut = 0 - pwr;
        else pwrOut = pwr;

        return {
            pwrLeft: pwrOut,
            pwrRight: pwrOut,
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
        accTwoEncMinStartPwr = Math.abs(minStartPwr);
        accTwoEncMaxPwr = Math.abs(maxPwr);
        accTwoEncMinEndPwr = Math.abs(minEndPwr);
        accTwoEncAccelDist = Math.abs(accelDist);
        accTwoEncDecelDist = Math.abs(decelDist);
        accTwoEncTotalDist = Math.abs(totalDist);
        if (minStartPwr <= 0 && maxPwr < 0 && minEndPwr <= 0) accTwoEncIsNeg = true; // if (minPwr <= 0 && maxPwr < 0) accTwoEncIsNeg = 1;
        else accTwoEncIsNeg = false;
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
    export function accTwoEnc(eLeft: number, eRight: number): AccEncReturn {
        let done: boolean;
        let pwr: number;
        const currEnc = (Math.abs(eLeft) + Math.abs(eRight)) / 2;
        if (currEnc >= accTwoEncTotalDist) done = true;
        else if (currEnc > accTwoEncTotalDist / 2) {
            if (accTwoEncDecelDist == 0) pwr = accTwoEncMaxPwr;
            else pwr = (accTwoEncMaxPwr - accTwoEncMinEndPwr) / Math.pow(accTwoEncDecelDist, 2) * Math.pow(currEnc - accTwoEncTotalDist, 2) + accTwoEncMinEndPwr; // pwr = (accTwoEncMaxPwr - accTwoEncMinPwr) / Math.pow(accTwoEncDecelDist, 2) * Math.pow(currEnc - accTwoEncTotalDist, 2) + accTwoEncMinPwr;
            done = false;
        } else {
            if (accTwoEncAccelDist == 0) pwr = accTwoEncMaxPwr;
            else pwr = (accTwoEncMaxPwr - accTwoEncMinStartPwr) / Math.pow(accTwoEncAccelDist, 2) * Math.pow(currEnc - 0, 2) + accTwoEncMinStartPwr; // pwr = (accTwoEncMaxPwr - accTwoEncMinPwr) / Math.pow(accTwoEncAccelDist, 2) * Math.pow(currEnc - 0, 2) + accTwoEncMinPwr;
            done = false;
        }

        // if (pwr < accTwoEncMinPwr) pwr = accTwoEncMinPwr;
        // else if (pwr > accTwoEncMaxPwr) pwr = accTwoEncMaxPwr;
        if (currEnc > accTwoEncTotalDist / 2 && pwr < accTwoEncMinEndPwr) pwr = accTwoEncMinEndPwr;
        else if (pwr < accTwoEncMinStartPwr) pwr = accTwoEncMinStartPwr;
        else if (pwr > accTwoEncMaxPwr) pwr = accTwoEncMaxPwr;

        if (accTwoEncIsNeg) pwr = -pwr;

        return {
            pwrLeft: pwr,
            pwrRight: pwr,
            isDone: done
        };
    }

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
    export function accTwoEncExtConfig(startingPwr: number, maxPwrLeft: number, maxPwrRight: number, finishingPwr: number, accelDist: number, decelDist: number, totalDist: number) {
        accTwoEncStartingPwr = Math.abs(startingPwr);
        accTwoEncMaxPwrLeft = Math.abs(Math.constrain(maxPwrLeft, -100, 100));
        accTwoEncMaxPwrRight = Math.abs(Math.constrain(maxPwrRight, -100, 100));
        accTwoEncMaxPwr = (accTwoEncMaxPwrLeft + accTwoEncMaxPwrRight) / 2;
        accTwoEncFinishingPwr = Math.abs(finishingPwr);
        const vLeft = Math.abs(maxPwrLeft), vRight = Math.abs(maxPwrRight);
        if (vLeft !== vRight) accTwoEncRadius = (chassis.getBaseLength() * (vLeft + vRight)) / (2 * (vLeft - vRight));
        else accTwoEncRadius = Infinity; // Прямолинейное движение

        accTwoEncKLeft = accTwoEncRadius !== 0 ? (accTwoEncRadius + chassis.getBaseLength() / 2) / accTwoEncRadius : 1;
        accTwoEncKRight = accTwoEncRadius !== 0 ? (accTwoEncRadius - chassis.getBaseLength() / 2) / accTwoEncRadius : 1;

        accTwoEncAccelDist = Math.abs(accelDist);
        accTwoEncDecelDist = Math.abs(decelDist);
        accTwoEncTotalDist = Math.abs(totalDist);
        accTwoEncIsNegLeft = (startingPwr <= 0 && maxPwrLeft < 0 && finishingPwr <= 0);
        accTwoEncIsNegRight = (startingPwr <= 0 && maxPwrRight < 0 && finishingPwr <= 0);
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
    export function accTwoEncExt(eLeft: number, eRight: number): AccEncReturn {
        let done: boolean;
        let pwr: number;
        const currEnc = (Math.abs(eLeft) / accTwoEncKLeft + Math.abs(eRight) / accTwoEncKRight) / 2; // Приводим к пройденной дистанции центра
        if (currEnc >= accTwoEncTotalDist) done = true;
        else if (currEnc > accTwoEncTotalDist / 2) {
            if (accTwoEncDecelDist == 0) pwr = accTwoEncMaxPwr;
            else pwr = (accTwoEncMaxPwr - accTwoEncFinishingPwr) / Math.pow(accTwoEncDecelDist, 2) * Math.pow(currEnc - accTwoEncTotalDist, 2) + accTwoEncFinishingPwr;
            done = false;
        } else {
            if (accTwoEncAccelDist == 0) pwr = accTwoEncMaxPwr;
            else pwr = (accTwoEncMaxPwr - accTwoEncStartingPwr) / Math.pow(accTwoEncAccelDist, 2) * Math.pow(currEnc - 0, 2) + accTwoEncStartingPwr;
            done = false;
        }

        // if (currEnc > accTwoEncTotalDist / 2 && pwr < accTwoEncMinEndPwr) pwr = accTwoEncMinEndPwr;
        // else if (pwr < accTwoEncMinStartPwr) pwr = accTwoEncMinStartPwr;
        // else if (pwr > accTwoEncMaxPwr) pwr = accTwoEncMaxPwr;

        // Получаем мощности для моторов через коэффициенты
        let pwrLeft = pwr * accTwoEncKLeft;
        let pwrRight = pwr * accTwoEncKRight;

        // Ограничиваем мощности
        pwrLeft = Math.min(pwrLeft, accTwoEncMaxPwrLeft);
        pwrRight = Math.min(pwrRight, accTwoEncMaxPwrRight);

        // if (pwrLeft < accTwoEncFinishingPwr) pwrLeft = accTwoEncFinishingPwr;
        // if (pwrRight < accTwoEncFinishingPwr) pwrRight = accTwoEncFinishingPwr;

        // Применяем направление
        if (accTwoEncIsNegLeft) pwrLeft = -pwrLeft;
        if (accTwoEncIsNegRight) pwrRight = -pwrRight;

        return {
            pwrLeft,
            pwrRight,
            isDone: done
        };
    }

}