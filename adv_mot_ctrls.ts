/**
 * Motor controllers based OFDL Advanced Motor Controller Block module (algorithm part).
 * Based 1.1 ver, 2023/09/27.
 * https://github.com/ofdl-robotics-tw/EV3-CLEV3R-Modules/blob/main/Mods/AdvMtCtrls.bpm
 */
//% block="AdvMotCtrls" weight="50" color="#02ab38" icon="\uf3fd" advanced="true"
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

    let accMotorsStartPwr: number;
    let accMotorsMaxPwr: number;
    let accMotorsEndPwr: number;
    let accMotorsAccelDist: number;
    let accMotorsDecelDist: number;
    let accMotorsTotalDist: number;
    let accMotorsIsNeg: boolean;

    let accMotorsTotalDistsExt = { left: 0, right: 0 };
    let accMotorsAccelDistsExt = { left: 0, right: 0 };
    let accMotorsDecelDistsExt = { left: 0, right: 0 };
    let accMotorsStartingPwrsExt = { left: 0, right: 0 };
    let accMotorsMaxPwrsExt = { left: 0, right: 0 };
    let accMotorsFinishingPwrsExt = { left: 0, right: 0 };
    let accMotorsIsNegExt = { left: false, right: false };

    interface MotorsPower {
        pwrLeft: number;
        pwrRight: number;
    }

    interface AccelMotor {
        pwr: number,
        isDone: boolean
    }

    interface AccelMotors {
        pwrLeft: number,
        pwrRight: number,
        isDone: boolean
    }

    /**
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
    //% blockHidden="true"
    export function syncMotorsConfig(vLeft: number, vRight: number) {
        syncVLeft = vLeft;
        syncVRight = vRight;
        syncVLeftSign = Math.abs(vLeft + 1) - Math.abs(vLeft);
        syncVRightSign = Math.abs(vRight + 1) - Math.abs(vRight);
    }

    /**
     * Посчитать ошибку синхронизации моторов шассии с использованием значений с энкодеров.
     * Скорость (мощность) постоянная.
     * Возвращает число ошибки для регулятора.
     * @param eLeft входное значение энкодера левого мотора, eg: 0
     * @param eRight входное значение энкодера правого мотора, eg: 0
     */
    //% blockId="GetErrorSyncMotors"
    //% block="get error sync chassis at eLeft = $eLeft eRight = $eRight"
    //% block.loc.ru="получить ошибку синхронизации шасси при eLeft = $eLeft eRight = $eRight"
    //% inlineInputMode="inline"
    //% weight="98"
    //% group="Синхронизация шасси на скорости"
    //% blockHidden="true"
    export function getErrorSyncMotors(eLeft: number, eRight: number): number {
        return (syncVRight * eLeft) - (syncVLeft * eRight);
    }
    
    /**
     * Получить значения скоростей (мощности) для моторов шассии на основе управляющего воздействия, полученного от регулятора синхронизации.
     * Возвращает интерфейс скорости (мощности) левого и правого моторов.
     * @param u входное значение управляющего воздействия от регулятора, eg: 0
     */
    //% blockId="GetPwrSyncMotors"
    //% block="get pwr sync сhassis at u = $u"
    //% block.loc.ru="получить скорости синхронизации шасси при u = $u"
    //% inlineInputMode="inline"
    //% weight="97"
    //% group="Синхронизация шасси на скорости"
    //% blockHidden="true"
    export function getPwrSyncMotors(u: number): MotorsPower {
        const pLeft = syncVLeft - syncVRightSign * u;
        const pRight = syncVRight + syncVLeftSign * u;
        return {
            pwrLeft: pLeft,
            pwrRight: pRight
        };
    }
    
    /**
     * Посчитать ошибку синхронизации моторов шассии с использованием значений с энкодеров и с учётом необходимых скоростей (мощностей) для моторов.
     * Возвращает число ошибки для регулятора.
     * @param eLeft входное значение энкодера левого мотора, eg: 0
     * @param eRight входное значение энкодера правого мотора, eg: 0
     * @param vLeft входное значение скорости (мощности) левого мотора, eg: 50
     * @param vRight входное значение скорости (мощности) правого мотора, eg: 50
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
     * Получить скорости (мощности) для моторов по u воздействию регулятора и с необходимыми скоростями (мощностями).
     * @param u входное значение с регулятора, eg: 0
     * @param vLeft входное значение скорости (мощности) левого мотора, eg: 50
     * @param vRight входное значение скорости (мощности) правого мотора, eg: 50
     */
    //% blockId="GetPwrSyncMotorsAtPwr"
    //% block="get pwr sync сhassis at u = $u vLeft = $vLeft vRight = $vRight"
    //% block.loc.ru="получить скорости синхронизации шасси при u = $u vLeft = $vLeft vRight = $vRight"
    //% inlineInputMode="inline"
    //% weight="88"
    //% group="Синхронизация шасси на разных скоростях"
    export function getPwrSyncMotorsAtPwr(u: number, vLeft: number, vRight: number): MotorsPower {
        const pLeft = vLeft - (Math.abs(vRight + 1) - Math.abs(vRight)) * u;
        const pRight = vRight + (Math.abs(vLeft + 1) - Math.abs(vLeft)) * u;
        return {
            pwrLeft: pLeft,
            pwrRight: pRight
        };
    }

    /**
     * Конфигурация ускорения и замедления одного мотора.
     * @param minPwr входное значение скорости (мощности) на старте, eg: 20
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
        accMotorIsNeg = minPwr <= 0 && maxPwr < 0;
    }
    
    /**
     * Расчёт ускорения/замедления для одного мотора.
     * @param enc входное значение энкодера мотора, eg: 0
     */
    //% blockId="AccOneEncCompute"
    //% block="compute accel/deceleration motor at enc = $enc"
    //% block.loc.ru="расчитать ускорение/замедление управления мотора при enc = $enc"
    //% inlineInputMode="inline"
    //% weight="78"
    //% group="Ускорение/замедлениие мотора"
    export function accOneEncCompute(enc: number): AccelMotor {
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
     * Конфигурация ускорения и замедления шассии двух моторов.
     * @param minStartPwr входное значение скорости (мощности) на старте, eg: 20
     * @param maxPwr входное значение максимальной скорости (мощности), eg: 50
     * @param minEndPwr входное значение скорости (мощности) при замедлении, eg: 20
     * @param accelDist значение дистанции ускорения, eg: 150
     * @param decelDist значение дистанции замедления, eg: 150
     * @param totalDist значение всей дистанции, eg: 500
     */
    //% blockId="AccTwoEncLinearMotionConfig"
    //% block="config accel/deceleration chassis at minStartPwr = $minStartPwr maxPwr = $maxPwr minEndPwr = $minEndPwr|totalDist = $totalDist accelDist = $accelDist decelDist = $decelDist"
    //% block.loc.ru="конфигурирация ускорения/замедления управления шасси при minStartPwr = $minStartPwr maxPwr = $maxPwr minEndPwr = $minEndPwr|totalDist = $totalDist accelDist = $accelDist decelDist = $decelDist"
    //% inlineInputMode="inline"
    //% minStartPwr.shadow="motorSpeedPicker"
    //% maxPwr.shadow="motorSpeedPicker"
    //% minEndPwr.shadow="motorSpeedPicker"
    //% weight="69"
    //% group="Синхронизация шасси с ускорением/замедлением"
    export function accTwoEncLinearMotionConfig(minStartPwr: number, maxPwr: number, minEndPwr: number, accelDist: number, decelDist: number, totalDist: number) {
        accMotorsStartPwr = Math.abs(minStartPwr);
        accMotorsMaxPwr = Math.abs(maxPwr);
        accMotorsEndPwr = Math.abs(minEndPwr);
        accMotorsAccelDist = Math.abs(accelDist);
        accMotorsDecelDist = Math.abs(decelDist);
        accMotorsTotalDist = Math.abs(totalDist);
        accMotorsIsNeg = minStartPwr <= 0 && maxPwr < 0 && minEndPwr <= 0;
    }

    /**
     * Расчёт ускорения/замедления для двух моторов.
     * @param eLeft входное значение энкодера левого мотора, eg: 0
     * @param eRight входное значение энкодера правого мотора, eg: 0
     */
    //% blockId="AccTwoEncLinearMotionCompute"
    //% block="compute accel/deceleration chassis at eLeft = $eLeft eRight = $eRight"
    //% block.loc.ru="расчитать ускорение/замедление управления шасси при eLeft = $eLeft eRight = $eRight"
    //% inlineInputMode="inline"
    //% weight="68"
    //% group="Синхронизация шасси с ускорением/замедлением"
    export function accTwoEncLinearMotionCompute(eLeft: number, eRight: number): AccelMotor {
        let done: boolean = false;
        let pwrOut: number;
        const currEnc = (Math.abs(eLeft) + Math.abs(eRight)) / 2;

        if (currEnc >= accMotorsTotalDist) { // Фаза финиша
            pwrOut = 0;
            done = true;
        } else if (currEnc > accMotorsTotalDist - accMotorsDecelDist) { // Фаза замедления
            if (accMotorsDecelDist == 0) pwrOut = accMotorsMaxPwr;
            else pwrOut = (accMotorsMaxPwr - accMotorsEndPwr) / Math.pow(accMotorsDecelDist, 2) * Math.pow(currEnc - accMotorsTotalDist, 2) + accMotorsEndPwr;
            pwrOut = Math.max(accMotorsEndPwr, pwrOut); // Защита от "проседания" ниже EndPwr (мало ли, плавающая точка)
        } else if (currEnc < accMotorsAccelDist) { // Фаза ускорения
            if (accMotorsAccelDist == 0) pwrOut = accMotorsMaxPwr;
            else pwrOut = (accMotorsMaxPwr - accMotorsStartPwr) / Math.pow(accMotorsAccelDist, 2) * Math.pow(currEnc, 2) + accMotorsStartPwr;
            pwrOut = Math.max(accMotorsStartPwr, pwrOut); // Защита от старта ниже StartPwr
        } else { // Фаза постоянной скорости (между ускорением и замедлением)
            pwrOut = accMotorsMaxPwr;
        }

        pwrOut = Math.min(accMotorsMaxPwr, pwrOut); // Финальный ограничитель сверху (на всякий случай для всех фаз)

        if (accMotorsIsNeg) pwrOut = -pwrOut; // Инверсия знака

        return {
            pwr: pwrOut,
            isDone: done
        };
    }

    /**
     * Конфигурация ускорения и замедления шасси двух моторов с разными максимальными скоростями.
     * @param startingPwr входное значение скорости (мощности) моторов на старте, eg: 20
     * @param maxPwrLeft входное значение максимальной скорости (мощности) левого мотора, eg: 50
     * @param maxPwrRight входное значение максимальной скорости (мощности) правого мотора, eg: 75
     * @param finishingPwr входное значение скорости (мощности) моторов при замедлении, eg: 20
     * @param accelDistCenter значение дистанции ускорения, eg: 150
     * @param decelDistCenter значение дистанции замедления, eg: 150
     * @param totalDistCenter значение всей дистанции, eg: 500
     */
    //% blockId="AccTwoEncComplexMotionConfig"
    //% block="config accel/deceleration chassis at startingPwr = $startingPwr maxPwrLeft = $maxPwrLeft maxPwrRight = $maxPwrRight finishingPwr = $finishingPwr|totalDist = $totalDist accelDist = $accelDist decelDist = $decelDist"
    //% block.loc.ru="конфигурирация ускорения/замедления управления шасси при startingPwr = $startingPwr maxPwrLeft = $maxPwrLeft maxPwrRight = $maxPwrRight finishingPwr = $finishingPwr|totalDist = $totalDist accelDist = $accelDist decelDist = $decelDist"
    //% inlineInputMode="inline"
    //% startingPwr.shadow="motorSpeedPicker"
    //% maxPwrLeft.shadow="motorSpeedPicker"
    //% maxPwrRight.shadow="motorSpeedPicker"
    //% finishingPwr.shadow="motorSpeedPicker"
    //% weight="59"
    //% group="Синхронизация шасси с ускорением/замедлением"
    export function accTwoEncComplexMotionConfig(startingPwr: number, maxPwrLeft: number, maxPwrRight: number, finishingPwr: number, accelDistCenter: number, decelDistCenter: number, totalDistCenter: number) {
        // Определяем, какой мотор медленнее (с меньшей максимальной мощностью)
        const absMaxLeft = Math.abs(maxPwrLeft);
        const absMaxRight = Math.abs(maxPwrRight);

        // Коэффициент пропорциональности (отношение макс. скоростей)
        const ratio = absMaxLeft < absMaxRight ? absMaxRight / absMaxLeft : absMaxLeft / absMaxRight;

        if (absMaxLeft < absMaxRight) { // Левый мотор медленнее - он получает базовые значения
            accMotorsStartingPwrsExt.left = startingPwr;
            accMotorsStartingPwrsExt.right = startingPwr * ratio;
            accMotorsFinishingPwrsExt.left = finishingPwr;
            accMotorsFinishingPwrsExt.right = finishingPwr * ratio;

            accMotorsTotalDistsExt.left = totalDistCenter;
            accMotorsTotalDistsExt.right = totalDistCenter * ratio;
            accMotorsAccelDistsExt.left = accelDistCenter;
            accMotorsAccelDistsExt.right = accelDistCenter * ratio;
            accMotorsDecelDistsExt.left = decelDistCenter;
            accMotorsDecelDistsExt.right = decelDistCenter * ratio;
        } else { // Правый мотор медленнее - он получает базовые значения
            accMotorsStartingPwrsExt.left = startingPwr * ratio;
            accMotorsStartingPwrsExt.right = startingPwr;
            accMotorsFinishingPwrsExt.left = finishingPwr * ratio;
            accMotorsFinishingPwrsExt.right = finishingPwr;

            accMotorsTotalDistsExt.left = totalDistCenter * ratio;
            accMotorsTotalDistsExt.right = totalDistCenter;
            accMotorsAccelDistsExt.left = accelDistCenter * ratio;
            accMotorsAccelDistsExt.right = accelDistCenter;
            accMotorsDecelDistsExt.left = decelDistCenter * ratio;
            accMotorsDecelDistsExt.right = decelDistCenter;
        }
        accMotorsMaxPwrsExt.left = maxPwrLeft;
        accMotorsMaxPwrsExt.right = maxPwrRight;

        accMotorsIsNegExt.left = startingPwr < 0 && maxPwrLeft < 0 && finishingPwr < 0;
        accMotorsIsNegExt.right = startingPwr < 0 && maxPwrRight < 0 && finishingPwr < 0;
    }

    /**
     * Расчёт ускорения/замедления для двух моторов с разными максимальными скоростями (мощностями).
     * @param eLeft входное значение энкодера левого мотора, eg: 0
     * @param eRight входное значение энкодера правого мотора, eg: 0
     */
    //% blockId="AccTwoEncComplexMotionCompute"
    //% block="compute accel/deceleration chassis at with different max speeds eLeft = $eLeft eRight = $eRight"
    //% block.loc.ru="расчитать ускорение/замедление управления шасси с разными макс скоростями при eLeft = $eLeft eRight = $eRight"
    //% inlineInputMode="inline"
    //% weight="58"
    //% group="Синхронизация шасси с ускорением/замедлением"
    export function accTwoEncComplexMotionCompute(eLeft: number, eRight: number): AccelMotors {
        const profileLeft = accTwoEncComputeMotorProfile(
            Math.abs(eLeft),
            accMotorsTotalDistsExt.left, accMotorsAccelDistsExt.left, accMotorsDecelDistsExt.left,
            accMotorsStartingPwrsExt.left, accMotorsMaxPwrsExt.left, accMotorsFinishingPwrsExt.left,
            accMotorsIsNegExt.left
        ); // Расчёт мощности левого мотора
        const profileRight = accTwoEncComputeMotorProfile(
            Math.abs(eRight),
            accMotorsTotalDistsExt.right, accMotorsAccelDistsExt.right, accMotorsDecelDistsExt.right,
            accMotorsStartingPwrsExt.right, accMotorsMaxPwrsExt.right, accMotorsFinishingPwrsExt.right,
            accMotorsIsNegExt.right
        ); // Расчёт мощности правого мотора

        return {
            pwrLeft: profileLeft.pwr,
            pwrRight: profileRight.pwr,
            isDone: profileLeft.isDone && profileRight.isDone
        };
    }

    // Расчёт профиля скорости (мощности) мотора 
    function accTwoEncComputeMotorProfile(currEnc: number, totalDist: number, accelDist: number, decelDist: number, startPwr: number, maxPwr: number, endPwr: number, isNeg: boolean): AccelMotor {
        let done: boolean = false;
        let pwr: number;

        const absStartPwr = Math.abs(startPwr);
        const absMaxPwr = Math.abs(maxPwr);
        const absEndPwr = Math.abs(endPwr);

        if (currEnc >= totalDist) { // Фаза финиша
            pwr = 0;
            done = true;
        } else if (currEnc > totalDist - decelDist) { // Фаза замедления
            if (decelDist == 0) pwr = absMaxPwr;
            else pwr = (absMaxPwr - absEndPwr) / Math.pow(decelDist, 2) * Math.pow(currEnc - totalDist, 2) + absEndPwr;
            pwr = Math.max(absEndPwr, Math.min(absMaxPwr, pwr)); // Ограничение для фазы замедления
        } else if (currEnc < accelDist) { // Фаза ускорения
            if (accelDist == 0) pwr = absMaxPwr;
            else pwr = (absMaxPwr - absStartPwr) / Math.pow(accelDist, 2) * Math.pow(currEnc, 2) + absStartPwr;
            pwr = Math.max(absStartPwr, Math.min(absMaxPwr, pwr)); // Ограничение для фазы ускорения
        } else { // Фаза постоянной скорости (между ускорением и замедлением)
            pwr = absMaxPwr;
        }

        return {
            pwr: isNeg ? -pwr : pwr,
            isDone: done
        };
    }

}