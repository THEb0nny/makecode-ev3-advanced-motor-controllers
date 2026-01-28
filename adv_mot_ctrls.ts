/**
 * Motor controllers based OFDL Advanced Motor Controller Block module (algorithm part).
 * Based 1.1 ver, 2023/09/27.
 * https://github.com/ofdl-robotics-tw/EV3-CLEV3R-Modules/blob/main/Mods/AdvMtCtrls.bpm
 */
//% block="AdvMotCtrls" weight="50" color="#02ab38" icon="\uf3fd" advanced="true"
namespace advmotctrls {
    // Используется математика параболистического профиля ускорения моторов

    let accMotorMinPwr: number;
    let accMotorMaxPwr: number;
    let accMotorTotalDist: number;
    let accMotorAccelDist: number;
    let accMotorDecelDist: number;
    let accMotorIsNeg: boolean;

    let accMotorsStartingPwr: number;
    let accMotorsMaxPwr: number;
    let accMotorsFinishingPwr: number;
    let accMotorsTotalDist: number;
    let accMotorsAccelDist: number;
    let accMotorsDecelDist: number;
    let accMotorsIsNeg: boolean;

    let accMotorsComplexMotionStartingPwrs = { left: 0, right: 0 };
    let accMotorsComplexMotionMaxPwrs = { left: 0, right: 0 };
    let accMotorsComplexMotionFinishingPwrs = { left: 0, right: 0 };
    let accMotorsComplexMotionIsNeg = { left: false, right: false };
    let accMotorsComplexMotionTotalDist = { left: 0, right: 0 };
    let accMotorsComplexMotionAccelDist = { left: 0, right: 0 };
    let accMotorsComplexMotionDecelDist = { left: 0, right: 0 };

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
        isDoneLeft: boolean,
        isDoneRight: boolean
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
     * @param totalDist значение всей дистанции, eg: 500
     * @param accelDist значение дистанции ускорения, eg: 150
     * @param decelDist значение дистанции замедления, eg: 150
     */
    //% blockId="AccOneEncConfig"
    //% block="config motor acceleration minPwr = $minPwr maxPwr = $maxPwr accelDist = $accelDist decelDist = $decelDist totalDist = $totalDist"
    //% block.loc.ru="конфигурация ускорения мотора minPwr = $minPwr maxPwr = $maxPwr accelDist = $accelDist decelDist = $decelDist totalDist = $totalDist"
    //% inlineInputMode="inline"
    //% minPwr.shadow="motorSpeedPicker"
    //% maxPwr.shadow="motorSpeedPicker"
    //% weight="79"
    //% group="Ускорение/замедлениие мотора"
    export function accOneEncConfig(minPwr: number, maxPwr: number, totalDist: number, accelDist: number, decelDist: number) {
        accMotorMinPwr = Math.abs(minPwr);
        accMotorMaxPwr = Math.abs(maxPwr);
        accMotorTotalDist = Math.abs(totalDist);
        accMotorAccelDist = Math.abs(accelDist);
        accMotorDecelDist = Math.abs(decelDist);
        accMotorIsNeg = maxPwr < 0;
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
        let done: boolean = false;
        let pwrOut: number;
        const currEnc = Math.abs(enc);

        if (currEnc >= accMotorTotalDist) {
            pwrOut = 0;
            done = true;
        } else if (currEnc > accMotorTotalDist / 2) {
            if (accMotorDecelDist == 0) pwrOut = accMotorMaxPwr;
            else pwrOut = (accMotorMaxPwr - accMotorMinPwr) / Math.pow(accMotorDecelDist, 2) * Math.pow(currEnc - accMotorTotalDist, 2) + accMotorMinPwr;
        } else {
            if (accMotorAccelDist == 0) pwrOut = accMotorMaxPwr;
            else pwrOut = (accMotorMaxPwr - accMotorMinPwr) / Math.pow(accMotorAccelDist, 2) * Math.pow(currEnc - 0, 2) + accMotorMinPwr;
        }

        pwrOut = Math.constrain(pwrOut, accMotorMinPwr, accMotorMaxPwr);

        return {
            pwr: accMotorIsNeg ? -pwrOut : pwrOut,
            isDone: done
        };
    }

    /**
     * Конфигурация ускорения и замедления шассии двух моторов.
     * @param startPwr входное значение скорости (мощности) на старте, eg: 20
     * @param maxPwr входное значение максимальной скорости (мощности), eg: 50
     * @param endPwr входное значение скорости (мощности) при замедлении, eg: 20
     * @param totalDist значение всей дистанции, eg: 500
     * @param accelDist значение дистанции ускорения, eg: 150
     * @param decelDist значение дистанции замедления, eg: 150
     */
    //% blockId="AccTwoEncLinearMotionConfig"
    //% block="config accel/deceleration chassis at startPwr = $startPwr maxPwr = $maxPwr endPwr = $endPwr|totalDist = $totalDist accelDist = $accelDist decelDist = $decelDist"
    //% block.loc.ru="конфигурирация ускорения/замедления управления шасси при startPwr = $startPwr maxPwr = $maxPwr endPwr = $endPwr|totalDist = $totalDist accelDist = $accelDist decelDist = $decelDist"
    //% inlineInputMode="inline"
    //% startPwr.shadow="motorSpeedPicker"
    //% maxPwr.shadow="motorSpeedPicker"
    //% endPwr.shadow="motorSpeedPicker"
    //% weight="69"
    //% group="Синхронизация шасси с ускорением/замедлением"
    export function accTwoEncLinearMotionConfig(startPwr: number, maxPwr: number, endPwr: number, totalDist: number, accelDist: number, decelDist: number) {
        accMotorsStartingPwr = Math.abs(startPwr);
        accMotorsMaxPwr = Math.abs(maxPwr);
        accMotorsFinishingPwr = Math.abs(endPwr);
        accMotorsTotalDist = Math.abs(totalDist);
        accMotorsAccelDist = Math.abs(accelDist);
        accMotorsDecelDist = Math.abs(decelDist);
        accMotorsIsNeg = maxPwr < 0;
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
            else pwrOut = (accMotorsMaxPwr - accMotorsFinishingPwr) / Math.pow(accMotorsDecelDist, 2) * Math.pow(currEnc - accMotorsTotalDist, 2) + accMotorsFinishingPwr;
            pwrOut = Math.max(accMotorsFinishingPwr, pwrOut); // Защита от "проседания" ниже EndPwr (мало ли, плавающая точка)
        } else if (currEnc < accMotorsAccelDist) { // Фаза ускорения
            if (accMotorsAccelDist == 0) pwrOut = accMotorsMaxPwr;
            else pwrOut = (accMotorsMaxPwr - accMotorsStartingPwr) / Math.pow(accMotorsAccelDist, 2) * Math.pow(currEnc, 2) + accMotorsStartingPwr;
            pwrOut = Math.max(accMotorsStartingPwr, pwrOut); // Защита от стартовой скорости ниже StartPwr
        } else { // Фаза постоянной скорости (между ускорением и замедлением)
            pwrOut = accMotorsMaxPwr;
        }

        return {
            pwr: accMotorsIsNeg ? -pwrOut : pwrOut,
            isDone: done
        };
    }

    /**
     * Конфигурация ускорения и замедления шасси двух моторов с разными максимальными скоростями.
     * @param startingPwr входное значение скорости (мощности) моторов на старте, eg: 20
     * @param maxPwrLeft входное значение максимальной скорости (мощности) левого мотора, eg: 50
     * @param maxPwrRight входное значение максимальной скорости (мощности) правого мотора, eg: 75
     * @param finishingPwr входное значение скорости (мощности) моторов при замедлении, eg: 20
     * @param totalDist значение всей дистанции, eg: 500
     * @param accelDist значение дистанции ускорения, eg: 100
     * @param decelDist значение дистанции замедления, eg: 150
     */
    //% blockId="AccTwoEncComplexMotionConfig"
    //% block="config accel/deceleration chassis at startingPwr = $startingPwr maxPwrLeft = $maxPwrLeft maxPwrRight = $maxPwrRight finishingPwr = $finishingPwr|totalDist = $totalDist accelDist = $accelDist decelDist = $decelDist"
    //% block.loc.ru="конфигурирация ускорения/замедления шасси при startingPwr = $startingPwr maxPwrLeft = $maxPwrLeft maxPwrRight = $maxPwrRight finishingPwr = $finishingPwr|totalDist = $totalDist accelDist = $accelDist decelDist = $decelDist"
    //% inlineInputMode="inline"
    //% startingPwr.shadow="motorSpeedPicker"
    //% maxPwrLeft.shadow="motorSpeedPicker"
    //% maxPwrRight.shadow="motorSpeedPicker"
    //% finishingPwr.shadow="motorSpeedPicker"
    //% weight="59"
    //% group="Синхронизация шасси с ускорением/замедлением"
    export function accTwoEncComplexMotionConfig(startingPwr: number, maxPwrLeft: number, maxPwrRight: number, finishingPwr: number, totalDist: number, accelDist: number, decelDist: number) {
        const absStartingPwr = Math.abs(startingPwr);
        const absMaxLeft = Math.abs(maxPwrLeft);
        const absMaxRight = Math.abs(maxPwrRight);
        const absFinishingPwr = Math.abs(finishingPwr);

        const absTotalDist = Math.abs(totalDist);
        const absAccelDist = Math.abs(accelDist);
        const absDecelDist = Math.abs(decelDist);

        // Коэффициент пропорциональности (отношение макс. скоростей). Если один из моторов = 0, используем другой как базовый
        const ratio = (absMaxLeft == 0 || absMaxRight == 0) ? 1 : (absMaxLeft < absMaxRight ? absMaxRight / absMaxLeft : absMaxLeft / absMaxRight);
        
        if (absMaxLeft < absMaxRight && absMaxLeft > 0) { // Левый мотор медленнее - он получает базовые значения
            accMotorsComplexMotionStartingPwrs.left = absStartingPwr;
            accMotorsComplexMotionStartingPwrs.right = absStartingPwr * ratio;
            accMotorsComplexMotionFinishingPwrs.left = absFinishingPwr;
            accMotorsComplexMotionFinishingPwrs.right = absFinishingPwr * ratio;
            accMotorsComplexMotionTotalDist.left = absTotalDist;
            accMotorsComplexMotionTotalDist.right = absTotalDist * ratio;
            accMotorsComplexMotionAccelDist.left = absAccelDist;
            accMotorsComplexMotionAccelDist.right = absAccelDist * ratio;
            accMotorsComplexMotionDecelDist.left = absDecelDist;
            accMotorsComplexMotionDecelDist.right = absDecelDist * ratio;
        } else { // Правый мотор медленнее - он получает базовые значения
            accMotorsComplexMotionStartingPwrs.left = absStartingPwr * ratio;
            accMotorsComplexMotionStartingPwrs.right = absStartingPwr;
            accMotorsComplexMotionFinishingPwrs.left = absFinishingPwr * ratio;
            accMotorsComplexMotionFinishingPwrs.right = absFinishingPwr;
            accMotorsComplexMotionTotalDist.left = absTotalDist * ratio;
            accMotorsComplexMotionTotalDist.right = absTotalDist;
            accMotorsComplexMotionAccelDist.left = absAccelDist * ratio;
            accMotorsComplexMotionAccelDist.right = absAccelDist;
            accMotorsComplexMotionDecelDist.left = absDecelDist * ratio;
            accMotorsComplexMotionDecelDist.right = absDecelDist;
        }
        accMotorsComplexMotionMaxPwrs.left = maxPwrLeft;
        accMotorsComplexMotionMaxPwrs.right = maxPwrRight;

        // КРИТИЧНО для поворота относительно мотора, если скорость мотора 0, обнуляем ВСЕ параметры
        zeroMotorProfile("left", absMaxLeft);
        zeroMotorProfile("right", absMaxRight);

        accMotorsComplexMotionIsNeg.left = maxPwrLeft < 0;
        accMotorsComplexMotionIsNeg.right = maxPwrRight < 0;
    }

    function zeroMotorProfile(side: "left" | "right", absMax: number) {
        if (absMax === 0) {
            accMotorsComplexMotionStartingPwrs[side] = 0;
            accMotorsComplexMotionFinishingPwrs[side] = 0;
            accMotorsComplexMotionTotalDist[side] = 0;
            accMotorsComplexMotionAccelDist[side] = 0;
            accMotorsComplexMotionDecelDist[side] = 0;
        }
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
        const profileLeft = accTwoEncComplexMotionComputeMotorProfile(
            eLeft,
            accMotorsComplexMotionTotalDist.left, accMotorsComplexMotionAccelDist.left, accMotorsComplexMotionDecelDist.left,
            accMotorsComplexMotionStartingPwrs.left, Math.abs(accMotorsComplexMotionMaxPwrs.left), accMotorsComplexMotionFinishingPwrs.left,
            accMotorsComplexMotionIsNeg.left
        ); // Расчёт мощности левого мотора
        const profileRight = accTwoEncComplexMotionComputeMotorProfile(
            eRight,
            accMotorsComplexMotionTotalDist.right, accMotorsComplexMotionAccelDist.right, accMotorsComplexMotionDecelDist.right,
            accMotorsComplexMotionStartingPwrs.right, Math.abs(accMotorsComplexMotionMaxPwrs.right), accMotorsComplexMotionFinishingPwrs.right,
            accMotorsComplexMotionIsNeg.right
        ); // Расчёт мощности правого мотора

        return {
            pwrLeft: profileLeft.pwr,
            pwrRight: profileRight.pwr,
            isDoneLeft: profileLeft.isDone,
            isDoneRight: profileRight.isDone
        };
    }

    // Расчёт профиля скорости (мощности) мотора 
    function accTwoEncComplexMotionComputeMotorProfile(enc: number, totalDist: number, accelDist: number, decelDist: number, startPwr: number, maxPwr: number, endPwr: number, isNeg: boolean): AccelMotor {
        let done: boolean = false;
        let pwrOut: number;
        const currEnc = Math.abs(enc);

        const absStartPwr = Math.abs(startPwr);
        const absMaxPwr = Math.abs(maxPwr);
        const absEndPwr = Math.abs(endPwr);

        if (currEnc >= totalDist) { // Фаза финиша
            pwrOut = 0;
            done = true;
        } else if (currEnc > totalDist - decelDist) { // Фаза замедления
            if (decelDist == 0) pwrOut = absMaxPwr;
            else pwrOut = (absMaxPwr - absEndPwr) / Math.pow(decelDist, 2) * Math.pow(currEnc - totalDist, 2) + absEndPwr;
            pwrOut = Math.max(absEndPwr, Math.min(absMaxPwr, pwrOut)); // Ограничение для фазы замедления
        } else if (currEnc < accelDist) { // Фаза ускорения
            if (accelDist == 0) pwrOut = absMaxPwr;
            else pwrOut = (absMaxPwr - absStartPwr) / Math.pow(accelDist, 2) * Math.pow(currEnc, 2) + absStartPwr;
            pwrOut = Math.max(absStartPwr, Math.min(absMaxPwr, pwrOut)); // Ограничение для фазы ускорения
        } else { // Фаза постоянной скорости (между ускорением и замедлением)
            pwrOut = absMaxPwr;
        }

        return {
            pwr: isNeg ? -pwrOut : pwrOut,
            isDone: done
        };
    }

}