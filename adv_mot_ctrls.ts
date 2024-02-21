/**
* OFDL Advanced Motor Controller Block module (algorithm part).
* Based 1.1 ver, 2023/09/27.
*/
//% block="AdvMotCtrls"
namespace advmotctrls2 {

    let pwr: number;

    let syncVLeft: number;
    let syncVRight: number;
    let syncVLeftSign: number;
    let syncVRightSign: number;

    //% blockId=SyncMotorsConfig
    export function SyncMotorsConfig(vLeft: number, vRight: number) {
        syncVLeft = vLeft;
        syncVRight = vRight;
        syncVLeftSign = Math.abs(vLeft + 1) - Math.abs(vLeft);
        syncVRightSign = Math.abs(vRight + 1) - Math.abs(vRight);
    }

    //% blockId=GetErrorSyncMotors
    export function GetErrorSyncMotors(eLeft: number, eRight: number): number {
        return ((syncVRight * eLeft) - (syncVLeft * eRight));
    }

    interface MotorsPower {
        powerLeft: number;
        powerRight: number;
    }

    //% blockId=GetPowerSyncMotors
    export function GetPowerSyncMotors(U: number): MotorsPower {
        const pLeft = syncVLeft - syncVRightSign * U;
        const pRight = syncVRight + syncVLeftSign * U;
        return {
            powerLeft: pLeft,
            powerRight: pRight
        };
    }

}