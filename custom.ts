namespace control {

    /**
     * Function to wait for a loop to complete for a specified time.
     * @param startTime start time, eg: 0
     * @param delay waiting time, eg: 10
     */
    //% blockId="PauseUntilTime"
    //% block="wait $delay ms|at start at $startTime"
    //% block.loc.ru="ждать $delay мс|при начале в $startTime|мс"
    //% weight="6"
    export function pauseUntilTime(startTime: number, ms: number) {
        if (startTime == 0) startTime = control.millis();
        const waitCompletionTime = startTime + ms;
        while (control.millis() < waitCompletionTime) loops.pause(0.01);
    }

}