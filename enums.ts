const enum MeasurementUnit {
    //% block="cm"
    //% block.loc.ru="см"
    Centimeters,
    //% block="mm"
    //% block.loc.ru="мм"
    Millimeters
}

// Перечисление о типах относительных поворотов
const enum WheelPivot {
    //% block="left"
    //% block.loc.ru="левого колеса"
    LeftWheel,
    //% block="right"
    //% block.loc.ru="правого колеса"
    RightWheel
}

// Перечислении о типах торможения
const enum Braking {
    //% block="break at hold"
    //% block.loc.ru="тормоз с удержанием"
    Hold,
    //% block="no break"
    //% block.loc.ru="тормоз без удержания"
    NoBreak,
    //% block="nothing"
    //% block.loc.ru="не тормозить"
    NoStop
}