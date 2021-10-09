//   RADIO BRIDGE PACKET DECODER v1.0
// (c) 2019 RadioBridge USA by John Sheldon

// General defines used in decode
var RESET_EVENT = "00";
var SUPERVISORY_EVENT = "01";
var TAMPER_EVENT = "02";
var LINK_QUALITY_EVENT = "FB";
var RATE_LIMIT_EXCEEDED_EVENT = "FC";
var TEST_MESSAGE_EVENT = "FD";
var DOWNLINK_ACK_EVENT = "FF";
var DOOR_WINDOW_EVENT = "03";
var PUSH_BUTTON_EVENT = "06";
var CONTACT_EVENT = "07";
var WATER_EVENT = "08";
var TEMPERATURE_EVENT = "09";
var TILT_EVENT = "0A";
var ATH_EVENT = "0D";
var ABM_EVENT = "0E";
var TILT_HP_EVENT = "0F";
var ULTRASONIC_EVENT = "10";
var SENSOR420MA_EVENT = "11";
var THERMOCOUPLE_EVENT = "13";
var VOLTMETER_EVENT = "14";
var CUSTOM_SENSOR_EVENT = "15";
var GPS_EVENT = "16";
var HONEYWELL5800_EVENT = "17";
var MAGNETOMETER_EVENT = "18";
var VIBRATION_LB_EVENT = "19";
var VIBRATION_HB_EVENT = "1A";



// Different network servers have different callback functions
// Each of these is mapped to the generic decoder function

// ----------------------------------------------

// function called by ChirpStack
function Decode(fPort, bytes, variables) {
    return Generic_Decoder(bytes, fPort);
}

// function called by TTN
function Decoder(bytes, port) {
    return Generic_Decoder(bytes, port);
}

// ----------------------------------------------



// The generic decode function called by one of the above network server specific callbacks
function Generic_Decoder(bytes, port) {

    // data structure which contains decoded messages
    var decoded = {};
    decoded.Message = {};
    decoded.Message.decoderVersion = "1.0";
    // The first byte contains the protocol version (upper nibble) and packet counter (lower nibble)
    ProtocolVersion = (bytes[0] >> 4) & 0x0f;
    PacketCounter = bytes[0] & 0x0f;


    // the event type is defined in the second byte
    EventType = Hex(bytes[1]);

    // the rest of the message decode is dependent on the type of event
    switch (EventType) {

        // ==================    RESET EVENT    ====================
        case RESET_EVENT:

            decoded.Message.event = "reset";

            // third byte is device type, convert to hex format for case statement 
            DeviceTypeByte = Hex(bytes[2]);

            // device types are enumerated below
            switch (DeviceTypeByte) {
                case "01": DeviceType = "Door/Window Sensor"; break;
                case "02": DeviceType = "Door/Window High Security"; break;
                case "03": DeviceType = "Contact Sensor"; break;
                case "04": DeviceType = "No-Probe Temperature Sensor"; break;
                case "05": DeviceType = "External-Probe Temperature Sensor"; break;
                case "06": DeviceType = "Single Push Button"; break;
                case "07": DeviceType = "Dual Push Button"; break;
                case "08": DeviceType = "Acceleration-Based Movement Sensor"; break;
                case "09": DeviceType = "Tilt Sensor"; break;
                case "0A": DeviceType = "Water Sensor"; break;
                case "0B": DeviceType = "Tank Level Float Sensor"; break;
                case "0C": DeviceType = "Glass Break Sensor"; break;
                case "0D": DeviceType = "Ambient Light Sensor"; break;
                case "0E": DeviceType = "Air Temperature and Humidity Sensor"; break;
                case "0F": DeviceType = "High-Precision Tilt Sensor"; break;
                case "10": DeviceType = "Ultrasonic Level Sensor"; break;
                case "11": DeviceType = "4-20mA Current Loop Sensor"; break;
                case "12": DeviceType = "Ext-Probe Air Temp and Humidity Sensor"; break;
                case "13": DeviceType = "Thermocouple Temperature Sensor"; break;
                case "14": DeviceType = "Voltage Sensor"; break;
                case "15": DeviceType = "Custom Sensor"; break;
                case "16": DeviceType = "GPS"; break;
                case "17": DeviceType = "Honeywell 5800 Bridge"; break;
                case "18": DeviceType = "Magnetometer"; break;
                case "19": DeviceType = "Vibration Sensor - Low Frequency"; break;
                case "1A": DeviceType = "Vibration Sensor - High Frequency"; break;
                default: DeviceType = "Device Undefined"; break;
            }

            decoded.Message.deviceType = DeviceType;

            // the hardware version has the major version in the upper nibble, and the minor version in the lower nibble
            HardwareVersion = ((bytes[3] >> 4) & 0x0f) + "." + (bytes[3] & 0x0f);

            decoded.Message.hardwareVersion =  HardwareVersion;

            // the firmware version has two different formats depending on the most significant bit
            FirmwareFormat = (bytes[4] >> 7) & 0x01;

            // FirmwareFormat of 0 is old format, 1 is new format
            // old format is has two sections x.y
            // new format has three sections x.y.z
            if (FirmwareFormat == 0)
                FirmwareVerison = bytes[4] + "." + bytes[5];
            else
                FirmwareVerison = ((bytes[4] >> 2) & 0x1F) + "." + ((bytes[4] & 0x03) + ((bytes[5] >> 5) & 0x07)) + "." + (bytes[5] & 0x1F);

            decoded.Message.firmwareVersion = FirmwareVerison;

            break;

        // ================   SUPERVISORY EVENT   ==================

        case SUPERVISORY_EVENT:
            decoded.Message.event = "supervisory";

            // note that the sensor state in the supervisory message is being depreciated, so those are not decoded here

            // battery voltage is in the format x.y volts where x is upper nibble and y is lower nibble
            BatteryLevel = ((bytes[4] >> 4) & 0x0f) + "." + (bytes[4] & 0x0f);

            decoded.Message.batteryLevel =  BatteryLevel;

            // the accumulation count is a 16-bit value
            AccumulationCount = (bytes[9] * 256) + bytes[10];
            decoded.Message.accumulationCount = AccumulationCount;


            // decode bits for error code byte
            TamperSinceLastReset = (bytes[2] >> 4) & 0x01;
            decoded.Message.tamperSinceLastReset = TamperSinceLastReset;

            CurrentTamperState = (bytes[2] >> 3) & 0x01;
            decoded.Message.currentTamperState = CurrentTamperState;

            ErrorWithLastDownlink = (bytes[2] >> 2) & 0x01;
            decoded.Message.errorWithLastDownlink = ErrorWithLastDownlink;

            BatteryLow = (bytes[2] >> 1) & 0x01;
            decoded.Message.batteryLow = BatteryLow;

            RadioCommError = bytes[2] & 0x01;
            decoded.Message.radioCommError = RadioCommError;

            break;

        // ==================   TAMPER EVENT    ====================
        case TAMPER_EVENT:
            decoded.Message.event = "tamper";

            TamperState = bytes[2];

            // tamper state is 0 for open, 1 for closed
            if (TamperState == 0)
                decoded.Message.tamperState = "open";
            else
                decoded.Message.tamperState = "closed";

            break;

        // ==================   LINK QUALITY EVENT    ====================
        case LINK_QUALITY_EVENT:
            decoded.Message.event = "linkQuality";

            CurrentSubBand = bytes[2];
            decoded.Message.currentSubBand = CurrentSubBand;

            RSSILastDownlink = bytes[3];
            decoded.Message.rssiLastDownlink = RSSILastDownlink;

            SNRLastDownlink = bytes[4];
            decoded.Message.snrLastDownlink = SNRLastDownlink;

            break;

        // ==================   RATE LIMIT EXCEEDED EVENT    ====================
        case RATE_LIMIT_EXCEEDED_EVENT:

            // this feature is depreciated so it is not decoded here
            decoded.Message.event = "rateLimit";

            break;

        // ==================   TEST MESSAGE EVENT    ====================
        case TEST_MESSAGE_EVENT:

            // this feature is depreciated so it is not decoded here
            decoded.Message.event = "test";

            break;


        // ================  DOOR/WINDOW EVENT  ====================
        case DOOR_WINDOW_EVENT:

            decoded.Message.event = "door/window";

            SensorState = bytes[2];

            // 0 is closed, 1 is open
            if (SensorState == 0)
                decoded.Message.doorWindowState = "closed";
            else
                decoded.Message.doorWindowState = "open";

            break;

        // ===============  PUSH BUTTON EVENT   ===================
        case PUSH_BUTTON_EVENT:

            decoded.Message.event = "pushButton";

            ButtonID = Hex(bytes[2]);

            switch (ButtonID) {
                // 01 and 02 used on two button
                case "01": decoded.Message.buttonSelected = "button1"; break;
                case "02": decoded.Message.buttonSelected = "button2"; break;
                // 03 is single button
                case "03": decoded.Message.buttonSelected = "button1"; break;
                // 12 when both buttons pressed on two button
                case "12": decoded.Message.buttonSelected = "buttonBoth"; break;
                default: decoded.Message.buttonSelected = "buttonUndefined"; break;
            }

            ButtonState = bytes[3];

            switch (ButtonState) {
                case 0: decoded.Message.buttonState = "pressed"; break;
                case 1: decoded.Message.buttonState = "released"; break;
                case 2: decoded.Message.buttonState = "held"; break;
                default: decoded.Message.buttonState = "undefined"; break;
            }

            decoded.Message += ", Button State: " + SensorStateDescription;

            break;

        // =================   CONTACT EVENT   =====================
        case CONTACT_EVENT:
            
            decoded.Message.event = "dryContact";

            ContactState = bytes[2];

            // if state byte is 0 then shorted, if 1 then opened
            if (ContactState == 0)
                decoded.Message.dryContactState = "closed"
            else
                decoded.Message.dryContactState = "open"
            
            break;

        // ===================  WATER EVENT  =======================
        case WATER_EVENT:

            decoded.Message.event = "water";

            SensorState = bytes[2];

            if (SensorState == 0)
                decoded.Message.waterSensorState = "present";
            else
                decoded.Message.waterSensorState = "notPresent";

            WaterRelativeResistance = bytes[3];

            decoded.Message.waterSensorRelativeResistance = WaterRelativeResistance;

            break;

        // ================== TEMPERATURE EVENT ====================
        case TEMPERATURE_EVENT:

            decoded.Message.event = "temperature";

            TemperatureEvent = bytes[2];
            
            decoded.Message.reportReason = DecodeReportReason(TemperatureEvent);

            // current temperature reading
            CurrentTemperature = Convert(bytes[3], 0);
            decoded.Message.temperature = CurrentTemperature;

            // relative temp measurement for use with an alternative calibration table
            RelativeMeasurement = Convert(bytes[4], 0);
            decoded.Message.temperatureRelative = RelativeMeasurement;

            break;

        // ====================  TILT EVENT  =======================
        case TILT_EVENT:

            decoded.Message.event = "tilt";

            TiltEvent = bytes[2];

            switch (TiltEvent) {
                case 0: TiltEventDescription = "transitionedVertical"; break;
                case 1: TiltEventDescription = "transitionedHorizontal"; break;
                case 2: TiltEventDescription = "changeVertical"; break;
                case 3: TiltEventDescription = "changeHorizontal"; break;
                default: TiltEventDescription = "undefined"; break;
            }

            decoded.Message.reportReason = TiltEventDescription;

            TiltAngle = bytes[3];

            decoded.Message.tiltAngle = tiltAngle;

            break;

        // =============  AIR TEMP & HUMIDITY EVENT  ===============
        case ATH_EVENT:

            decoded.Message.event = "airTemperatureHumidity";

            ATHEvent = bytes[2];

            switch (ATHEvent) {
                case 0: 
                case 1: 
                case 2: 
                case 3: 
                case 4: ATHDescription = DecodeReportReason(ATHEvent); break;
                case 5: 
                case 6: 
                case 7: 
                case 8: ATHDescription = DecodeReportReason(ATHEvent-4); break;
                default: ATHDescription = "undefined"; break;
            }

            decoded.Message.reportReason = ATHDescription;
            if(ATHEvent != 0){
                decoded.Message.reportSensor = (ATHEvent > 4) ? "humidity" : "temperature";
            }
            // integer and fractional values between two bytes
            Temperature = Convert((bytes[3]) + ((bytes[4] >> 4) / 10), 1);
            decoded.Message.temperature = Temperature;

            // integer and fractional values between two bytes
            Humidity = +(bytes[5] + ((bytes[6]>>4) / 10)).toFixed(1);
            decoded.Message.humidity = Humidity;

            break;

        // ============  ACCELERATION MOVEMENT EVENT  ==============
        case ABM_EVENT:

            decoded.Message.event = "acceleration";

            ABMEvent = bytes[2];

            if (ABMEvent == 0)
                ABMEventDescription = "started";
            else
                ABMEventDescription = "stopped";

            decoded.Message.state = ABMEventDescription;

            break;

        // =============  HIGH-PRECISION TILT EVENT  ===============
        case TILT_HP_EVENT:

            decoded.Message.event = "highPrecisiontTilt";

            TiltEvent = bytes[2];

            switch (TiltEvent) {
                case 0: TiltEventDescription = "periodic"; break;
                case 1: TiltEventDescription = "vertical"; break;
                case 2: TiltEventDescription = "notVertical"; break;
                case 3: TiltEventDescription = "changeTowardVertical"; break;
                case 4: TiltEventDescription = "changeAwayVertical"; break;
                default: TiltEventDescription = "Undefined"; break;
            }

            decoded.Message.reportReason = TiltEventDescription;

            // integer and fractional values between two bytes
            Angle = +(bytes[3] + (bytes[4] / 10)).toFixed(1);
            decoded.Message.angle = Angle;

            Temperature = Convert(bytes[5], 0);
            decoded.Message.temperature = Temperature;

            break;

        // ===============  ULTRASONIC LEVEL EVENT  ================
        case ULTRASONIC_EVENT:

            decoded.Message = "ultrasonicLevel";

            UltrasonicEvent = bytes[2];

            decoded.Message.reportReason = DecodeReportReason(UltrasonicEvent);

            // distance is calculated across 16-bits
            Distance = ((bytes[3] * 256) + bytes[4]);

            decoded.Message.distance = Distance;
            break;

        // ================  4-20mA ANALOG EVENT  ==================
        case SENSOR420MA_EVENT:

            decoded.Message.sensor420ma = "4-20mA";

            Sensor420mAEvent = bytes[2];


            decoded.Message.reportReason = DecodeReportReason(Sensor420mAEvent);

            // calculatec across 16-bits, convert from units of 10uA to mA
            Analog420Measurement = ((bytes[3] * 256) + bytes[4]) / 100;

            decoded.Message.current = Analog420Measurement;

            break;

        // =================  THERMOCOUPLE EVENT  ==================
        case THERMOCOUPLE_EVENT:

            decoded.Message = "thermocouple";

            ThermocoupleEvent = bytes[2];

            decoded.Message = DecodeReportReason(ThermocoupleEvent);

            // decode is across 16-bits
            Temperature = parseInt(((bytes[3] * 256) + bytes[4]) / 16);

            decoded.Message.temperature = Temperature;

            Faults = bytes[5];

            // decode each bit in the fault byte
            FaultColdOutsideRange = (Faults >> 7) & 0x01;
            FaultHotOutsideRange = (Faults >> 6) & 0x01;
            FaultColdAboveThresh = (Faults >> 5) & 0x01;
            FaultColdBelowThresh = (Faults >> 4) & 0x01;
            FaultTCTooHigh = (Faults >> 3) & 0x01;
            FaultTCTooLow = (Faults >> 2) & 0x01;
            FaultVoltageOutsideRange = (Faults >> 1) & 0x01;
            FaultOpenCircuit = Faults & 0x01;

            // Decode faults
            if (Faults == 0)
                decoded.Message.fault = "none";
            else {
                if (FaultColdOutsideRange)
                    decoded.Message.fault = "Fault: The cold-Junction temperature is outside of the normal operating range";

                if (FaultHotOutsideRange)
                    decoded.Message.fault = "Fault: The hot junction temperature is outside of the normal operating range";

                if (FaultColdAboveThresh)
                    decoded.Message.fault = "Fault: The cold-Junction temperature is at or above than the cold-junction temperature high threshold";

                if (FaultColdBelowThresh)
                    decoded.Message.fault = "Fault: The Cold-Junction temperature is lower than the cold-junction temperature low threshold";

                if (FaultTCTooHigh)
                    decoded.Message.fault = "Fault: The thermocouple temperature is too high";

                if (FaultTCTooLow)
                    decoded.Message.fault = "Fault: Thermocouple temperature is too low";

                if (FaultVoltageOutsideRange)
                    decoded.Message.fault = "Fault: The input voltage is negative or greater than VDD";

                if (FaultOpenCircuit)
                    decoded.Message.fault = "Fault: An open circuit such as broken thermocouple wires has been detected";
            }

            break;

        // ================  VOLTMETER ANALOG EVENT  ==================
        case VOLTMETER_EVENT:

            decoded.Message = "voltage";

            VoltmeterEvent = bytes[2];

            decoded.Message.reportReason = DecodeReportReason(VoltmeterEvent);

            // voltage is measured across 16-bits, convert from units of 10mV to V
            VoltageMeasurement = ((bytes[3] * 256) + bytes[4]) / 100;

            decoded.Message.voltage = VoltageMeasurement;
            break;


        // ================  CUSTOM SENSOR EVENT  ==================
        case CUSTOM_SENSOR_EVENT:

            decoded.Message = "custom";

            // Custom sensors are not decoded here

            break;


        // ================  VOLTMETER ANALOG EVENT  ==================
        case GPS_EVENT:

            decoded.Message = "gps";

            GPSStatus = bytes[2];

            // decode status byte
            GPSValidFix = GPSStatus & 0x01;

            decoded.Message.hasFix = GPSValidFix;

            // latitude and longitude calculated across 32 bits each, show 12 decimal places
            Latitude = toFixed((((bytes[3] * (2 ^ 24)) + (bytes[4] * (2 ^ 16)) + (bytes[5] * (2 ^ 8)) + bytes[6]) / (10 ^ 7)), 12);
            Latitude = toFixed((((bytes[7] * (2 ^ 24)) + (bytes[8] * (2 ^ 16)) + (bytes[9] * (2 ^ 8)) + bytes[10]) / (10 ^ 7)), 12);

            decoded.Message.latitude = Latitude;
            decoded.Message.longitude = Longitude;

            break;


        // ================  HONEYWELL 5800 EVENT  ==================
        case HONEYWELL5800_EVENT:

            decoded.Message.event = "honeywell5800";

            // honeywell sensor ID, 24-bits
            HWSensorID = (bytes[2] * (2 ^ 16)) + (bytes[3] * (2 ^ 8)) + bytes[4];

            decoded.Message.hwSensorID = HWSensorID;

            HWEvent = bytes[5];

            switch (HWEvent) {
                case 0: HWEventDescription = "Status code"; break;
                case 1: HWEventDescription = "Error Code"; break;
                case 2: HWEventDescription = "Sensor Data Payload"; break;
                default: HWEventDescription = "Undefined"; break;
            }

            decoded.Message.hwReportType = HWEventDescription;

            // represent the honeywell sensor payload in hex
            HWSensorPayload = Hex((bytes[6] * 256) + bytes[7]);

            decoded.Message.hwReportValue = HWSensorPayload;

            break;


        // ================  MAGNETOMETER EVENT  ==================
        case MAGNETOMETER_EVENT:
            decoded.Message = "magnetometer";
            // TBD

            break;


        // ================  VIBRATION LOW BANDWIDTH EVENT  ==================
        case VIBRATION_LB_EVENT:

            decoded.Message.event = "vibrationLowFrequency";

            VibeEvent = bytes[2];

            switch (VibeEvent) {
                case 0: VibeEventDescription = "Low Frequency Periodic Report"; break;
                case 4: VibeEventDescription = "Low Frequency X-Axis Has Risen Above Upper Threshold"; break;
                case 5: VibeEventDescription = "Low Frequency X-Axis Has Fallen Below Lower Threshold"; break;
                case 6: VibeEventDescription = "Low Frequency Y-Axis Has Risen Above Upper Threshold"; break;
                case 7: VibeEventDescription = "Low Frequency Y-Axis Has Fallen Below Lower Threshold"; break;
                case 8: VibeEventDescription = "Low Frequency Z-Axis Has Risen Above Upper Threshold"; break;
                case 9: VibeEventDescription = "Low Frequency Z-Axis Has Fallen Below Lower Threshold"; break;
                case 11: VibeEventDescription = "Low Frequency Exceeded G-Force Range"; break;
                default: VibeEventDescription = "undefined"; break;
            }

            decoded.Message.reportReason = VibeEventDescription;

            // X, Y, and Z velocities are 16-bits
            XVelocity = (bytes[3] * 256) + bytes[4];
            YVelocity = (bytes[5] * 256) + bytes[6];
            ZVelocity = (bytes[7] * 256) + bytes[8];

            decoded.Message.xVelocityInchesSecond =  XVelocity;
            decoded.Message.yVelocityInchessecond += YVelocity;
            decoded.Message.zVelocityInchesSecond += ZVelocity;

            // capture sign of temp
            VibeTemp = parseInt(bytes[9]);

            decoded.Message.vibrationTemperature = VibeTemp;

            break;

        // ================  VIBRATION HIGH BANDWIDTH EVENT  ==================
        case VIBRATION_HB_EVENT:

            decoded.Message.event = "vibrationHighFrequency";

            VibeEvent = bytes[2];

            switch (VibeEvent) {
                case 1: VibeEventDescription = "High Frequency Periodic Report"; break;
                case 2: VibeEventDescription = "High Frequency Vibration Above Upper Threshold"; break;
                case 3: VibeEventDescription = "High Frequency Vibration Below Lower Threshold"; break;
                case 10: VibeEventDescription = "High Frequency Exceeded G-Force Range"; break;
                default: VibeEventDescription = "Undefined"; break;
            }

            decoded.Message.reportReason = VibeEventDescription;

            // peak g-force
            PeakGForce = (bytes[3] * 256) + bytes[4];

            decoded.Message.peakGForce = PeakGForce;

            // capture sign of temp
            VibeTemp = parseInt(bytes[5]);

            decoded.Message.vibrationTemperature = VibeTemp;

            break;


        // ==================   DOWNLINK EVENT  ====================
        case DOWNLINK_ACK_EVENT:

            decoded.Message = "downlinkAck";

            DownlinkEvent = bytes[2];

            if (DownlinkEvent == 1)
                DownlinkEventDescription = "ack";
            else
                DownlinkEventDescription = "nack";

            decoded.Message.ack = DownlinkEventDescription;
            break;

        // end of EventType Case
    }

    // add packet counter and protocol version to the end of the decode
    decoded.Message.packetCount = PacketCounter;
    decoded.Message.protocolVersion = ProtocolVersion;

    return decoded;
}

function Hex(decimal) {
    decimal = ('0' + decimal.toString(16).toUpperCase()).slice(-2);
    return decimal;
}

function DecodeReportReason(eventByte) {
    var result="";
    switch (eventByte) {
        case 0: result = "periodic"; break;
        case 1: result = "overUpper"; break;
        case 2: result = "underLower"; break;
        case 3: result = "changeIncrease"; break;
        case 4: result = "changeDecrease"; break;
        default: result = "undefined"; break;
    }

    return result;
}

function Convert(number, mode) {
    switch (mode) {
        // for EXT-TEMP and NOP 
        case 0: if (number > 127) { result = number - 256 } else { result = number }; break
        //for ATH temp
        case 1: if (number > 127) { result = -+(number - 128).toFixed(1) } else { result = +number.toFixed(1) }; break
    }
    return result;
}

exports.Generic_Decoder = Generic_Decoder;