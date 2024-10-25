# Nontraditional Manufacturing Laboratory - Aspiration Assisted Needle Biopsy Device
This repository contains the all the relevant code, documentation, and PCB design files for the needle biopsy device.

# Progress Overview
**Biopsy Needle Code:**
- This has been cleaned with legacy code removed, a rewritten state machine, and added documentation for all functions.
- Completely reorganized and formatted for new components.
- Saving current information to EEPROM has been integrated (PERSISTENT DATA NEEDS TOTAL REWORK / NEW DEV BOARD).
- Adjustable inputs have been added allowing for variable speeds.

**LCD Input Display:**
- Test code for this proof of concept is complete and functional.
- Supports input via a potentiometer and accurately displays in real time to the LCD.
- Adjustable backlight.
- Value calculations are the same that would be used on the Biopsy Needle Code.

**PCB Design:**
- Design Files have been collected.
- Work has begun on a custom library with necessary footprints (NEEDED FOR DECODER) and symbols (DONE).
- Most of the most important signals have been connected.

# Background
_Introduction_

A biopsy is a common procedure used for cancer diagnosis through tissue sample collection. For such conditions, such as prostate cancer, earlier and more accurate sampling leads to a higher probability of treatment success and has been associated with a decreased death rate.
Prostate biopsy implements tend to be core needle biopsy devices, which hold tissue samples inside a hollow portion after insertion, because of their accessibility. Such devices are subdivided into side-cut and end-cut needles. Side-cut needles utilize a hollow notch along the needle which inherently restricts potential sample size, loses orientation/positional information of the sample, introduces opportunity for operational inaccuracy, and may demand multiple cuts whilst compromising tissue behind the sample area. End-cut needles in contrast use an entirely hollow needle with a surrounding cannula which enables larger sample collection but also risks failures regarding actual sample yield.
Today’s offerings of either core needle devices additionally tend to utilize a violent spring-loaded mechanism that can cause patient discomfort and potentially damage the sample tissue.

_Mechanical Properties of Prostate Tissue_

There is much potential for variation when measuring the properties of tissues due to natural variation and differing methodologies. The primary measure for prostate tissue is stiffness as it holds diagnostic value: i.e. stiffer cancer cells are less likely to spread to the rest of the body. When experimentally recording such data, the following relationship was discovered: cancerous tissue is more stiff at the macroscopic level despite the cells themselves being less stiff than the cells of benign tissue. This is hypothesized to be due to cancer’s growth conditions which involve rapid increase in limited areas, increasing compression. These are important considerations when selecting phantom materials that as an experimental equivalent to such tissues (ranging between benign and cancerous stiffness).

_Tissue Collection Effectiveness of Handheld Aspiration-Assisted Biopsy Device_
_(Handheld Aspiration-Assisted Biopsy Device Design)_

The device design of this project, the UF Device, utilizes a syringe, coaxial needle, inner stylet connected to syringe plunger, and two-pronged external needle. The operation is as follows: The device is inserted into the tissue until reaching the desired depth. Then a button press signals the DC motor to move the barrel and external needle forward until the lead nut jams the motor by making contact with the housing, increasing the load on the motor which spikes the current. The current sensor in the device (breadboard) detects this spike and puts the motor in a standby state before retraction. Here, the vacuum will hold the sample in, and pulling the device from the tissue will complete extraction. Then the sample can be manually retrieved either by turning the motor on again into a tissue collection receptacle or by removing the needle and pushing on the plunger. Another button press will signal retraction at which point the lead nut making contact with the casing at the end of the screw will spike the current once more, signaling the device to cease operation.

_(Tissue Collection Testing Methodology)_

The UF device was tested on six animal tissues and two separate gelatin phantoms (7g/100mL and 21g/100mL) with its performance compared to the BioPince and Max-Core devices, all with a constant needle length of 255 mm. Out of the three, only the Max-Core cannot adjust its needle travel distance. For the UF device, such adjustments are made by changing the thickness of the lead nut, which for this experiment was 33 mm to match the BioPince. Trial sample collections were used to select adequate phantom materials (bovine) by a professional physician and another 12 trials were used with each biopsy device measuring masses, sample lengths, and distortion.

_(Tissue Collection Testing Results)_

The statistical significance of the trial results indicates that the UF device’s samples were more massive which was surprising given that the BioPince collected the longest samples. These findings imply some degree of tissue distortion either being that the spring mechanism of the BioPince and its general operation stretch the tissue or that the UF device’s vacuum is compressing tissue. The UF device’s vacuum may be sucking in more material than initially desired and such distortions demand further investigation. Importantly, the UF device’s samples also displayed an enlarged ball-like structure at one end which is likely a result of being held in the space between the inner stylet and needle. There is potential for this to be used as orientational information but how such a phenomenon can be controlled must be explored.