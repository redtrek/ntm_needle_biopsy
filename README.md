# Nontraditional Manufacturing Laboratory - Smart Biopsy Needle
This repository contains the all the relevant code, documentation, and PCB design files for the needle biopsy device.

https://redtrek.github.io/ntm_needle_biopsy/

# Progress Overview
**Biopsy Needle Firmware:**
- This has been cleaned with legacy code removed, a rewritten state machine, and added documentation for all functions.
- Completely reorganized and formatted for new components.
- Saving current, position, RPM, and force data to microSD card.
- Adjustable inputs have been added allowing for variable speeds. Now fully implemented.
- Button holding supported as a form of input to zero and return the device to origin.

**OLED Display:**
- Supports input via a potentiometer and accurately displays in real time.
- Shows current, RPM, position, and force information in real-time.
- General status and battery capacity information.

**Real-time Data Collection**
- Device can track and log position, current, RPM, and force information in real time.
- Navigation of state machine through calculated sensor values.
- Automated safety limits based on distance traveled by device.
- Real-time low pass and moving average filter calculations and logging.

**MSC Capabilities**
- MSC complete: Can view created files on a Windows machine through the native filesystem.
- Can read/write logs to the SD card from Windows machine and RP2040 device.
- Automatically opens after USB connection.
- Separate toggle to enter filesystem mode.

**PCB Design:**
- Design Files have been collected.
- All components represented and connected in schematic files.
- Necessary devices configured for RP2040's operation.
- INA219 breakout board recreated.

**Bugs/Challenges:**
- Broken ASM (FIXED): safety condition meant that users had to press the button twice to exit states at times. It was a mix of a timing issue and trying to have the safety features independent from the states themselves. I moved the conditions inside the states as well as shifting all the operations into the correct positions.
- Difficulties with potentiometer circuit (FIXED): the voltage oscillates a lot causing the read values to change to quickly to be viable for normal inputs. I have added a capacitor to act as a low pass filter in order to mitigate this. Additionally, I have implemented an averaging function in order to smoothen out the values. Currently, values can be read in 5% increments. (Fixed 10/28/24: Hardware bug with the circuit. Wire underneath the potentiometer prevented contacts from having a stable position)
- Unmarked code (FIXED): unlabeled code has been documented in the main code file and functions that were not operating correctly have been removed. One example is the velocity calculation function which was incomplete, unlabeled, and did not use a time dependent quantity.
- INA219 (FIXED): not functioning properly. Returning extremely high values for calculated voltages and currents.
- RPM and Positional Data Incorrect (FIXED): interrupt not catching every falling edge of the motor_outputA signal.
- Motor activation on startup (FIXED): PWM pin receiving 3.3 Volts from the feather board on startup activating the motor.
- Variable speed (FIXED): inputs crash the system. Likely due to timing issues with state transitions that need to be resolved.
- MSC not working (FIXED): library needs to be configured and various changes to the source code necessary.


# Quick-Start
** In your WSL2 environment ***
- Clone this repository and navigate to the build folder of code/biopsy_needle
- Build the code using the command "cmake -DPICO_BOARD=adafruit_feather_rp2040 .."
- Run the command "make".
- Put the RP2040 device into bootloader mode by holding down BOOTSEL and pressing RESET.
- Drag the .uf2 file inside of the build folder into the flash file explorer.

# PCB Ordering
- To order a new PCB upload biopsy_needle.zip to OSHPark.
- To order a new stencil, zip the F_Paste.gbr and B_Paste.gbr and submit to OSHStencils.
- The PCB has been designed around OSHPark constraints so be cautious if ordering from other vendors.


# PCB Modification
- If modifying the PCB, extract biopsy_needle_src.zip and open the project file using KiCAD 8.0 or above. Be careful to adjust design rules to PCB vendor. All information can be found on the website of said vendors.
- Make sure to run the DRC and ERC before continuing with PCB order.
- To replace parts you can manually do so by heating up a soldering iron to 700-800 degrees farenheit, applying flux to the pads, and using tweezers to remove components. Flux solder wick can be used to clean the area and Isopropyl alcohol can be applied to remove any access flux.

# PCB Assembly
- Refer to Bill of Materials, typically labeled biopsy_needle.xlsx for parts ordering. Typical UF supported vendors are Digikey, Pololu, and Mouser Electronics.
- 1. Attach PCB to appropriate jig and place stencil on top such that all copper regions of the PCB are visible through stencil holes.
- 2. While holding the stencil steady, firmly wipe solder paste across entire stencil.
- 3. Slowly lift stencil up from the PCB.
- 4. Place SMT components in pads utilizing PCB board design files as reference. One order that could be used is: capacitors/resistors, ICs. You may need a magnifying glass, microscope, and tweezers for smaller components (e.g. 0402)
- 5. Bake in a reflow oven (typically 7 minute process) and when ready wait 10 minutes to cool.
- 6. Repeat for back side if necessary.
- 7. Hand solder headers and other through hole components if necessary.

# PCB Firmware
- To test PCB functionality of pilot devices and interfacing with the motor, uncomment the testingSuite() function in main. This will prevent the rest of the code from running and essentially send the device into a state were the operation of the buttons, potentiometer, and other modules can be observed without involving the other power components.

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