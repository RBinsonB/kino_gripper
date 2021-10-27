# Assembly instructions
## Necessary parts

The following parts are required to assemble the gripper.

### Printed parts
* (1x) Base
* (1x) Base attachement
* (1x) Gear finger right
* (1x) Gear finger left
* (1x) Outer finger right
* (1x) Outer finger left
* (1x) Central finger right
* (1x) Central finger left
* (1x) finger tips, multiple options possible:
  * Without pads:
    * (1x) finger tip left
    * (1x) finger tip right
  * With thin pads:
    * (1x) finger tip pads left
    * (1x) finger tip pads right
  * With thick pads:
    * (1x) finger tip thick pads left
    * (1x) finger tip thick pads right

### Electronics
* (1x) [Dynamixel AX-12A Servomotor](https://emanual.robotis.com/docs/en/dxl/ax/ax-12a/)
* (1x) [USB2AX](http://www.xevelabs.com/doku.php?id=product:usb2ax:usb2ax)
* (1x) Voltage regulator (step-down) LM2596 boards. The boards are cheap and easily findable online, but they should have the following dimensions to fit within the case.

<a><img src="/resources/documentation/pictures/lm2596_board.jpg" height="300" width="300"/></a>
<img src="/resources/documentation/pictures/lm2596_dimensions.png" height="300"/>

### Mechanical parts
* (18x) Ball bearing 10x4x3mm
* (2x) Extension spring (length max of 50mm at rest, diameter max of 8.5mm)
* (12x) M3 inserts (M4 outer diameter)
* (7x) M3 nuts
* (8x) (optionally self-locking) M3 nuts
* (12x) M3 x 16mm hex socket head screws
* (8x) M3 x 20mm hex socket head screws
* (2x) M3 x 5mm hex socket head screws
* (1x) M3 x 25mm countersunk head screw
* (7x) M5 x 8mm hex socket head screws (for attaching to the HEBI actuator)
* (6x) M2 x 10mm screws (for fastening servomotor)
* (6x) M2 nuts (for fastening servomotor)
* (3x) Zip ties

### Connectors
* (1x) Molex minifit Jr. male housing (2x1) and its associated crimp terminals (for connecting with HEBI Robotics Arm power supply)
* (2x) Mini-SPOX 2.50mm female housing (3 pins) and its associated crimp terminals (for connecting with the Dynamixel AX-12A servomotor, check [Dynamixel servomotor documentation](https://emanual.robotis.com/docs/en/dxl/ax/ax-12a/#connector-information) for more information)
* (1x) male & female connector (and associated crimp terminals) with at least 3 pins, small enough to fit inside the gripper. Here, a 2x2 Molex Microfit is used.

### Printing the parts
#### PLA
Most parts were printed using Ultimaker black/red PLA and/or black tough PLA with PVA supports (dual-printcore printer). The PVA support material is disovable in water and allow for a high precision in the printed connector housings. It's probably possible to print the parts without the PVA support material, but that would require more post-processing to remove the supports.

The following printing parameters were used for all parts printed in PLA:

| Parameter | Value |
| --------- | ----- |
| Layer height | 0.15mm |
| Infill | 50% |
| Infill pattern | cubic subdivision |
| Support material | PVA |
| Support pattern | triangles |
| Support overhang | 45% |

### Assembling everything

Add the inserts in the attachment part. Use a soldering iron is needed.

<img src="/resources/documentation/pictures/assembly_1.jpg" align="center" width="500"/>

Clip in the USB2AX.

<img src="/resources/documentation/pictures/assembly_2.jpg" align="center" width="500"/>

Solder the required wires on the DC power supply. It is also a good time to adjust the voltage output for the servomotor (between 9V and !"V, ideally 11.1V).

<img src="/resources/documentation/pictures/assembly_3.jpg" align="center" width="500"/>

It can be then screwed in place using M3x5mm screws. On the other end on the input wires, crimp the minifit male housing connector, so that the gripper can be plugged to the HEBI Robotics power supply.

<img src="/resources/documentation/pictures/assembly_4.jpg" align="center" width="500"/>

<img src="/resources/documentation/pictures/assembly_5.jpg" align="center" width="500"/>

Prepare the output cable of the USB2AX by crimping a mini-SPOX connector. Only the two side wires should be connected, not the middle one, since power will be provided by the DC power supply.

<img src="/resources/documentation/pictures/assembly_6.jpg" align="center" width="500"/>

Connect the cable and ziptie it with the power output wires.

<img src="/resources/documentation/pictures/assembly_7.jpg" align="center" width="500"/>

Respecting the connector pins, crimp the output power sypply ground and the USB2AX output ground together. Connect the ground, data output wire and power supply positive wire in a connector (here a Molex microfit).

<img src="/resources/documentation/pictures/assembly_8.jpg" align="center" width="500"/>

On the other main part, prepare a cable that will be connected to the servomotor. Crimp a mini-SPOX connector on one side and the complementary connector (here a Molex microfit), respecting the wiring of Dynamixel AX12-A servomotor. Fasten the cable against the part using zipties.

<img src="/resources/documentation/pictures/assembly_9.jpg" align="center" width="500"/>

Insert the M3 nut under the part.

<img src="/resources/documentation/pictures/assembly_10.jpg" align="center" width="500"/>

Put the servomotor in place, sliding it from under, and fasten it using M2x10mm screws.

<img src="/resources/documentation/pictures/assembly_11.jpg" align="center" width="500"/>

Prepare the right geared finger by mounting the 2 ball bearing on its axle, one on each side.

<img src="/resources/documentation/pictures/assembly_12.jpg" align="center" width="500"/>

Slide the finger in place and fasten it on its axle using a M3x25mm countersunk head screw.

<img src="/resources/documentation/pictures/assembly_13.jpg" align="center" width="500"/>







