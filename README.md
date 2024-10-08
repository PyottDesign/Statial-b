![Statial.b Title Banner](img/banner.png)

# Statial.b Adjustable Mouse
_Modern grip styles have grown out of players adapting to standardized mouse shapes.<br/>
The Statial.b lets new mouse shapes grow from grip styles._

The Statial.b is an open source DIY mouse design. This concept has adjustable surfaces that can be moved and locked in position to create a wide range of ergonomics for any grip style. Files for ambidextrous builds are included in the STL.zip "extras" folder.

Knowledge of 3D printing, basic electronics & soldering skills as well as the ability to upload the supplied code to an Arduino are needed for this build. The design is based around a 16,000 DPI optical sensor in the [PMW3389](https://www.tindie.com/products/citizenjoe/pmw3389-motion-sensor/) and driven by an Arduino compatible [Pro Micro](https://deskthority.net/wiki/Arduino_Pro_Micro) controller.

This is a fairly complex project. I tried to cover all the assembly details in the in the [instructions PDF](statial-b_instructions_v01-00.pdf) and on the [YouTube instructions](https://youtu.be/HTKbrcy7GT0), but some problem solving skills are still going to be required.

The finished design is a functioning prototype that requires a lot of adjusting to get it dialed in and is more fragile than a normal commercially produced product. It’s also a heavy mouse weighing up to a hefty 130 grams (depending on components used). All that said, it works great and is super fun to use. There’s really nothing else like it out there if you’re looking to explore mouse ergonomics.

All 3D printed parts need to be made in a “Tough” or “ABS like” resin material. Resin print color is up to you.<br/> ***This project requires resin printed parts. 
It (probably) won't work if parts are 3D printed with an FDM printer.***

Current cost estimate is around $200 for the build materials (that's in 2024 US dollar bucks, not including tools). This is just a rough guess so you have an idea of what you’re getting into.

Please check [Statial.b Youtube build](https://youtu.be/HTKbrcy7GT0) and the [Statial.b Instructions PDF](statial-b_instructions_v01-00.pdf) for more info on this project.

![Statial.b Orthos](img/orthos.png)
![Statial.b](img/statial-b.png)
<br/><br/>

# Code Background
* The Statial-b Arduino Sketch is modified from the [Ben Makes Everything PMW3389 Mouse](https://github.com/BenMakesEverything/PMW3389_Mouse)
* Which is based on [Dkao's Trackball Project](https://github.com/dkao/Kensington_Expert_Mouse_PMW3389_Arduino)
* That uses source code from [MrJohnk PMW3389](https://github.com/mrjohnk/PMW3389DM)
<br/><br/>

# Parts
Full BOM with source links can be found in the [Statial.b Instructions PDF](statial-b_instructions_v01-00.pdf).

* ~170 mL of [Tough](https://formlabs.com/store/materials/tough-2000-resin/) or [ABS Like](https://store.anycubic.com/products/abs-like-resin-pro-2) resin
* [PMW3389 Optical Sensor](https://www.tindie.com/products/citizenjoe/pmw3389-motion-sensor/)
* [Pro Micro (5 Volt/16 MHz)](https://deskthority.net/wiki/Arduino_Pro_Micro)
* [Custom Bridge Board & Middle Routing Board PCB](https://jlcpcb.com/?from=PyottDesign)
* M2 button head screws & M2.5 set screws
* M2 and M2.5 nuts
* 1/8" diameter aluminum tube stock
* Mouse Glides for Logitech G-Pro
* 2.0mm pitch PH type end connectors & right angle through hole board connectors
* (5) Mouse switches
* Push button momentary switch, 6mm x 6mm x 4.5mm
* 2.54mm headers, surface mount gull wing, cut to length
* [TTC Rotary Encoder](https://a.co/d/5zIg8kU)
* [NeoPixel 5050 RGB LED](https://www.adafruit.com/product/1655)
* ~6' Micro USB cable or Mouse cable w/ micro USB port connector component 
* 28 Ga. 6 conductor silicone ribbon wire (or similar)
* Soldering Iron, Multi-meter, [PH Crimping tool](https://a.co/d/hdylA0W) and other misc. tools

![Statial.b All Parts](img/statial-b_allparts.JPG)
<br/><br/>

# Configurations
Below are some common grip type configurations for the Statial.b as starting points for further adjustment. Mouse ergronomics are extreamly sensitive. A 1mm or 1 degree adjustment can take an uncomfortable setup to a great one (and vice versa). When using the Statial.b, try and think about what feels right and what feels off in your grip and make micro adjstments to single surfaces only until it's dialed in. 

### VARIABLE PALM
When fully collapsed the Statial.b is slightly smaller than a normal high performance mouse. Surfaces can be moved out significantly for larger hands.
![Statial.b palm grip](img/grip_palm-02.png)
![Statial.b palm grip](img/grip_palm-01.png)

### STUBNOSE CLAW
Unlike a normal mouse, the buttons on the Statial.b can pitch forward for different grip types. This allows claw grip users to position buttons perpendicular to the pressing motion.
![Statial.b claw grip](img/grip_claw-02.png)
![Statial.b claw grip](img/grip_claw-01.png)

### BACKLESS FINGER
Surfaces can be configured to experiment with alternate grip methods. For example; finger grip users can remove back surface entirely.
![Statial.b finger grip](img/grip_finger-02.png)
![Statial.b finger grip](img/grip_finger-01.png)

### ERGO VERTICAL
Surfaces can be configured to +40° angle to mimic the fit of a vertical mouse. Fixed ends of rear surface arms can be extended or longer sections of tube can be cut for even steeper angles. 
![Statial.b angle grip](img/grip_angle-02.png)
![Statial.b angle grip](img/grip_angle-01.png)
<br/><br/>

# Instructions
For complete instructions, please download the [Statial.b Instructions PDF](https://github.com/PyottDesign/Statial-b/statial-b_instructions_v01-00.pdf).

![Statial.b Instruction Manual](img/statial-b_inst_05.png)

![Statial.b Instruction Manual](img/statial-b_inst_07.png)

![Statial.b Instruction Manual](img/statial-b_inst_08.png)

![Statial.b Instruction Manual](img/statial-b_inst_10.png)

![Statial.b Instruction Manual](img/statial-b_inst_12.png)

![Statial.b Instruction Manual](img/statial-b_inst_13.png)

![Statial.b Instruction Manual](img/statial-b_inst_14.png)

![Statial.b Instruction Manual](img/statial-b_inst_15.png)

![Statial.b Instruction Manual](img/statial-b_inst_16.png)

![Statial.b Instruction Manual](img/statial-b_inst_17.png)

![Statial.b Instruction Manual](img/statial-b_inst_18.png)

![Statial.b Instruction Manual](img/statial-b_inst_21.png)

![Statial.b Instruction Manual](img/statial-b_inst_22.png)