# IGV1-16 Driver Backpack
This is a board that screws onto the back of an IGV1-16, GIPS-16 or Burroughs Self-Scan and handles the HV buffering between it and a RPi Pico W, among other things. 
![IMG_2801 2](https://user-images.githubusercontent.com/20519442/228124671-2401a1ef-7dea-4a35-b771-d1d10b69e3e6.jpeg)
![IMG_3037](https://user-images.githubusercontent.com/20519442/228124653-a4e316f6-7dc2-4522-b0a4-e72d010b065c.jpeg)

I made a thing to handle the mess of wiring from the [IGV1-16 Compass](https://github.com/Architeuthis-Flux/IGV1-16-Compass) but I had a ton of extra board space and GPIO so I ended up making a somewhat generalized development board for these plasma scan displays. 

![IGV1](https://user-images.githubusercontent.com/20519442/228126670-bead3f02-e34d-475b-a32a-090246437866.png)

Soon I'll be adding some code here that will be easier to use with whatever you end up doing with your plasma scan display. That other repo is just a small amount of display handling shoved into a huge mess of [USFSMAX sensor fusion module](https://github.com/gregtomasch/USFSMAX_MMC_Module/tree/master/MMC_USFS_MAX_Module_Dragonfly_Simple_Host_Utility_v0.0) code, and I'd like for people to be able to make this display show anything they like and not be confined to one specific use. 

![IMG_3045](https://user-images.githubusercontent.com/20519442/228124714-d6dfc560-bd8d-48ea-8e9d-c31dd5a8f394.jpeg)

For now, this is just the Kicad files for the hardware and some basic driving code for the Pico will be added in the next couple days. If you hve an idea for something cool to do with one of [these displays](https://www.ebay.com/itm/224643685691), let me know and I'll make sure you get a backpack board to play with. 

It uses one of these [cheap bipolar HV power supply modules](https://www.ebay.com/itm/295529826686) to generate the +280V and the (optional) -82V to drive the display. If you end up with one of these modules without the negative supply, there's a solder jumper on the board to instead drive the Blank line to ground. In which case it just needs to be held low a bit longer, giving you a bit less time to do other things between screen updates, but otherwise is totally fine.  


![IMG_3040](https://user-images.githubusercontent.com/20519442/228124697-1d9fe498-8b36-4f75-aac1-4d8e80a6ec45.jpeg)

![IMG_3049](https://user-images.githubusercontent.com/20519442/228124728-33367ec8-2250-4522-852b-82bedb05e8fc.jpeg)

