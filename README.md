### Ardunio Hardware Support Package for LGT8F's 
##

### Task status

- [x] PWM & Timers update
- [x] Fast_IO update
- [ ] Differential Amplifier update 

### Summary
Larduino_HSP is 3'rd party hardware support package for LGT8F core based arduino boards.<br>
The core of HSP is fork from arduino distribution. We have try best to keep all the standard <br>features compatible with current arduino world. So you can feel free to try resuse all of <br> the libraries which designed for arduino platform.<br><br>
LGT8F's core based microcontroller, e.g LGT8F328P has more advanced features which not <br>
covered in standard arduino implementation. so there are also many new features merged to this<br> 3'rd party package. Important update and new features as following:<br>

* *External/Internal crystal can be selected at runtime*
* *Fast_IO interface for fast I/O speed and small code size*
* *Full configurable PWM features, including complementary and dead-band control*
* *Differential Amplifier front-end for 12bit Analog-to-Digital converter*
* *1/2 channel 8bit Digtial-to-Analog output, campatible with `analogWrite()`*
* *More standalone I/Os*

###### *Fast_IO code snippets
	void setup() {
		//set D10 to output
		fastioMode(D10, OUTPUT);

		// driver D10 to low level
		fastioWrite(D10, LOW);
	}

	void loop() {
		// toggle D10
		fastioToggle(D10);
	}

###### *PWM code snippets
	void setup() {
		// usage: pwmMode(pin, pwm_mode, freq_mode)
		// PWM_MODE_SOLO: set pwm of D5/D6 to independent mode
		// PWM_FREQ_FAST: set PWM to fast mode 
		pwmMode(D5, PWM_MODE_SOLO, PWM_FREQ_FAST);

		// usage: pwmResolution(pin, res_bits)
		pwmResolution(D5, 7);
		// or we can set PWM frequency directly
		// usage: pwmFrequency(pin, freq_in_hz)
		// pwmFrequency(D5, 500000);
 
		// usage: pwmWrite(pin, duty)
		pwmWrite(D5, 128 >> 1);
		pwmWrite(D6, 128 >> 2);
	}


> More detail will be documented in comming Wiki page

### Bootloader 
Larduino HSP is based on optiboot. Source can be found inside [bootloader directory](https://github.com/LGTMCU/Larduino_HSP/tree/master/hardware/LGT/avr/bootloaders/lgt8fx8p)

* *Bootloader upload baudrate: 57600bps*
	
### Installation:
1. Unzip Larduino_HSP_vX.X.rar
1. Copy [**sketches**], [**hardware**] and [**libraries**] directories to arduino's sketchbook direcotry
1. Restart Arduino, you will see new board from [**Tools**]->[**Border**] menu.

### About arduino's sketchbook directory

You can always find this directory from **[File]->[Preferences]** menu.
Here is the default sketchbook directory for most popluar system:

| System | Path of sketchbook |
| :----- | :----------------- |
| Windows | C:\Users\<Username>\Documents\Arduino |
| Mac OSX | /Users/user/Documents/Arduino |
| LINUX | /home/<Username\>/sketchbook |

### Arduino Board based on LGT8F's
Here we list one remarkable arduino board based on LGT8F's core, which is designed by **[OCROBOT](http://www.ocrobot.com/doku.php?id=zh:start)**<br>
[ALPHA 8F382P-U](http://www.ocrobot.com/doku.php?id=zh:ocrobot:alpha:8f328p-u:main) stick, with USB to UART on board. You can afford it just of **RMB 8.00**. <br>
![](http://www.ocrobot.com/lib/exe/fetch.php?w=400&tok=4f133f&media=zh:ocrobot:alpha:8f328p-u:328p-u%E4%BE%A7%E9%9D%A2435.png)<br>
Please follow **[OCROBOT ALPHA 8F832P-U](http://www.ocrobot.com/doku.php?id=zh:ocrobot:alpha:8f328p-u:main)** official page for more details.