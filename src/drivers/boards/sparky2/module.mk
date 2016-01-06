#
# Board-specific startup code for the PX4FMU
#

SRCS		 = sparky2_can.c \
		   sparky2_init.c \
		   sparky2_pwm_servo.c \
		   sparky2_spi.c \
		   sparky2_usb.c \
		   sparky2_led.c

MAXOPTIMIZATION	 = -Os
