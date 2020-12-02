cmake_minimum_required(VERSION 3.13)
set(project_name dma_for_teensy)
project(${project_name})

set(CMAKE_CXX_STANDARD 14)

add_executable(${project_name}
        DmaTeensy4_0.ino
        DmaSerialTeensy.cpp DmaSerialTeensy.h

        libs/analog.c
        libs/Arduino.h
        libs/arm_common_tables.h
        libs/arm_const_structs.h
        libs/arm_math.h
        libs/AudioStream.cpp
        libs/AudioStream.h
        libs/avr_emulation.h
        libs/avr_functions.h
        libs/binary.h
        libs/bootdata.c
        libs/Client.h
        libs/clockspeed.c
        libs/core_cm7.h
        libs/core_cmInstr.h
        libs/core_id.h
        libs/core_pins.h
        libs/debugprintf.c
        libs/delay.c
        libs/digital.c
        libs/DMAChannel.cpp
        libs/DMAChannel.h
        libs/eeprom.c
        libs/elapsedMillis.h
        libs/EventResponder.cpp
        libs/EventResponder.h
        libs/HardwareSerial.cpp
        libs/HardwareSerial.h
        libs/HardwareSerial1.cpp
        libs/HardwareSerial2.cpp
        libs/HardwareSerial3.cpp
        libs/HardwareSerial4.cpp
        libs/HardwareSerial5.cpp
        libs/HardwareSerial6.cpp
        libs/HardwareSerial7.cpp
        libs/HardwareSerial8.cpp
        libs/imxrt.h
        libs/interrupt.c
        libs/IntervalTimer.cpp
        libs/IntervalTimer.h
        libs/IPAddress.cpp
        libs/IPAddress.h
        libs/keylayouts.c
        libs/keylayouts.h
        libs/math_helper.h
        libs/new.cpp
        libs/nonstd.c
        libs/pins_arduino.h
        libs/Print.cpp
        libs/Print.h
        libs/Printable.h
        libs/pwm.c
        libs/rtc.c
        libs/Server.h
        libs/startup.c
        libs/Stream.cpp
        libs/Stream.h
        libs/tempmon.c
        libs/Tone.cpp
        libs/Udp.h
        libs/usb.c
        libs/usb_desc.c
        libs/usb_desc.h
        libs/usb_dev.h
        libs/usb_inst.cpp
        libs/usb_names.h
        libs/usb_rawhid.c
        libs/usb_rawhid.h
        libs/usb_seremu.c
        libs/usb_seremu.h
        libs/usb_serial.c
        libs/usb_serial.h
        libs/WCharacter.h
        libs/wiring.h
        libs/wiring_private.h
        libs/WMath.cpp
        libs/WProgram.h
        libs/WString.cpp
        libs/WString.h
        libs/yield.cpp

        )

add_definitions(-D__IMXRT1062__ -DUSB_SERIAL -DTEENSYDUINO=148 -DF_CPU=180000000 -D__cplusplus)
include_directories(libs)
