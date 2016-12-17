pi-rc
=====

pi-rc is a program that turns your Raspberry Pi into a radio controller for RC
cars without any additional hardware. With it, you should be able to drive just
about any cheap toy-grade RC car programmatically with your Raspberry Pi.

Note that running this program turns your Pi into a rather powerful radio
transmitter. Do not use this code unless you understand what it does, what
frequencies it might interfere with, what frequencies are allowed, what is
legal in your country, etc. I assume no responsibility for your actions or for
any problems running this program may cause.

Getting started
---------------

pi-rc should work with most toy-grade RC cars, i.e. ones that only drive
forward, backward, left, and right. To get started, place a jumper cable on
GPIO pin 4 (that's pin 7 on the revision B board, see [this
diagram](http://upload.wikimedia.org/wikipedia/commons/9/97/Raspberrypi_pcb_overview_Pinout_v01.svg)
to act as an antenna and compile the `pi_pcm` program on your Raspberry Pi and
run it as root:

    apt-get -y install scons libjansson-dev
    scons
    su
    ./pi_pcm

If you get an error like

    fatal error: jansson.h: No such file or directory compilation terminated.

you'll need to install the Jansson JSON-parsing library. Run:

    apt-get install libjansson-dev

and then try compiling again.

Every RC car I've seen uses a different set of command codes, so you'll need to
run another program that iterates through possible command codes and watches
the RC car to see if it responds. Once it does, it will save the image and the
command that caused the car to move.

To run this search, you'll need a computer with a webcam with `streamer`
installed, or a Raspberry Pi with a camera module. To install streamer, run:

    apt-get install streamer

You can run this search program on the Raspberry Pi or a separate computer.
Place the car in an area where you can control the lighting and avoid changes
in ambient lighting, such as a closet, and turn the car on. If your car is
darkly colored, try to place it against a white background, for example by
putting a piece of paper behind it. Point the webcam at it and run:

    python watch.py -f [frequency] -s [Pi IP address]

where frequency is the radio control frequency of the RC car. Most toy-grade RC
cars run in the 27 or 49 MHz band, which should be printed on the car. Most
cars run in a specific frequency in the 27 or 49 MHz band, such as 27.255; if
you don't know the car's exact frequency, the program will incrementally search
all of them if you specify 27 or 49. Once the program sees the car move, it
will save the image of the car moving with a file named with the parameters
of the command that it just broadcast.

Now that you have the basic command structure, you can search for the specific
commands to make it drive. Run

    python control.py -f [frequency] -s [Pi IP address]

The program will prompt you for the values found from the search, and then ask
you repeatedly to enter different signal bursts. Try entering values from 1-100
and see how the car reacts.  Certain values should make the car drive some
combination of forward/reverse + left/right.

Once you have all of the signals, you can save the information in a JSON file.
See `control-specs/pro-dirt.json` for an example. Now you should be able to run

    python interative_control.py -s [Pi IP address] [JSON control file]

to drive the car using the arrow keys.

Signals
-------

[How Stuff Works](http://electronics.howstuffworks.com/rc-toy2.htm) has a good
explanation of what the command signals for toy RC cars look like. One thing to
note is that the Raspberry Pi cannot reliably stop broadcasting for a certain
amount of time, so it instead broadcasts at a different frequency to simulate
the pauses.

Programming
-----------

That's fun and all, but the real fun in my mind comes from buying an external
battery pack for the Raspberry Pi, taping it to the top of the car, and turning
it into an autonomous vehicle. I've tried to make it easy to send commands to
the controller program.

The `pi_pcm` program listens for JSON-formatted commands over UDP port 12345 by
default. I chose UDP because I figured most commands would be sent locally from
the Raspberry Pi anyway, and you can just repeatedly send commands to it, say
10 times a second; the car will continue to respond to the last message
received.

Messages are formatted as an array of objects with the following fields defined:

<table>
    <tr>
        <td>frequency</td>
        <td>The frequency that the car runs at.</td>
    </tr>
    <tr>
        <td>dead_frequency</td>
        <td>
            Another frequency to simulate a "pause" in broadcasting.  If you
            are in the United States and your car is running in the 27 MHz
            band, 49.890 MHz should be a safe signal here, and for cars in 49
            MHz, try 26.995. DO NOT use frequencies outside of the normal range
            for toy RC cars, and make sure you comply with all regulations in
            your country.
        </td>
    </tr>
    <tr>
        <td>burst_us</td>
        <td>The length of a signal burst in microseconds.</td>
    </tr>
    <tr>
        <td>spacing</td>
        <td>The length of a signal spacing in microseconds.</td>
    </tr>
    <tr>
        <td>repeats</td>
        <td>The number of times to broadcast the message.</td>
    </tr>
</table>

A typical command might look like:

    [
        {
            "frequency": 26.995,
            "dead_frequency": 49.830,
            "burst_us": 1200,
            "spacing_us": 400,
            "repeats": 4
        },
        {
            "frequency": 26.995,
            "dead_frequency": 49.830,
            "burst_us": 400,
            "spacing_us": 400,
            "repeats": 40
        }
    ]

where the first object is the synchronization signal and the second is the
command signal.  The Pi will continue broadcasting a given command until a new
command is received. If you want to stop the car from moving, try sending a
signal repeat that doesn't correspond to any action from the RC car.

I chose to use UDP for listening for commands. If you're worried about messages
being dropped, you can include the parameter

    "request_response": true

in one of your objects. The server will send a response message over UDP to the
port one above from the port that the client sent the message from.
