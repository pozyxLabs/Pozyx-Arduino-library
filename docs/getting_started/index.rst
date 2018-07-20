Getting started
---------------

.. note::

    In this text, the word tag and shield will be used interchangeably.

Arduino
~~~~~~~~~~~~~~~~~~~~~~~~

If you're not familiar with the Arduino platform as a whole, `this <https://www.arduino.cc/en/Guide/HomePage>`_ is a good place to get started and set up with testing your Arduino's basic functionality.


Setup
~~~~

Role of Arduino
_______________

The Arduino communicates to the Pozyx device using the I2C protocol. 

Something that has been confusing to a lot of people is how to set up the Pozyx on Arduino, especially
when multiple devices come into play.

And with the additional existence of the `Python library that provides direct USB access to the Pozyx <pypozyx.rtfd.io>`_,
we have even seen people trying to use the Python library with an Arduino which had a
Pozyx shield attached, which of course did not work.

In this section, we intend to give you insight of the hardware you need!

Pozyx on Arduino
________________

The only Pozyx device that needs an Arduino is the one you're running the positioning/ranging sketches on.
This device is called the master device, as this will also direct the operation of other Pozyx devices through
UWB communication.

Remote Pozyx devices on Arduino
_______________________________

Remote Pozyx tags don't need to have an Arduino attached! This is an important point. They just need to be powered.
Phone powerbanks play well with the micro USB port on the tags!

Only perform positioning and ranging functions on your master device.

.. note::

    You might want to have an Arduino on remote shields if you want to read their sensor data locally.
    However, do not use functions like positioning and ranging on multiple devices at the same time!

    An example is the Cloud example where an Arduino is used to read the tag's position. 
    In this example, the positioning is directed by the Pozyx attached to the server, and the Arduino
    checks the tag's status to see whether a new position has been calculated.

Pozyx via USB
_______________

Instead of using an Arduino locally, you can skip the Arduino and use an USB cable directly.

This has advantages like:

* Very cross platform serial protocol.
* Easy to use Python programming language for flexible functionality (and extendability).
* Computer can be as powerful as you want.
* You can implement the serial communication in any other programming language.

But disadvantages too:

* Arduino is very cheap and small standalone hardware, compared to a Raspberry Pi or regular PC.
* The new Arduino Web IDE is amazing.
* You're comfortable with Arduino programming but not with Python.

Ultimately, the decision which you want to use depends on your application and your
available hardware.

The documentation for using USB directly can be found `here <pypozyx.rtfd.io>`_.

Required headers
~~~~~~~~~~~~~~~~

To use the Pozyx library, you have to include the following headers:

.. code-block:: cpp

    #include <Pozyx.h>
    #include <Pozyx_definitions.h>


Connecting to the Pozyx
~~~~~~~~~~~~~~~~~~~~~~~

Connecting with the Pozyx is very straightforward. A safe way is presented here:

.. code-block:: cpp

   void setup(){
        Serial.begin(115200);
        if(Pozyx.begin() == POZYX_FAILURE){
            Serial.println(F("ERROR: Unable to connect to POZYX shield"));
            Serial.println(F("Reset required"));
            delay(100);
            abort();
        }
    }

With this, you initialize the Pozyx and can use the entire API in the rest of your script!

General philosophy
~~~~~~~~~~~~~~~~~~

Essentially, you can do three things with Pozyx:

1. Reading register data, which includes sensors and the device's configuration
2. Writing data to registers, making it possible to change the device's configuration ranging from its positioning algorithm to its very ID.
3. Performing Pozyx functions like ranging, positioning, saving the device's configuration to its flash memory...

All these things are possible to do on the shield connected to your Arduino, and powered remote devices as well. In this section we'll go over all of these.

Reading data
~~~~~~~~~~~~

To read data from the Pozyx, a simple pattern is followed. This pattern can be used with almost all methods starting with the words 'get':

1. Initialize the appropriate container for your data read.
2. Pass this container along with the get functions.
3. Check the status to see if the operation was successful and thus the data trustworthy.

You can see the same pattern in action above when reading the UWB data.

.. TODO An overview of all data containers, their usage and their particularities can be found here:

.. TODO also mention that they all have human readable __str__ conversions

.. code-block:: cpp

    // initialize the data container
    uint8_t who_am_i;
    uint8_t status = Pozyx.getWhoAmI(&whoami);

    // check the status to see if the read was successful. Handling failure is covered later.
    if (status == POZYX_SUCCESS) {
      // print the container. Note how a SingleRegister will print as a hex string by default.
      Serial.println(who_am_i); // will print '0x43'
    }

    # and repeat
    # initialize the data container
    acceleration_t acceleration;
    # get the data, passing along the container
    Pozyx.getAcceleration_mg(&acceleration);

    # initialize the data container
    euler_angles_t euler_angles;
    # get the data, passing along the container
    Pozyx.getEulerAngles_deg(&euler_angles)


Writing data
~~~~~~~~~~~~

Writing data follows a similar pattern as reading, but making a container for the data is optional. This pattern can be used with all methods starting with the words 'set':

1. (Optional) Initialize the appropriate container with the right contents for your data write.
2. Pass this container or the right value along with the set functions.
3. Check the status to see if the operation was successful and thus the data written.

Some typical write operations

.. code-block:: cpp

   # initialize Pozyx as above

   uint8_t status = Pozyx.setPositionAlgorithm(POZYX_POS_ALG_UWB_ONLY);
   // Note: this shouldn't ever happen when writing locally.
   if (status == POZYX_FAILURE) {
     Serial.println("Settings the positioning algorithm failed");
   }

   Pozyx.setPositioningFilter(FILTER_TYPE_MOVING_AVERAGE, 10);


Performing functions
~~~~~~~~~~~~~~~~~~~~

Positioning, ranging, configuring the anchors for a tag to use... While the line is sometimes thin,
these aren't per se writes or reads as they are implemented as functions on the Pozyx.

A Pozyx device function typically can take a container object for storing the function's
return data, and a container object for the function parameters.

For example, when adding an anchor to a tag's device list, the anchor's ID and position
are the function's parameters, but there is no return data. Thus, the function addDevice
only needs a container object containing the anchor's properties.

In the library, function wrappers are written in such a way that when no parameters are
required, they are hidden from the user, and the same goes for return data.

.. code-block:: cpp

    // assume an anchor 0x6038 that we want to add to the device list and
    // immediately save the device list after.
    device_coordinates_t anchor;
    anchor.network_id = 0x6038;
    anchor.flag = 0x1;
    anchor.pos.x = 5000;
    anchor.pos.y = 5000;
    anchor.pos.z = 0;
    Pozyx.addDevice(anchor);

.. TODO find better example than positioning since that's a lie

Remote
~~~~~~

To interface with a remote device, every function has a remote_id optional parameter. Thus, every function you just saw can be performed on a remote device as well!

(The exceptions are doPositioning and doRanging, which have doRemotePositioning and doRemoteRanging)

.. code-block:: cpp

    // let's assume there is another tag present with ID 0x6039
    uint16_t remote_device_id = 0x6039;

    // this will read the WHO_AM_I register of the remote tag
    uint8_t who_am_i;
    Pozyx.getWhoAmI(&whoami, remote_device_id);


Saving writable register data
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Basically, every register you can write data to as a user can be saved in the device's flash memory. This means that when the device is powered on, its configuration will remain. Otherwise, the device will use its default values again.

.. TODO add default values for registers so that users know what to expect.

This is useful for multiple things:

* Saving the UWB settings so all your devices remain on the same UWB settings.
* Saving the anchors the tag uses for positioning. This means that after a reset, the tag can resume positioning immediately and doesn't need to be reconfigured!
* Saving positioning algorithm, dimension, filter... you'll never lose your favorite settings when the device shuts down.

There are various helpers in the library to help you save the settings you prefer, not requiring you to look up the relevant registers.

.. 
    .. code-block:: python
        # Saves the positioning settings
        pozyx.savePositioningSettings()
        # Saves the device list used for positioning
        pozyx.saveNetwork()
        # Saves the device's UWB settings
        pozyx.saveUWBSettings()


Finding out the error
~~~~~~~~~~~~~~~~~~~~~

Pozyx functions typically return a status to indicate the success of the function. This is useful to indicate failure especially. When things go wrong, it's advised to read the error as well.

A code snippet shows how this is typically done

.. 
    .. code-block:: python
        from pypozyx import PozyxSerial, get_first_pozyx_serial_port, POZYX_SUCCESS, SingleRegister
        # initialize Pozyx as above
        if pozyx.saveUWBSettings() != POZYX_SUCCESS:
            # this is one way which retrieves the error code
            error_code = SingleRegister()
            pozyx.getErrorCode(error_code)
            print('Pozyx error code: %s' % error_code)
            # the other method returns a descriptive string
            print(pozyx.getSystemError())
