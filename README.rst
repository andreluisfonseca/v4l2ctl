************
v4l2ctl
************
A simple, libv4l2-based frames capture library.
The `v4l2ctl` module based pyv4l2  development of pupil-labs:

* Full access to all uvc settings (Zoom,Focus,Brightness,etc.)
* Full access to all stream and format parameters (rates,sizes,etc.)
* Enumerate all capture devices with list_devices()

I removed the  some partes  of the code: I removed  [libjpegturbo] (http://libjpeg-turbo.virtualgl.org/)  libary,
the most original data is same. v4l2ctl module in `https://github.com/pupil-labs/pyv4l2`, Thank you to share the code
pupil-labs.

============
Installation
============
+++++++
Libv4l2
+++++++
Libv4l2 is packaged by various distributions:

-----------------
Debian and Ubuntu
-----------------
.. code-block:: bash

    # apt-get install libv4l-dev

------
Fedora
------
.. code-block:: bash

    # dnf install libv4l-devel

----------
Arch Linux
----------
.. code-block:: bash

    # pacman -S v4l-utils

++++++
v4l2ctl
++++++

v4l2ctl is only compatible with Python 3.

To install v4l2ctl:

###dependecy cython
.. code-block:: bash

    # sudo pip3 install cython

### just build locally
.. code-block:: bash

    # sudo python3 setup.py build

### or install system wide
.. code-block:: bash

    # sudo python setup.py install


=====
Usage
=====
.. code-block:: python

   # List alls devices
    import v4l2ctl
    v4l2ctl.list_devices()

    from v4l2ctl import Frame
    from v4l2ctl import V4l2
     
   # Capture Code
    frame = Frame('/dev/video0')
    frame_data = frame.get_frame()

   # Get info of device
    control.get_info()

   # Control Code
    control = V4l2('/dev/video0')

   # List all controls
    control.get_controls()

   # alter controls by id
    control.get_control(9963776)
    control.set_control(9963776, 8)

   # List formats
    control.transport_formats

   # Set format
    control.transport_format = 'H264'

   # List resolutions
    control.frame_sizes

   # Set resolution
    control.frame_size = (800, 448)

   # List frame rates
    control.frame_rates

   # Set frame rate
    control.frame_rates = (1, 30)

