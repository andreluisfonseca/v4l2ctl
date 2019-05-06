from v4l2 cimport *
from posix cimport fcntl,unistd, stat, time
from libc.errno cimport errno, EINTR, EINVAL
from libc.string cimport memset, memcpy, strerror
from libc.stdlib cimport malloc, calloc, free
from typing import Any, List
from posix.fcntl cimport O_RDWR
from posix.ioctl cimport ioctl
from libc.errno cimport errno,EINTR,EINVAL,EAGAIN,EIO,ERANGE
from libc.string cimport strerror
from posix.select cimport fd_set, timeval, FD_ZERO, FD_SET, select
from posix.mman cimport PROT_READ, PROT_WRITE, MAP_SHARED

from os import listdir as oslistdir

import cython
cimport numpy as np
import numpy as np


class CameraError(Exception):
    pass

def list_devices():
    """
    List all devices of ambient

    """
    file_names = [x for x in oslistdir("/dev") if x.startswith("video")]
    file_names.sort()
    devices = []
    for file_name in file_names:
        path = "/dev/" + file_name
        try:
            cap = Info(path)
            devices.append(cap.get_info())
            cap.close()
        except IOError:
            print("Could not get device info for %s"%path)
    return devices


cdef class Info:
    """
    class used to get information of device.

    """
    cdef int fd 
    cdef bytes dev_name

    def __cinit__(self,dev_name):
        self.dev_name = dev_name.encode()
        self.fd = self.open_device()

    def close(self):
        self.close_device()

    def __dealloc__(self):
        if self.fd != -1:
            self.close()

    def get_info(self):
        cdef v4l2_capability caps
        if self.xioctl(VIDIOC_QUERYCAP,&caps) !=0:
            raise Exception("VIDIOC_QUERYCAP error. Could not get devices info.")

        return {'dev_path':self.dev_name.decode(),'driver':caps.driver.decode(),
                'dev_name':caps.card.decode(),'bus_info':caps.bus_info.decode()}

    cdef xioctl(self, int request, void *arg):
        cdef int r
        while True:
            r = ioctl(self.fd, request, arg)
            if -1 != r or EINTR != errno:
                break
        return r

    cdef open_device(self):
        dev_handle = v4l2_open(self.dev_name, O_RDWR)
        if -1 == self.fd:
            raise CameraError('Error opening device {}'.format(self.dev_name))

        return dev_handle

    cdef close_device(self):
        v4l2_close(self.fd)
        self.fd = -1


cdef class Frame:
    """
    class used to get Frames of device.

    """

    cdef int fd
    cdef fd_set fds

    cdef v4l2_format fmt

    cdef v4l2_requestbuffers buf_req
    cdef v4l2_buffer buf
    cdef buffer_info *buffers

    cdef timeval tv

    def __cinit__(self, device_path):
        device_path = device_path.encode()

        self.fd = v4l2_open(device_path, O_RDWR)
        if -1 == self.fd:
            raise CameraError('Error opening device {}'.format(device_path))

        memset(&self.fmt, 0, sizeof(self.fmt))
        self.fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE

        if -1 == xioctl(self.fd, VIDIOC_G_FMT, &self.fmt):
            raise CameraError('Getting format failed')

        if -1 == xioctl(self.fd, VIDIOC_S_FMT, &self.fmt):
            raise CameraError('Setting format failed')



        memset(&self.buf_req, 0, sizeof(self.buf_req))
        self.buf_req.count = 4
        self.buf_req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE
        self.buf_req.memory = V4L2_MEMORY_MMAP

        if -1 == xioctl(self.fd, VIDIOC_REQBUFS, &self.buf_req):
            raise CameraError('Requesting buffer failed')

        self.buffers = <buffer_info *>calloc(self.buf_req.count,
                                             sizeof(self.buffers[0]))
        if self.buffers == NULL:
            raise CameraError('Allocating memory for buffers array failed')
        self.initialize_buffers()

        if -1 == xioctl(self.fd, VIDIOC_STREAMON, &self.buf.type):
            raise CameraError('Starting capture failed')

    cdef inline int initialize_buffers(self) except -1:
        for buf_index in range(self.buf_req.count):
            memset(&self.buf, 0, sizeof(self.buf))
            self.buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE
            self.buf.memory = V4L2_MEMORY_MMAP
            self.buf.index = buf_index

            if -1 == xioctl(self.fd, VIDIOC_QUERYBUF, &self.buf):
                raise CameraError('Querying buffer failed')

            bufptr = v4l2_mmap(NULL, self.buf.length,
                               PROT_READ | PROT_WRITE,
                               MAP_SHARED, self.fd, self.buf.m.offset)

            if bufptr == <void *>-1:
                raise CameraError('MMAP failed: {}'.format(
                    strerror(errno).decode())
                )

            self.buffers[buf_index] = buffer_info(bufptr, self.buf.length)

            memset(&self.buf, 0, sizeof(self.buf))
            self.buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE
            self.buf.memory = V4L2_MEMORY_MMAP
            self.buf.index = buf_index

            if -1 == xioctl(self.fd, VIDIOC_QBUF, &self.buf):
                raise CameraError('Exchanging buffer with device failed')

        return 0


    cpdef bytes get_frame(self):
        FD_ZERO(&self.fds)
        FD_SET(self.fd, &self.fds)

        self.tv.tv_sec = 2

        r = select(self.fd + 1, &self.fds, NULL, NULL, &self.tv)
        while -1 == r and errno == EINTR:
            FD_ZERO(&self.fds)
            FD_SET(self.fd, &self.fds)

            self.tv.tv_sec = 2

            r = select(self.fd + 1, &self.fds, NULL, NULL, &self.tv)

        if -1 == r:
            raise CameraError('Waiting for frame failed')

        memset(&self.buf, 0, sizeof(self.buf))
        self.buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE
        self.buf.memory = V4L2_MEMORY_MMAP

        if -1 == xioctl(self.fd, VIDIOC_DQBUF, &self.buf):
            raise CameraError('Retrieving frame failed')

        buf =  self.buffers[self.buf.index].start
        buf_len = self.buf.bytesused

        frame_data = <unsigned char *>malloc(buf_len * sizeof(char))
        try:
            memcpy(frame_data, buf, buf_len)
            if -1 == xioctl(self.fd, VIDIOC_QBUF, &self.buf):
                raise CameraError('Exchanging buffer with device failed')
            return frame_data[:buf_len]
        finally:
            free(frame_data)

    @property
    def fd(self):
        return self.fd

    def close(self):
        xioctl(self.fd, VIDIOC_STREAMOFF, &self.buf.type)

        for i in range(self.buf_req.count):
            v4l2_munmap(self.buffers[i].start, self.buffers[i].length)

        v4l2_close(self.fd)


cdef class V4l2:
    """
    Video Control class.
    A Class giving access to devices information using the v4l2 provides drivers and userspace API.
    All controls are exposed and can be enumerated using the controls list.
    """
    cdef int dev_handle
    cdef bytes dev_name
    cdef bint _camera_streaming, _buffers_initialized
    cdef object _transport_formats, _frame_rates,_frame_sizes
    cdef object  _frame_rate, _frame_size # (rate_num,rate_den), (width,height)
    cdef __u32 _transport_format

    cdef bint _buffer_active
    cdef int _allocated_buf_n
    cdef v4l2_buffer _active_buffer
    cdef list buffers

    def __cinit__(self,dev_name):
        pass

    def __init__(self,dev_name):
        self.dev_name = dev_name.encode()
        self.dev_handle = self.open_device()
        self.verify_device()
        self.get_format()
        self.get_streamparm()

        self._transport_formats = None
        self._frame_rates = None
        self._frame_sizes = None

        self._buffer_active = False
        self._allocated_buf_n = 0

        self._buffers_initialized = False
        self._camera_streaming = False

        #set some sane defaults:
        #self.transport_format = "MJPG"

    def restart(self):
        self.close()
        self.dev_handle = self.open_device()
        self.verify_device()
        #self.transport_format = "MJPG" #this will set prev parms
        print("restarted capture device")

    def close(self):
        try:
            self.stop()
            self.close_device()
        except:
            print("Could not shut down Capture properly.")

    def __dealloc__(self):
        if self.dev_handle != -1:
            self.close()

    def get_info(self):
        cdef v4l2_capability caps
        if self.xioctl(VIDIOC_QUERYCAP,&caps) !=0:
            raise Exception("VIDIOC_QUERYCAP error. Could not get devices info.")

        return  {'dev_path':self.dev_name.decode(),'driver':caps.driver.decode(),
                 'dev_name':caps.card.decode(),'bus_info':caps.bus_info.decode()}


    cdef xioctl(self, int request, void *arg):
        cdef int r
        while True:
            r = ioctl(self.dev_handle, request, arg)
            if -1 != r or EINTR != errno:
                break
        return r

    cdef open_device(self):
        cdef stat.struct_stat st
        cdef int dev_handle = -1
        if -1 == stat.stat(<char *>self.dev_name, &st):
            raise Exception("Cannot find '%s'. Error: %d, %s\n"%(self.dev_name, errno, strerror(errno) ))
        if not stat.S_ISCHR(st.st_mode):
            raise Exception("%s is no device\n"%self.dev_name)

        dev_handle = fcntl.open(<char *>self.dev_name, fcntl.O_RDWR | fcntl.O_NONBLOCK, 0)
        if -1 == dev_handle:
            raise Exception("Cannot open '%s'. Error: %d, %s\n"%(self.dev_name, errno, strerror(errno) ))
        return dev_handle

    cdef close_device(self):
        if not self.dev_handle == -1:
            if unistd.close(self.dev_handle) == -1:
                raise Exception("Could not close device. Handle: '%s'. Error: %d, %s\n"%(self.dev_handle, errno, strerror(errno) ))
            self.dev_handle = -1

    cdef verify_device(self):
        cdef v4l2_capability cap
        if self.xioctl(VIDIOC_QUERYCAP, &cap) ==-1:
            if EINVAL == errno:
                raise Exception("%s is no V4L2 device\n"%self.dev_name)
            else:
                raise Exception("Error during VIDIOC_QUERYCAP: %d, %s"%(errno, strerror(errno) ))

        if not (cap.capabilities & V4L2_CAP_VIDEO_CAPTURE):
            raise Exception("%s is no video capture device"%self.dev_name)

        if not (cap.capabilities & V4L2_CAP_STREAMING):
            raise Exception("%s does not support streaming i/o"%self.dev_name)

    cdef stop(self):
        cdef v4l2_buf_type buf_type
        if self._camera_streaming:
            buf_type = V4L2_BUF_TYPE_VIDEO_CAPTURE
            if self.xioctl(VIDIOC_STREAMOFF,&buf_type) == -1:
                self.close_device()
                raise Exception("Could not deinit buffers.")

            self._camera_streaming = False
            print("Control stopped.")

    cdef start(self):
        cdef v4l2_buffer buf
        cdef v4l2_buf_type buf_type
        if not self._camera_streaming:
            for i in range(len(self.buffers)):
                buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE
                buf.memory = V4L2_MEMORY_MMAP
                buf.index = i
                if self.xioctl(VIDIOC_QBUF, &buf) == -1:
                    raise Exception('VIDIOC_QBUF')

            buf_type = V4L2_BUF_TYPE_VIDEO_CAPTURE
            if self.xioctl(VIDIOC_STREAMON, &buf_type) ==-1:
                raise Exception("VIDIOC_STREAMON")
            self._camera_streaming = True
            print("Control started.")

    cdef set_format(self):
        cdef v4l2_format  format
        format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE
        format.fmt.pix.width       = self.frame_size[0]
        format.fmt.pix.height      = self.frame_size[1]
        format.fmt.pix.pixelformat = self._transport_format


        format.fmt.pix.field       = V4L2_FIELD_ANY
        if self.xioctl(VIDIOC_S_FMT, &format) == -1:
            self.close()
            raise Exception("Could not set v4l2 format")

    cdef set_streamparm(self):
        cdef v4l2_streamparm streamparm
        streamparm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE
        streamparm.parm.capture.timeperframe.numerator = self.frame_rate[0]
        streamparm.parm.capture.timeperframe.denominator = self.frame_rate[1]
        if self.xioctl(VIDIOC_S_PARM, &streamparm) == -1:
            self.close()
            raise Exception("Could not set v4l2 parameters")

    cdef get_format(self):
        cdef v4l2_format format
        format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE
        if self.xioctl(VIDIOC_G_FMT, &format) == -1:
            self.close()
            raise Exception("Could not get v4l2 format")
        else:
            self._frame_size = format.fmt.pix.width,format.fmt.pix.height
            self._transport_format = format.fmt.pix.pixelformat

    cdef get_streamparm(self):
        cdef v4l2_streamparm streamparm
        streamparm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE
        if self.xioctl(VIDIOC_G_PARM, &streamparm) == -1:
            self.close()
            raise Exception("Could not get v4l2 parameters")
        else:
            self._frame_rate = streamparm.parm.capture.timeperframe.numerator,streamparm.parm.capture.timeperframe.denominator

    property transport_formats:
        def __get__(self):
            cdef v4l2_fmtdesc fmt
            if self._transport_formats is None:
                fmt.type =  V4L2_BUF_TYPE_VIDEO_CAPTURE
                fmt.index = 0
                formats = []
                while self.xioctl(VIDIOC_ENUM_FMT,&fmt)>=0:
                    formats.append( fourcc_string(fmt.pixelformat ) )
                    fmt.index += 1
                print("Reading Transport formats: %s"%formats)
                self._transport_formats = formats
            return self._transport_formats

        def __set__(self,val):
            raise Exception("Read Only")

    property frame_sizes:
        def __get__(self):
            cdef  v4l2_frmsizeenum frmsize
            if self._frame_sizes is None:
                frmsize.pixel_format = fourcc_u32(self.transport_format.encode())
                frmsize.index = 0
                sizes = []
                while self.xioctl(VIDIOC_ENUM_FRAMESIZES, &frmsize) >= 0:
                    if frmsize.type == V4L2_FRMSIZE_TYPE_DISCRETE:
                        sizes.append((frmsize.discrete.width,frmsize.discrete.height))
                    elif frmsize.type == V4L2_FRMSIZE_TYPE_STEPWISE:
                        sizes.append( (frmsize.stepwise.max_width, frmsize.stepwise.max_height) )
                    frmsize.index+=1
                print("Reading frame sizes@'%s': %s"%(self.transport_format,sizes) )
                self._frame_sizes = sizes

            return self._frame_sizes

        def __set__(self,val):
            raise Exception("Read Only")

    property frame_rates:
        def __get__(self):
            cdef v4l2_frmivalenum interval

            if self._frame_rates is None:
                interval.pixel_format = fourcc_u32(self.transport_format.encode())
                interval.width,interval.height = self.frame_size
                interval.index = 0
                self.xioctl(VIDIOC_ENUM_FRAMEINTERVALS,&interval)
                rates = []
                if interval.type == V4L2_FRMIVAL_TYPE_DISCRETE:
                    while self.xioctl(VIDIOC_ENUM_FRAMEINTERVALS,&interval) >= 0:
                        rates.append((interval.discrete.numerator,interval.discrete.denominator))
                        interval.index += 1
                #non-discreete rates are very seldom, the second and third case should never happen
                elif interval.type == V4L2_FRMIVAL_TYPE_STEPWISE or interval.type == V4L2_FRMIVAL_TYPE_CONTINUOUS:
                    minval = float(interval.stepwise.min.numerator)/interval.stepwise.min.denominator
                    maxval = float(interval.stepwise.max.numerator)/interval.stepwise.max.denominator
                    if interval.type == V4L2_FRMIVAL_TYPE_CONTINUOUS:
                        stepval = 1
                    else:
                        stepval = float(interval.stepwise.step.numerator)/interval.stepwise.step.denominator
                    rates = range(minval,maxval,stepval)
                print("Reading frame rates@'%s'@%s: %s"%(self.transport_format,self.frame_size,rates) )
                self._frame_rates = rates

            return self._frame_rates

        def __set__(self,val):
            raise Exception("Read Only")


    property transport_format:
        def __get__(self):
            return fourcc_string(self._transport_format)

        def __set__(self,val):
            self._transport_format = fourcc_u32(val.encode())
            self.stop()
            self.set_format()
            self.get_format()
            self.set_streamparm()
            self.get_streamparm()
            self._frame_sizes = None
            self._frame_rates = None

    property frame_size:
        def __get__(self):
            return self._frame_size
        def __set__(self,val):
            self._frame_size = val
            self._frame_rates = None
            self.stop()
            self.set_format()
            self.get_format()
            self.set_streamparm()
            self.get_streamparm()

    property frame_rate:
        def __get__(self):
            return self._frame_rate
        def __set__(self,val):
            self._frame_rate = val
            self.stop()
            self.set_streamparm()
            self.get_streamparm()

    def get_controls(self):
        cdef v4l2_queryctrl queryctrl
        queryctrl.id = V4L2_CTRL_CLASS_USER | V4L2_CTRL_FLAG_NEXT_CTRL
        controls = []
        control_type = {V4L2_CTRL_TYPE_INTEGER:'int',
                        V4L2_CTRL_TYPE_BOOLEAN:'bool',
                        V4L2_CTRL_TYPE_MENU:'menu'}

        while (0 == self.xioctl(VIDIOC_QUERYCTRL, &queryctrl)):

            if V4L2_CTRL_ID2CLASS(queryctrl.id) != V4L2_CTRL_CLASS_CAMERA:
                #we ignore this conditon
                pass
            control = {}
            control['name'] = queryctrl.name.decode()
            control['type'] = control_type[queryctrl.type]
            control['id'] = queryctrl.id
            control['min'] = queryctrl.minimum
            control['max'] = queryctrl.maximum
            control['step'] = queryctrl.step
            control['default'] = queryctrl.default_value
            control['value'] = self.get_control(queryctrl.id)
            if queryctrl.flags & V4L2_CTRL_FLAG_DISABLED:
                control['disabled'] = True
            else:
                control['disabled'] = False

                if queryctrl.type == V4L2_CTRL_TYPE_MENU:
                    control['menu'] = self.enumerate_menu(queryctrl)

            controls.append(control)

            queryctrl.id |= V4L2_CTRL_FLAG_NEXT_CTRL

        if errno != EINVAL:
            print("VIDIOC_QUERYCTRL")
            # raise Exception("VIDIOC_QUERYCTRL")
        return controls

    cdef enumerate_menu(self,v4l2_queryctrl queryctrl):
        cdef v4l2_querymenu querymenu
        querymenu.id = queryctrl.id
        querymenu.index = queryctrl.minimum
        menu = {}
        while querymenu.index <= queryctrl.maximum:
            if 0 == self.xioctl(VIDIOC_QUERYMENU, &querymenu):
                menu[querymenu.name.decode()] = querymenu.index
            querymenu.index +=1
        return menu

    cpdef set_control(self, int control_id,value):
        cdef v4l2_control control
        control.id = control_id
        control.value = value
        if self.xioctl(VIDIOC_S_CTRL, &control) ==-1:
            if errno == ERANGE:
                print("Control out of range")
            else:
                print("Could not set control")

    cpdef get_control(self, int control_id):
        cdef v4l2_control control
        control.id = control_id
        if self.xioctl(VIDIOC_G_CTRL, &control) ==-1:
            if errno == EINVAL:
                print("Control is not supported")
            else:
                print("Could not set control")
        return control.value

def get_sys_time_monotonic():
    cdef time.timespec t
    time.clock_gettime(time.CLOCK_MONOTONIC, &t)
    return t.tv_sec + <double>t.tv_nsec * 1e-9

def fourcc_string(i):
    s = chr(i & 255)
    for shift in (8,16,24):
        s += chr(i>>shift & 255)
    return s

cpdef __u32 fourcc_u32(char * fourcc):
    return v4l2_fourcc(fourcc[0],fourcc[1],fourcc[2],fourcc[3])