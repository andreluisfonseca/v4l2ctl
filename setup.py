__version__='0.2'

import os, platform
import numpy

from distutils.core import setup
from distutils.extension import Extension
from Cython.Build import cythonize
import glob

extensions = [
    Extension(  name="v4l2ctl",
                sources=['v4l2ctl.pyx'],
                include_dirs =  [],
                libraries = ['v4l2'],
                extra_link_args=[],
                extra_objects = [],
                extra_compile_args=[]
            ),
]

setup(
    name='v4l2ctl',
    version=__version__,
    description='Simple controler for v4l2',
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: BSD License',
        'Operating System :: POSIX :: Linux',
        'Programming Language :: Cython',
        'Programming Language :: C',
        'Topic :: Software Development :: Libraries'
    ],
    author='Andre Luis',
    author_email='andreluisfonseca@id.uff.br',
    license='The MIT License (MIT)',
    setup_requires=['Cython >= 0.18'],
    ext_modules=cythonize(extensions)
    
)

