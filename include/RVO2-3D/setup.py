from setuptools import setup, Extension
from setuptools.command.build_ext import build_ext as _build_ext
from Cython.Build import cythonize
import os
import os.path
import subprocess


class BuildRvo23DExt(_build_ext):
    """Builds rvo_wrapper before our module."""

    def run(self):
        # Build rvo_wrapper

        build_dir = os.path.abspath('build/rvo_wrapper')
        if not os.path.exists(build_dir):
            os.makedirs(build_dir)

            # Set generator
            if os.name == 'nt':
                
                # Use msvc 64 bit
                subprocess.check_call(['cmake', '-G', 'Visual Studio 14 2015 Win64', '../..'],
                                      cwd=build_dir)
            else:
                subprocess.check_call(['cmake', '../..', '-DCMAKE_CXX_FLAGS=-fPIC'],
                                      cwd=build_dir)
								  

        if os.name == 'nt':
            subprocess.check_call(['cmake', '--build', '.', '--config', 'Release'], cwd=build_dir)
        else:
            subprocess.check_call(['cmake', '--build', '.'], cwd=build_dir)
		
        print("Run building extension")
        _build_ext.run(self)




# MSVC on windows
if os.name == 'nt':
    extensions = [
        Extension('rvo_wrapper', ['src/rvo_wrapper.pyx'],
              include_dirs=['src'],
              libraries=['RVO'],
              library_dirs=['build/rvo_wrapper/src/Release'],
              extra_compile_args=[],
	      extra_link_args=[]),
]
# GCC on linux
else:
    extensions = [
        Extension('rvo_wrapper', ['src/rvo_wrapper.pyx'],
              include_dirs=['src'],
              libraries=['RVO'],
              library_dirs=['build/rvo_wrapper/src'],
              extra_compile_args=['-fPIC'],
	      extra_link_args=[]),
]

setup(
    name="pyrvo23d",
    version='0.1.0',
    ext_modules=cythonize(extensions),
    cmdclass={'build_ext': BuildRvo23DExt},
    classifiers=[
        'Development Status :: 5 - Production/Stable',
        'Intended Audience :: Developers',
        'Intended Audience :: Education',
        'Intended Audience :: Information Technology',
        'Operating System :: OS Independent',
        'Programming Language :: Python',
        'Programming Language :: Python :: 2.7',
        'Programming Language :: Python :: 3',
        'Programming Language :: Python :: 3.4',
        'Programming Language :: Cython',
        'Topic :: Games/Entertainment :: Simulation',
        'Topic :: Software Development :: Libraries :: Python Modules',
    ],
)
