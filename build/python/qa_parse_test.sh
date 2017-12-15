#!/bin/sh
export VOLK_GENERIC=1
export GR_DONT_LOAD_PREFS=1
export srcdir=/home/ljwong/software_radios/final_project/gr-bluetooth5/python
export GR_CONF_CONTROLPORT_ON=False
export PATH=/home/ljwong/software_radios/final_project/gr-bluetooth5/build/python:$PATH
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH
export PYTHONPATH=/home/ljwong/software_radios/final_project/gr-bluetooth5/build/swig:$PYTHONPATH
/usr/bin/python2 /home/ljwong/software_radios/final_project/gr-bluetooth5/python/qa_parse.py 
