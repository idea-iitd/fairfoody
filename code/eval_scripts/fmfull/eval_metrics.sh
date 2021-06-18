#!/bin/bash
for f in $(ls results/*/*/fmfull*); do echo "-----------$f-----------" ; python3 scripts/evaluation_script.py --input_file $f --output_file metrics/$(echo $f | cut -d / -f2- ) --start $(echo $f | cut -d _ -f2 | cut -d '-' -f 1) --end $(echo $f | cut -d '-' -f2 | cut -d '.' -f 1) ; done
