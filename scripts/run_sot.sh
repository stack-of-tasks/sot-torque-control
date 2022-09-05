#!/bin/sh
cd /home/adelpret/devel/hrp2-n14-system/_build-RELEASE/tools/direct-access/
chrt --rr 40 ./run_sot_fast_da --input-file /opt/openrobots/lib/libsot-hrp2-14-controller.so --dt=0.0015
