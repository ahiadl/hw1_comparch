#!/bin/bash

./sim_main tests/example2.img 43 > actualResults/example2.txt
./sim_main tests/example2.img 43 -s > actualResults/example2-s.txt
./sim_main tests/example2.img 43 -f > actualResults/example2-f.txt
./sim_main tests/example2.img 100 > actualResults/example2_nolimit.txt
./sim_main tests/example2.img 100 -s > actualResults/example2-s_nolimit.txt
./sim_main tests/example2.img 100 -f > actualResults/example2-f_nolimit.txt
./sim_main tests/example3.img 43 > actualResults/example3.txt
./sim_main tests/example3.img 43 -s > actualResults/example3-s.txt
./sim_main tests/example3.img 43 -f > actualResults/example3-f.txt
./sim_main tests/example3.img 100 > actualResults/example3_nolimit.txt
./sim_main tests/example3.img 100 -s > actualResults/example3-s_nolimit.txt
./sim_main tests/example3.img 100 -f > actualResults/example3-f_nolimit.txt
./sim_main tests/example4.img 43 > actualResults/example4.txt
./sim_main tests/example4.img 43 -s > actualResults/example4-s.txt
./sim_main tests/example4.img 43 -f > actualResults/example4-f.txt
./sim_main tests/example4.img 100 > actualResults/example4_nolimit.txt
./sim_main tests/example4.img 100 -s > actualResults/example4-s_nolimit.txt
./sim_main tests/example4.img 100 -f > actualResults/example4-f_nolimit.txt
