== SignalAnalyzer ==
Author: Avinash Ranganath 
E-mail: nash911@gmail.com
License: GPLv3
License URI: https://www.gnu.org/licenses/quick-guide-gplv3.html

== Description ==
This is a project for extracting signal parameters, for multiple signals, from raw signal data. Parameters such as amplitude, offset, frequency and relative phase difference between signal pairs can be extracted.

A file containing raw signal data is accepted as input to the system. The data file should have the following format,
* The data columns are separated by a white-space.
* The first column contains the time data, while columns 2 to n contain |n-1 signals.
* Each row represents a time value and the corresponding signal(s) data, at one point in time.
* Omits lines that begin with or contain “#”.

Following are the constraints that input signals on data file must follow,
* Signals should be periodic .
* All signals should have the same frequency. 

Following are the features of the system,
* Can accurately estimate amplitude, offset, relative phase difference and frequency of clean signals with an accuracy of three decimal places.
* Can produce a vector of relative phase difference, over time, between a pair of signals, containing the change is phase difference between a pair, over time.
* Can produce output files containing phase difference vector data in plottable format, in folder 'root/Output/'.

== Build and execute ==
Open a terminal and go to the project root directory, and then run the following,
1. cd BUILD
2. cmake ..
3. make
4. ./SignalAnalyzer

  A file, containing signal data, can be provided as a parameter at command line. By default the file 'root/Data/signals.dat' is taken as input.

==Default input file==
The default file 'root/Data/signals.dat' contains four sinusoidal signals that were generated at a time resolution of 0.001 s, and with the following parameters respectively,

Signal 1
----------
Amplitude: 45.0°
Offset: 0.0°
Phase: 0.0°
Frequency: 1.2345678

Signal 2
----------
Amplitude: 20.0°
Offset: 10.0°
Phase: 60.0°
Frequency: 1.2345678

Signal 3
----------
Amplitude: 50.0°
Offset: -15.0°
Phase: -125.0°
Frequency: 1.2345678

Signal 4
----------
Amplitude: 40.0°
Offset: 20.0°
Phase: 1.0°
Frequency: 1.2345678
