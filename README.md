# 263A_final_project
 From the shell:
mkdir rvctools
cd rvctools
git clone https://github.com/petercorke/robotics-toolbox-matlab.git robot
git clone https://github.com/petercorke/spatial-math.git smtb
git clone https://github.com/petercorke/toolbox-common-matlab.git common
make -C robot

The last command builds the MEX files and Java class files. Then, from within MATLAB
>> addpath rvctools/common  %  rvctools is the same folder as above
>> startup_rvc