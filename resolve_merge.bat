@echo off


for %%folder in (Blocks doc ftcCommon FtcRobotController Hardware Inspection libs OpenRC RobotCore RobotServer TeamCode) do
    RD /S /Q %%folder

for /R %%file in (*.gradle) do
    call git checkout --ours %%file

for /R EasyFTCLib\ %%lib in (.) do
    call git checkout --theirs %%lib
