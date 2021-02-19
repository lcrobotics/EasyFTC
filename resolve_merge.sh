#!/usr/bin/env bash
# array of modules to delete
declare -a sdk=('Blocks' 'doc' 'ftcCommon' 'FtcRobotController' 'Hardware' 'Inspection' 'libs' 'OpenRC' 'RobotCore' 'RobotServer' 'TeamCode')

# delete modules
for module in "${sdk[@]}"; do
  rm -rf "$module"
done
# choose master's version of .gradle files
find . -name "*.gradle" | while read file
do
  git checkout --ours "$file"
done
# choose development's version of lib files
find EasyFTCLib -and -not -name "*.gradle" -type f | while read file
do
  git checkout --theirs "$file"
done