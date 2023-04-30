@echo off

del *.bak /S

cd fpga
call _clean.bat
cd ..

cd demo-fatfs
call _clean.bat
cd ..

cd demo-fatfs-tcts
call _clean.bat
cd ..
