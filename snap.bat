echo OFF
rem Backup D:\usr\a\n\Netsensor\NetworkedSensors\projects\ccts\B032-ec  Unit Backup
rem purpose here is to be able to snapshot copy all files in the tree 
rem 
rem
set CdBackup=c:\n\backup
set prename=PlatformIO_Projects_STC3100arduino_
set CdName=%prename%%1.zip
set CdNas=\\stor03\home\backup
dir %CdBackup%\%prename%*
pause
"C:\Program Files\WinZip\wzzip.exe" -aPrexz  -xbackup\.pio %CdBackup%\%CdName% src\*.* examples\*.*  .\platformio.ini

rem move  %CdBackup%\%CdName%  %CdNas%\%CdName%
move  %CdBackup%\%CdName%  %CdNas%\
echo  ** stored in  %CdBackup%\%CdName% and attempted moved to  %CdNas%
