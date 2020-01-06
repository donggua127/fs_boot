cd /d %cd%\..\out
copy %cd%\..\Debug\fs_boot.out %cd%\fs_boot.out
HexAIS_OMAP-L138.exe -ini NandFlash.ini -o fs_boot.ais fs_boot.out