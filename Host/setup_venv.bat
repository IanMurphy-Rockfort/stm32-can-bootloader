echo off
echo This script will setup a Python Virtual Environment for the Host, including installing required libraries...
echo on
pause
python -m venv venv
call .\venv\Scripts\activate
pip install pyserial
pip install pyinstaller
pip list
pause
