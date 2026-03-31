@echo off
echo Select config:
echo   1. Deadzone
echo   2. Killing Floor 3
echo.
set /p choice="Enter number (or press Enter for default): "

if "%choice%"=="1" FalconJoyForce.exe warzone
if "%choice%"=="2" FalconJoyForce.exe halo
if "%choice%"=="3" FalconJoyForce.exe default
if "%choice%"==""  FalconJoyForce.exe
```

**Step 3 — Each config file just has the button mapping:**
```
# configs\Deadzone.txt
falcon0=RT
falcon1=LT
falcon2=RS
falcon3=Y