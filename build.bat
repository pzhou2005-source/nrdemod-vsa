@echo off
rem Builds the nrdemod shared library and prepares the Python environment.
rem Usage:  build.bat  [Release|Debug]

setlocal enabledelayedexpansion
set CONFIG=%~1
if "%CONFIG%"=="" set CONFIG=Release

set ROOT=%~dp0
set CORE=%ROOT%nrdemod

if not exist "%CORE%\CMakeLists.txt" (
  echo [ERROR] %CORE% not found. Unpack the full nrdemod-vsa bundle.
  exit /b 1
)

where cmake >nul 2>&1
if errorlevel 1 (
  echo [ERROR] cmake is not on PATH. Install CMake or open a Developer Command Prompt.
  exit /b 1
)

echo === configuring ===
cmake -S "%CORE%" -B "%CORE%\build" -A x64
if errorlevel 1 exit /b 1

echo === building (%CONFIG%) ===
cmake --build "%CORE%\build" --config %CONFIG% -j
if errorlevel 1 exit /b 1

echo === C++ unit test ===
ctest --test-dir "%CORE%\build" -C %CONFIG% --output-on-failure
if errorlevel 1 echo [WARN] C++ test failed

if not exist "%ROOT%.venv\Scripts\python.exe" (
  echo === creating .venv ===
  py -3 -m venv "%ROOT%.venv" || python -m venv "%ROOT%.venv"
)
echo === installing python dependencies ===
"%ROOT%.venv\Scripts\python.exe" -m pip install --upgrade pip
"%ROOT%.venv\Scripts\python.exe" -m pip install -r "%ROOT%requirements.txt"

echo === self test ===
"%ROOT%.venv\Scripts\python.exe" "%ROOT%tools\correlate.py" selftest
if errorlevel 1 (
  echo [ERROR] self test failed
  exit /b 1
)

echo.
echo Done. Next:
echo   .venv\Scripts\python.exe tools\vsa_link.py --probe --host 127.0.0.1
echo   .venv\Scripts\python.exe tools\correlate.py live --config configs\fr1_100mhz_mu1_64qam.json
endlocal
