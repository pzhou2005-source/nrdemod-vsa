@echo off
rem Build GNU Radio OOT modules from source against the conda-forge "gnuradio" env (MSVC + Ninja).
rem Usage: tools\gnuradio\build_oot.bat [source_dir]   (default C:\code\gr-oot)
setlocal EnableDelayedExpansion
set "SRC=%~1"
if "%SRC%"=="" set "SRC=C:\code\gr-oot"
set "CONDA=%LOCALAPPDATA%\miniforge3"

call "%CONDA%\Scripts\activate.bat" gnuradio || (echo conda env "gnuradio" not found & exit /b 1)
set "VSWHERE=%ProgramFiles(x86)%\Microsoft Visual Studio\Installer\vswhere.exe"
for /f "usebackq delims=" %%i in (`"%VSWHERE%" -latest -products * -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath`) do set "VSDIR=%%i"
if not defined VSDIR (echo MSVC C++ build tools not found & exit /b 1)
call "%VSDIR%\VC\Auxiliary\Build\vcvars64.bat" >nul || exit /b 1

if not exist "%SRC%" mkdir "%SRC%"
set "FAILED="
rem gr-foo / gr-ieee802-11 / gr-ieee802-15-4 / gr-lora_sdr ship prebuilt for win-64 on the ryanvolz channel:
rem   conda install -n gnuradio -c conda-forge -c ryanvolz gnuradio-ieee802_11 gnuradio-ieee802_15_4 gnuradio-lora_sdr
rem name  repo  subdir-with-CMakeLists
call :module uwb-gnuradio     https://github.com/junqi-ma/uwb-gnuradio.git   gr-uwb

echo.
if defined FAILED (echo FAILED:!FAILED! & exit /b 1)
echo All OOT modules built and installed into %CONDA_PREFIX%
exit /b 0

:module
set "DIR=%SRC%\%~1"
if not exist "%DIR%\.git" (git clone --depth 1 %2 "%DIR%" || goto :fail)
cmake -S "%DIR%\%~3" -B "%DIR%\build" -G Ninja -DCMAKE_BUILD_TYPE=Release ^
  -DCMAKE_INSTALL_PREFIX="%CONDA_PREFIX%\Library" -DCMAKE_PREFIX_PATH="%CONDA_PREFIX%\Library" ^
  -DPython_EXECUTABLE="%CONDA_PREFIX%\python.exe" -DPYTHON_EXECUTABLE="%CONDA_PREFIX%\python.exe" ^
  -DGR_PYTHON_DIR="%CONDA_PREFIX%\Lib\site-packages" -DENABLE_DOXYGEN=OFF -DENABLE_TESTING=OFF ^
  > "%DIR%\build.log" 2>&1 || goto :fail
cmake --build "%DIR%\build" >> "%DIR%\build.log" 2>&1 || goto :fail
cmake --install "%DIR%\build" >> "%DIR%\build.log" 2>&1 || goto :fail
echo [OK]   %~1
exit /b 0
:fail
echo [FAIL] %~1  (see %DIR%\build.log)
set "FAILED=!FAILED! %~1"
exit /b 1
