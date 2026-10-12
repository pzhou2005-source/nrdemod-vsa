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

rem Pick a toolchain: MSVC (cl.exe) if a Developer Command Prompt is active, otherwise a
rem real mingw-w64 compiler. Never use cygwin's g++ here: it crashes native (non-cygwin)
rem processes that load the resulting DLL via ctypes (see memory/windows-build-tooling notes).
set GENERATOR=
set CXX_COMPILER=
where cl >nul 2>&1
if not errorlevel 1 (
  set GENERATOR=-A x64
) else (
  for /f "delims=" %%G in ('where g++ 2^>nul') do (
    echo %%G | findstr /i "cygwin" >nul
    if errorlevel 1 if "!CXX_COMPILER!"=="" set CXX_COMPILER=%%G
  )
  if "!CXX_COMPILER!"=="" (
    for /d %%D in ("%LOCALAPPDATA%\Microsoft\WinGet\Packages\*WinLibs*") do (
      if exist "%%D\mingw64\bin\g++.exe" set CXX_COMPILER=%%D\mingw64\bin\g++.exe
    )
  )
  if "!CXX_COMPILER!"=="" (
    echo [ERROR] no usable C++ compiler found. Install a Developer Command Prompt for MSVC,
    echo         or a mingw-w64 toolchain ^(e.g. winget install BrechtSanders.WinLibs.POSIX.UCRT^).
    exit /b 1
  )
  set GENERATOR=-G "MinGW Makefiles" -DCMAKE_CXX_COMPILER=!CXX_COMPILER!
)

rem A build directory configured for a different generator/platform refuses to reconfigure.
if exist "%CORE%\build\CMakeCache.txt" (
  findstr /i /c:"CMAKE_CXX_COMPILER:STRING=!CXX_COMPILER!" "%CORE%\build\CMakeCache.txt" >nul
  if errorlevel 1 rmdir /s /q "%CORE%\build"
)

echo === configuring ===
cmake -S "%CORE%" -B "%CORE%\build" %GENERATOR%
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
