@echo off
rem Host-test build+run helper (VS BuildTools + NMake). Usage:
rem   cmd.exe /c tests\host\run_tests.bat [test-name-filter]
set "RF_VCVARS=C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\VC\Auxiliary\Build\vcvars64.bat"
if not exist "%RF_VCVARS%" set "RF_VCVARS=C:\Program Files\Microsoft Visual Studio\18\Community\VC\Auxiliary\Build\vcvars64.bat"
if not exist "%RF_VCVARS%" (
  echo No supported Visual Studio vcvars64.bat found.
  exit /b 1
)
call "%RF_VCVARS%" >nul
if errorlevel 1 exit /b 1
cmake -S tests/host -B tests/host/build -G "NMake Makefiles" >nul
if errorlevel 1 exit /b 1
cmake --build tests/host/build
if errorlevel 1 exit /b 1
tests\host\build\host_tests.exe %*
