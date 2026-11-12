@echo off
setlocal
pushd "%~dp0"
if "%~1"=="" (
    ".venv\Scripts\python.exe" -X utf8 tools\run.py install_axis_board
) else (
    ".venv\Scripts\python.exe" -X utf8 tools\run.py install_axis_board --bundle "%~f1"
)
set "axis_result=%errorlevel%"
if not "%axis_result%"=="0" pause
popd
exit /b %axis_result%
