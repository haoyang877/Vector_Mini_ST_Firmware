@echo off
setlocal
"%~dp0.venv\Scripts\python.exe" -X utf8 "%~dp0tools\run.py" can_link_test %*
exit /b %errorlevel%
