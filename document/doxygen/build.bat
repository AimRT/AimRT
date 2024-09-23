@echo off
setlocal

set "DIR=%~1"
if "%DIR%"=="" set "DIR=."

where doxygen >nul 2>&1
if %ERRORLEVEL% == 0 (
  if exist "%DIR%\html" (
    rmdir /s /q "%DIR%\html"
  )
  doxygen "%DIR%\Doxyfile" > "%DIR%\doxygen.log" 2>&1
)

endlocal
