@echo off
REM ===================================================================
REM  build_profile.bat  --  build the -g profiler target and the tools that
REM  give SOURCE-LINE-level timing of the real (-O3 -mavx2) build.
REM
REM  Produces two ways to get line-level profiles (see docs/profiling.md):
REM   * line_profiler.exe + profile_driver.exe  -- self-contained sampling
REM     profiler that reads the DWARF directly (RECOMMENDED; no VS needed).
REM   * profile_driver_vs.exe (+ .pdb via cv2pdb) -- for the Visual Studio
REM     Performance Profiler GUI.
REM
REM  Usage (from the repo root, a normal cmd or the VS "Developer" prompt):
REM     tools\build_profile.bat
REM  Requires MinGW g++ on PATH. Edit MINGW below if yours lives elsewhere.
REM ===================================================================
setlocal

REM --- point this at your MinGW bin if g++ is not already on PATH ---
set "MINGW=E:\dev\claude\tools\mingw64-15.2\bin"
if exist "%MINGW%\g++.exe" set "PATH=%MINGW%;%PATH%"

where g++ >nul 2>nul
if errorlevel 1 (
  echo [error] g++ not found on PATH. Set MINGW at the top of this script.
  exit /b 1
)

REM Normalize ROOT to a full path (collapse the "tools\.."): otherwise the .cpp
REM files are passed to g++ as ...\tools\..\src\foo.cpp, that literal path is
REM baked into the debug info, and Visual Studio then fails to open the .cpp
REM sources ("source not found") while headers found via -I still resolve.
for %%I in ("%~dp0..") do set "ROOT=%%~fI"
set "OUT=%ROOT%\build_profile"
if not exist "%OUT%" mkdir "%OUT%"

echo [1/3] Compiling profile_driver.exe  (-O3 -mavx2 -g, the real perf build + DWARF)...
REM -static* : bundle the MinGW runtime so the exe runs without mingw64\bin on PATH.
REM DWARF is kept in this copy — line_profiler.exe reads it via addr2line.
g++ -O3 -mavx2 -g -std=c++17 -static -static-libgcc -static-libstdc++ ^
    -I "%ROOT%\include" -I "%ROOT%\src" -I "%ROOT%\third_party\stb" ^
    "%ROOT%\tools\profile_driver.cpp" ^
    "%ROOT%\src\gaussian.cpp" "%ROOT%\src\gradient.cpp" "%ROOT%\src\edge.cpp" ^
    "%ROOT%\src\feature.cpp" "%ROOT%\src\labeling.cpp" "%ROOT%\src\sweeplsd.cpp" ^
    "%ROOT%\src\sweeplsd_onepass.cpp" "%ROOT%\src\io.cpp" ^
    -o "%OUT%\profile_driver.exe"
if errorlevel 1 ( echo [error] compile failed & exit /b 1 )

echo [2/3] Compiling line_profiler.exe  (self-contained sampling line profiler)...
g++ -O2 -std=c++17 -static -static-libgcc -static-libstdc++ ^
    "%ROOT%\tools\line_profiler.cpp" -lpsapi -lwinmm -o "%OUT%\line_profiler.exe"
if errorlevel 1 ( echo [error] line_profiler compile failed & exit /b 1 )

echo [3/3] Making a VS copy and converting it to PDB with cv2pdb...
REM cv2pdb STRIPS the DWARF from the image, so run it on a COPY and leave
REM profile_driver.exe (with DWARF) for line_profiler.exe. Flat control flow
REM (goto, not nested parentheses) to keep the batch parser happy.
copy /y "%OUT%\profile_driver.exe" "%OUT%\profile_driver_vs.exe" >nul
if not exist "%ROOT%\tools\cv2pdb\cv2pdb64.exe" goto :nocv2pdb
"%ROOT%\tools\cv2pdb\cv2pdb64.exe" "%OUT%\profile_driver_vs.exe"
if errorlevel 1 echo [warn] cv2pdb failed; VS path unavailable, line_profiler still works.
goto :done
:nocv2pdb
echo [warn] tools\cv2pdb\cv2pdb64.exe missing; skipping the VS/PDB copy. See docs\profiling.md.
:done

echo.
echo Done.
echo.
echo   RECOMMENDED - source-line profile, no Visual Studio needed:
echo      "%OUT%\line_profiler.exe" "%OUT%\profile_driver.exe" ^<image^> --iters 400
echo.
echo   Visual Studio Performance Profiler (GUI):
echo      profile "%OUT%\profile_driver_vs.exe"  (Alt+F2, CPU Usage). See docs\profiling.md.
endlocal
